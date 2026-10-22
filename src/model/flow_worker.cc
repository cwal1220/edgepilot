#include "model/flow_worker.h"

#include "common/utils_math.h"  // kRadToDeg
#include "maixcam2/maix_ive_flow.h"

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <memory>

namespace {

constexpr size_t kQueueDepth = 3;
constexpr uint64_t kLeadLogFrames = 1200;

}  // namespace

FlowWorker::FlowWorker(const FlowIntrinsics &k, int width, int height, const FlowBlocks *restore)
    : tracker_(k, width, height, std::make_unique<IveFlow>(width, height)) {
    // 저장해 둔 블록은 CalibrationService가 같은 restore로 이미 걸렀다(잘못된 블록은 비워 둔다)
    if (restore && !restore->yaw.empty()) {
        tracker_.restore(*restore);
        std::fprintf(stderr, "modeld: flow calibration restored %zu blocks\n", restore->yaw.size());
    }
    state_ = tracker_.calibrator().state();
    blocks_ = tracker_.calibrator().blocks();
    thread_ = std::thread(&FlowWorker::run, this);
}

void FlowWorker::request_reset() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        reset_requested_ = true;
    }
    cv_.notify_one();
}

bool FlowWorker::take_blocks(FlowBlocks *out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!blocks_changed_) return false;
    *out = blocks_;
    blocks_changed_ = false;
    return true;
}

FlowWorker::~FlowWorker() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_one();
    if (thread_.joinable()) thread_.join();
}

void FlowWorker::push(FlowFrame frame) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= kQueueDepth) {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(frame));
    }
    cv_.notify_one();
}

FlowCalibrationState FlowWorker::state(uint32_t *resets_applied) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (resets_applied) *resets_applied = resets_applied_;
    return state_;
}

LeadFlowState FlowWorker::lead_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lead_state_;
}

void FlowWorker::run() {
    // modeld는 nice -15로 돈다. 이 스레드는 IVE를 기다리는 시간이 대부분이라 보통 우선순위면 된다.
    setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 0);
    uint32_t logged_blocks = tracker_.calibrator().state().blocks;
    uint32_t logged_mount_changes = tracker_.calibrator().state().mount_changes;
    uint64_t logged_lead_frames = 0;
    for (;;) {
        FlowFrame f;
        bool reset = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return stop_ || reset_requested_ || !queue_.empty(); });
            if (stop_) return;
            reset = reset_requested_;
            reset_requested_ = false;
            if (!reset) {
                f = std::move(queue_.front());
                queue_.pop_front();
            }
        }
        if (reset) {
            tracker_.reset_calibration();
            const FlowCalibrationState st = tracker_.calibrator().state();
            logged_blocks = st.blocks;
            logged_mount_changes = st.mount_changes;
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = st;
            blocks_ = FlowBlocks{};
            blocks_changed_ = false;  // 저장 파일은 보정 서비스가 지운다
            ++resets_applied_;
            std::fprintf(stderr, "flowcal: reset, collecting from scratch\n");
            continue;
        }
        tracker_.update(f);
        const FlowCalibrationState st = tracker_.calibrator().state();
        const LeadFlowState lead = tracker_.lead_state();
        const bool block_done = st.blocks != logged_blocks;
        uint64_t dropped = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = st;
            lead_state_ = lead;
            dropped = dropped_;
            if (block_done) {
                blocks_ = tracker_.calibrator().blocks();
                blocks_changed_ = true;
            }
        }
        const FlowTracker::Stats &s = tracker_.stats();
        const double frames = static_cast<double>(std::max<uint64_t>(s.frames, 1));
        if (s.lead_frames >= logged_lead_frames + kLeadLogFrames) {  // 앞차를 따라간 1분마다
            logged_lead_frames = s.lead_frames;
            std::fprintf(stderr,
                         "leadflow: frames %llu with rate %llu (%.0f%%) | last rate %+.3f/s relative %+.2f m/s at "
                         "%.1f m, %u/%u points | %.1f ms wall %.2f ms cpu per frame\n",
                         static_cast<unsigned long long>(s.lead_frames), static_cast<unsigned long long>(s.lead_rates),
                         100.0 * static_cast<double>(s.lead_rates) / static_cast<double>(s.lead_frames),
                         lead.expansion_rate, -lead.lead_x * lead.expansion_rate, lead.lead_x, lead.inliers,
                         lead.points, s.wall_ms / frames, s.cpu_ms / frames);
        }
        if (!block_done) continue;
        logged_blocks = st.blocks;
        if (st.mount_changes != logged_mount_changes) {
            logged_mount_changes = st.mount_changes;
            std::fprintf(stderr, "flowcal: mount change detected (%u so far), kept the last %u blocks\n",
                         st.mount_changes, st.estimate_blocks);
        }
        std::fprintf(stderr,
                     "flowcal: block %u windows %u yaw %+.3f pitch %+.3f deg (sd %.3f/%.3f, %u blocks, %s) | "
                     "last yaw %+.3f pitch %+.3f rot %.3f deg tracks %u inliers %u parallax %.1f px | frames %llu "
                     "done %llu lost %llu rejected %llu dropped %llu | %.1f ms wall %.2f ms cpu per frame\n",
                     st.blocks, st.windows, st.yaw * kRadToDeg, st.pitch * kRadToDeg, st.sd_yaw * kRadToDeg,
                     st.sd_pitch * kRadToDeg, st.estimate_blocks,
                     st.status == static_cast<uint32_t>(FlowCalibrationStatus::Estimate) ? "estimate" : "collecting",
                     st.window_yaw * kRadToDeg, st.window_pitch * kRadToDeg, st.window_rotation * kRadToDeg,
                     st.window_tracks, st.window_inliers, st.window_parallax, static_cast<unsigned long long>(s.frames),
                     static_cast<unsigned long long>(s.windows_done), static_cast<unsigned long long>(s.windows_lost),
                     static_cast<unsigned long long>(s.rejected), static_cast<unsigned long long>(dropped),
                     s.wall_ms / frames, s.cpu_ms / frames);
    }
}
