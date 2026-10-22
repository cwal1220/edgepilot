#include "model/flow_tracker.h"

#include <algorithm>
#include <ctime>
#include <utility>

#include "common/utils_time.h"

namespace {

// 칸 후보의 문턱(최댓값 대비). 밤에는 불빛이 최댓값을 차지해 1%로는 37~46점만 남는 구간이 있었다
// (2026-10 보드 재생). 칸마다 하나만 고르므로 문턱을 낮춰도 점은 고르게 퍼진다.
constexpr double kCornerQuality = 0.002;
// 보정 점과 앞차 점을 한 번의 LK로 따라간다
constexpr int kStepPoints = kFlowMaxPoints + kLeadMaxPoints;
static_assert(kStepPoints <= FlowEngine::kMaxPoints, "one LK call tracks both point sets");

double thread_cpu_ms() {
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) * 1e3 + static_cast<double>(ts.tv_nsec) / 1e6;
}

}  // namespace

FlowTracker::FlowTracker(const FlowIntrinsics &k, int width, int height, std::unique_ptr<FlowEngine> engine)
    : k_(k),
      engine_(std::move(engine)),
      lead_(k, width, height),
      start_(2 * kFlowMaxPoints),
      cur_(2 * kFlowMaxPoints),
      alive_(kFlowMaxPoints),
      step_in_(2 * kStepPoints),
      step_out_(2 * kStepPoints),
      step_ok_(kStepPoints),
      back_(2 * kLeadMaxPoints),
      back_ok_(kLeadMaxPoints) {
    live_.reserve(kFlowMaxPoints);
    fit_a_.reserve(2 * kFlowMaxPoints);
    fit_b_.reserve(2 * kFlowMaxPoints);
}

FlowTracker::~FlowTracker() = default;

void FlowTracker::start_window() {
    const int16_t *resp = engine_->corner_response();
    n_ = resp ? pick_flow_corners(resp, engine_->width() / 2, FlowEngine::kBandRows, 2, FlowEngine::kBandTop,
                                  kCornerQuality, kFlowMaxPoints, start_.data())
              : 0;
    std::copy(start_.begin(), start_.begin() + 2 * n_, cur_.begin());
    std::fill(alive_.begin(), alive_.begin() + n_, 1);
    steps_ = 0;
    in_window_ = n_ >= kFlowMinTracks;
}

// 이번 걸음에 따라간 보정 점(step_out_ 앞쪽 tracked개)으로 창을 잇고, 창이 차면 이동 방향을 푼다.
void FlowTracker::advance_window(uint64_t frame_id, int tracked) {
    int alive = 0;
    for (int j = 0; j < tracked; ++j) {
        const int i = live_[j];
        if (step_ok_[j]) {
            cur_[2 * i] = step_out_[2 * j];
            cur_[2 * i + 1] = step_out_[2 * j + 1];
            ++alive;
        } else {
            alive_[i] = 0;
        }
    }
    ++steps_;
    if (alive < kFlowMinTracks) {
        ++stats_.windows_lost;
        in_window_ = false;
        return;
    }
    if (steps_ < kFlowWindowSteps) return;
    fit_a_.clear();
    fit_b_.clear();
    for (int i = 0; i < n_; ++i) {
        if (!alive_[i]) continue;
        fit_a_.push_back(start_[2 * i]);
        fit_a_.push_back(start_[2 * i + 1]);
        fit_b_.push_back(cur_[2 * i]);
        fit_b_.push_back(cur_[2 * i + 1]);
    }
    const FlowSolution s = solve_flow_direction(k_, fit_a_.data(), fit_b_.data(), alive);
    if (s.valid) {
        calibrator_.add(s, alive, frame_id);
        ++stats_.windows_done;
    } else {
        calibrator_.note_rejected(s);
        ++stats_.rejected;
    }
    in_window_ = false;
}

void FlowTracker::update(const FlowFrame &f) {
    auto lose = [this] {
        if (in_window_) ++stats_.windows_lost;
        in_window_ = false;
    };
    const bool gate_open = flow_gate_open(f.gate);
    const bool lead_wanted = f.y_virt != nullptr && lead_flow_wanted(f.lead);
    if (!gate_open) lose();
    if (!lead_wanted) lead_.reset();
    if (!gate_open && !lead_wanted) {  // 곧은 길도 앞차도 아니면 IVE를 쓰지 않는다
        have_last_ = false;
        return;
    }
    const double wall0 = monotonic_now_ns() * 1e-6, cpu0 = thread_cpu_ms();
    const bool consecutive = have_last_ && f.frame_id == last_frame_ + 1;
    if (!engine_->push(f.y_phys) || (f.still_valid && !f.still_valid())) {
        lose();
        lead_.reset();
        have_last_ = false;
        return;
    }
    const std::function<bool()> prev_valid = consecutive ? last_valid_ : std::function<bool()>();
    have_last_ = true;
    last_frame_ = f.frame_id;
    last_valid_ = f.still_valid;
    ++stats_.frames;
    if (in_window_ && !consecutive) lose();  // 이동이 8 px 한계를 넘을 수 있다
    const bool lead_points =
        lead_wanted && lead_.begin(f.lead, f.frame_id, f.t_s, consecutive, f.y_virt, f.stride, f.invalidate);

    // 살아 있는 보정 점 다음에 앞차 점을 모아 한 번에 따라간다
    live_.clear();
    if (in_window_) {
        for (int i = 0; i < n_; ++i) {
            if (!alive_[i]) continue;
            const size_t m = live_.size();
            step_in_[2 * m] = cur_[2 * i];
            step_in_[2 * m + 1] = cur_[2 * i + 1];
            live_.push_back(i);
        }
    }
    const int cal = static_cast<int>(live_.size());
    const int lead = lead_points ? lead_.points(&step_in_[2 * static_cast<size_t>(cal)]) : 0;
    if (cal + lead > 0) {
        const float *lead_out = &step_out_[2 * static_cast<size_t>(cal)];
        bool tracked = engine_->track(step_in_.data(), cal + lead, step_out_.data(), step_ok_.data());
        // 앞차 점은 거꾸로도 따라가 본다(앞뒤 검사, LeadFlowTracker::advance)
        if (tracked && lead > 0) tracked = engine_->track(lead_out, lead, back_.data(), back_ok_.data(), true);
        // 원본 단은 링 슬롯을 그대로 읽었으므로 LK 동안 두 슬롯이 덮어써지지 않았어야 한다
        if (!tracked || (prev_valid && !prev_valid()) || (f.still_valid && !f.still_valid())) {
            lose();
            lead_.reset();
        } else {
            if (cal > 0) advance_window(f.frame_id, cal);
            if (lead > 0) {
                lead_.advance(lead_out, &step_ok_[static_cast<size_t>(cal)], back_.data(), back_ok_.data());
                ++stats_.lead_frames;
                if (lead_.state().valid) ++stats_.lead_rates;
            }
        }
    }
    // 창이 없으면(방금 끝났어도) 이 프레임에서 새 창을 연다
    if (gate_open && !in_window_) start_window();
    if (lead_wanted) {
        lead_.top_up();
        // begin이 상자 둘레를 복사한 뒤 슬롯이 덮어써졌으면 새 점과 템플릿을 믿을 수 없다
        if (f.still_valid && !f.still_valid()) lead_.reset();
    }
    stats_.wall_ms += monotonic_now_ns() * 1e-6 - wall0;
    stats_.cpu_ms += thread_cpu_ms() - cpu0;
}

void FlowTracker::reset_calibration() {
    calibrator_.reset();
    in_window_ = false;
}
