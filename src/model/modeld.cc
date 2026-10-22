// modeld: 카메라 프레임 링의 최신 프레임마다 supercombo를 NPU에서 돌려 ModelState를 낸다(20 Hz). 입력 워프는 GDC,
// 보정은 CalibrationService(포즈 또는 광류), 광류 보정과 앞차 확대율은 광류 워커 스레드(IVE)가 맡는다. 녹화할
// 프레임은 recordFrame으로 알린다.

#include "common/app_config.h"
#include "common/build_version.h"
#include "common/device_settings.h"
#include "common/ipc_channels.h"
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "common/utils_process.h"
#include "common/utils_time.h"
#include "maixcam2/maix_cmm.h"
#include "model/calibration_service.h"
#include "model/flow_worker.h"
#include "model/lead_tracker.h"
#include "model/model_state_fill.h"
#include "model/supercombo_model.h"

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// 웹 기기 설정(display.json calibration_use_flow)이 고른 보정 출처
CalibrationSource requested_calibration_source(const DeviceSettings &settings) {
    return settings.calibration_use_flow ? CalibrationSource::Flow : CalibrationSource::Pose;
}

volatile sig_atomic_t g_stop = 0;

// controlsd 스냅샷에서 모델 입력용 자차 속도와 desire, 광류 보정기의 직진 조건(곡률·깜빡이), 결합 상태(보정
// 출처 설정을 해제까지 미룬다)를 읽는다. controlsd가 아직 없으면 열릴 때까지 마지막 값을 유지한다. 스냅샷이
// kStaleNs보다 오래되면(controlsd가 죽거나 멈춤) 차 상태를 낡은 것으로 보고 결합도 풀린 것으로 본다: 마지막
// 속도로 광류 보정의 직진 조건이 계속 열려 있지 않게 한다.
class EgoStateReader {
public:
    static constexpr uint64_t kStaleNs = 1'000'000'000ULL;

    void poll() {
        if (!open_) open_ = sub_.open(kControlStateTopic, sizeof(ControlState), false);
        if (!open_) return;
        ControlState control_state;
        const bool read = sub_.read(&control_state, sizeof(control_state));
        if (read) last_stamp_ns_ = control_state.timestamp_ns;
        if (!timestamp_fresh_ns(last_stamp_ns_, monotonic_now_ns(), kStaleNs)) {
            gate_.vehicle_fresh = false;
            engaged_ = false;
            return;
        }
        if (!read) return;
        const float ego_speed_kph =
            control_state.ego_speed_kph > 0.0f ? control_state.ego_speed_kph : control_state.cluster_speed_kph;
        v_ego_ = std::max(0.0f, ego_speed_kph / 3.6f);
        desire_ = static_cast<int>(control_state.desire);
        gate_.v_ego = v_ego_;
        gate_.curvature = control_state.actual_curvature;
        gate_.blinker = control_state.left_blinker != 0 || control_state.right_blinker != 0;
        gate_.desire = control_state.desire != 0;
        gate_.vehicle_fresh = control_state.vehicle_fresh != 0;
        engaged_ = control_state.engaged != 0;
    }
    float v_ego() const { return v_ego_; }
    bool vehicle_fresh() const { return gate_.vehicle_fresh; }
    int desire() const { return desire_; }
    bool engaged() const { return engaged_; }
    const FlowGateInput &flow_gate() const { return gate_; }

private:
    LatestChannel sub_;
    bool open_ = false;
    uint64_t last_stamp_ns_ = 0;
    float v_ego_ = 0.0f;
    int desire_ = 0;
    bool engaged_ = false;
    FlowGateInput gate_;
};

// 웹 기기 설정(display.json). modeld에서는 이것만 파일을 읽어(1초에 한 번 stat) 카메라 장착과 보정 출처가 같은
// 값을 받는다.
class ModeldSettings {
public:
    ModeldSettings() {
        file_.poll(monotonic_now_ns(), &settings_);
        log();
    }

    // 프레임마다 한 번
    void poll() {
        if (file_.poll(monotonic_now_ns(), &settings_)) log();
    }
    const DeviceSettings &get() const { return settings_; }

private:
    void log() const {
        std::fprintf(stderr, "modeld: settings camera offset=%+.3f m height=%.2f m, calibration source %s\n",
                     settings_.camera_offset_m, settings_.camera_height_m,
                     settings_.calibration_use_flow ? "flow" : "pose");
    }

    DeviceSettingsFile file_;
    DeviceSettings settings_;
};

// 카메라 장착(기기 설정 camera_offset_m·camera_height_m). 바뀐 오프셋과 높이는 초당 kRateMps로 옮긴다: 모델은 t−4·t
// 두 프레임과 4.8초 특징 이력을 보므로 한 번에 바꾸면 차가 옆으로 순간 이동한 것처럼 보여 pose·보정·locationd·경로가
// 튄다(보정 워프도 같은 이유로 천천히 옮긴다, model/calibration_source). 높이도 오프셋이 있으면 워프의 옆 이동을
// 바꾸므로 같이 옮긴다(예전에는 높이만 한 번에 바꿨다). 시작할 때는 첫 프레임부터 그대로 쓴다.
class CameraMount {
public:
    static constexpr float kRateMps = 0.1f;

    explicit CameraMount(const DeviceSettings &start)
        : offset_m_(start.camera_offset_m), height_m_(start.camera_height_m) {}

    // 매 프레임 실행 직전. 이번 프레임 워프에 쓴 값이 offset_m()·height_m()이다.
    void apply(SupercomboModel &model, const DeviceSettings &settings) {
        const float step = kRateMps * kDtMdlF;
        offset_m_ += std::clamp(settings.camera_offset_m - offset_m_, -step, step);
        height_m_ += std::clamp(settings.camera_height_m - height_m_, -step, step);
        model.set_camera_mount(offset_m_, height_m_);
    }
    float offset_m() const { return offset_m_; }
    float height_m() const { return height_m_; }

private:
    float offset_m_ = 0.0f;
    float height_m_ = 0.0f;
};

// 앞차 쪽 입출력
struct LeadIo {
    LeadFlowState flow{};                                     // 광류 작업이 마지막으로 낸 확대율
    LeadFlowInput *next = nullptr;                            // 광류 작업이 다음 카메라 프레임에서 따라갈 앞차(채운다)
    LeadTracker *tracker = nullptr;                           // 앞차 추적기(ModelState.lead_track)
    float ego_mps = std::numeric_limits<float>::quiet_NaN();  // 자차 속도, 모르면 NaN(추적 시작에만 쓴다)
};

// 모델 쪽: 모델, 기기 설정, 보정, 카메라 장착(이 순서로 만든다. 모델의 세션이
// AX_SYS_Init을 한다). 프레임마다 prepare_frame → 실행 → publish.
struct ModelLoop {
    SupercomboModel model;
    ModeldSettings settings;
    CalibrationService calibration;
    CameraMount mount;

    explicit ModelLoop(const AppConfig &config)
        : model(config.axmodel_path.c_str(), config),
          calibration(config, requested_calibration_source(settings.get())),
          mount(settings.get()) {}

    // 실행 직전. 워프 보정은 보정 서비스의 지금 값이다: 첫 프레임은 시작 값, 그 뒤는 직전 프레임 출력으로
    // publish가 갱신한 값(openpilot cameraOdometry -> liveCalibration -> modeld 고리). 서비스는 publish에서만 바뀐다.
    void prepare_frame() {
        float rpy[3];
        calibration.input_rpy(rpy);
        model.set_input_calibration(rpy);
        mount.apply(model, settings.get());
    }

    // engaged는 controlsd 결합 상태. 워프에 쓴 보정(출처와 상관없이)이 calibration, 출처와 포즈 보정기 자신의
    // 추정이 calibration_source, 광류 상태가 flow_calibration이다.
    void publish(LatestChannel &model_pub, const ParsedModelOutput &parsed, uint64_t frame_id,
                 uint64_t capture_timestamp_ns, float model_ms, float v_ego, bool engaged, const LeadIo &lead);
};

void ModelLoop::publish(LatestChannel &model_pub, const ParsedModelOutput &parsed, uint64_t frame_id,
                        uint64_t capture_timestamp_ns, float model_ms, float v_ego, bool engaged, const LeadIo &lead) {
    calibration.update(parsed, v_ego, engaged, requested_calibration_source(settings.get()));
    float input_rpy[3];  // 다음 카메라 프레임의 워프 보정(광류 작업의 앞차 상자도 이것으로 투영한다)
    calibration.input_rpy(input_rpy);

    const ProjectionState projection = calibration.projection();

    ModelState state =
        fill_model_state(parsed, projection, calibration.snapshot(), frame_id, capture_timestamp_ns, model_ms);
    state.camera_offset_m = mount.offset_m();
    state.camera_height_m = mount.height_m();
    state.flow_calibration = calibration.flow_state();
    state.calibration_source = calibration.source_state();
    state.lead_flow = lead.flow;
    if (lead.tracker) {  // 모델 앞차 거리와 확대율을 섞는다(비전 크루즈가 쓴다)
        LeadTrackInput in;
        in.t_s = static_cast<double>(capture_timestamp_ns) * 1e-9;
        in.lead_valid = lead_usable(state.lead);
        in.distance_m = state.lead.x;
        in.relative_speed_mps = state.lead.velocity - lead.ego_mps;
        in.flow_valid = state.lead_flow.valid != 0;
        in.flow_frame_id = state.lead_flow.frame_id;
        in.flow_rate = state.lead_flow.expansion_rate;
        lead.tracker->update(in);
        state.lead_track = lead.tracker->state();
    }
    if (lead.next) {  // 광류 작업이 다음 카메라 프레임에서 따라갈 앞차
        lead.next->valid = state.lead.valid != 0;
        lead.next->probability = state.lead.probability;
        lead.next->x_m = state.lead.x;
        lead.next->y_m = state.lead.y;
        std::copy(input_rpy, input_rpy + 3, lead.next->rpy);
        lead.next->camera_height_m = mount.height_m();
        lead.next->camera_offset_m = mount.offset_m();
    }
    model_pub.publish(&state, sizeof(state));
}

// 광류 작업이 앞차 점을 고를 때 읽는 링 슬롯의 CPU 매핑(캐시, 읽을 행만 비운다). camerad가 다시 떠서 물리 주소가
// 바뀌면 다시 매핑한다.
struct RingSlotMaps {
    unsigned long long phys[kFrameSlots] = {};
    uint8_t *virt[kFrameSlots] = {};
    size_t size = 0;

    void attach(const FrameRing &ring, unsigned slot) {
        const unsigned long long p = ring.slot_phys(slot);
        if (p == phys[slot] && virt[slot]) return;
        if (virt[slot]) cmm_unmap(virt[slot], size);
        size = ring.frame_bytes();
        virt[slot] = cmm_map(p, size);
        phys[slot] = p;
    }
    ~RingSlotMaps() {
        for (unsigned i = 0; i < kFrameSlots; ++i) cmm_unmap(virt[i], size);
    }
};

int run_live(const AppConfig &config, LatestChannel &model_pub, LatestChannel &record_frame_pub) {
    LatestChannel frame_sub;
    FrameRing frame_ring;
    if (!frame_sub.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true))
        throw std::runtime_error("open roadAiFrame ipc failed");

    while (!g_stop && !frame_ring.open(false)) {
        std::fprintf(stderr, "modeld: waiting for road ai frame ring\n");
        usleep(500000);
    }
    if (!frame_ring.valid()) return 1;

    ModelLoop loop(config);
    EgoStateReader ego;
    // 링 슬롯의 CPU 매핑(광류 작업의 앞차 점 고르기). 광류 작업보다 먼저 선언해 나중에 없앤다.
    RingSlotMaps slot_maps;
    // 광류 작업(보정 창과 앞차 확대율). 링 슬롯을 IVE가 직접 읽으므로 frame_ring보다 먼저 없어져야 한다(뒤에
    // 선언). IVE를 쓸 수 없으면 끄고 계속 간다(저장해 둔 보정 추정은 보정 서비스가 그대로 쓴다). 보정을 워프에
    // 쓸지는 웹 기기 설정이고, 앞차 확대율은 ModelState.lead_flow에 기록하고 앞차 추적에 넣는다.
    std::unique_ptr<FlowWorker> flow;
    LeadFlowInput last_lead;   // 직전 모델 출력의 앞차(다음 카메라 프레임에서 따라간다)
    LeadTracker lead_tracker;  // ModelState.lead_track
    if (config.flow_calibration) {
        try {
            const FlowIntrinsics k{config.input_warp_fx, config.input_warp_fy, config.input_warp_cx,
                                   config.input_warp_cy};
            flow = std::make_unique<FlowWorker>(k, static_cast<int>(frame_ring.width()),
                                                static_cast<int>(frame_ring.height()),
                                                &loop.calibration.stored_flow_blocks());
            // 보정 서비스가 상태를 읽고 블록을 저장하고 초기화를 보낸다. 워커는 loop보다 늦게 선언돼 먼저
            // 없어지지만, 그 뒤에는 update를 부르지 않고 서비스의 소멸자도 측정원을 건드리지 않는다.
            loop.calibration.attach_flow(flow.get());
            std::fprintf(stderr, "modeld: flow calibration and lead flow on (VPP pyramid, IVE LK)\n");
        } catch (const std::exception &e) {
            std::fprintf(stderr, "modeld: flow calibration and lead flow off: %s\n", e.what());
        }
    }
    std::vector<float> raw;
    uint64_t last_frame_seq = 0;
    unsigned processed = 0;
    unsigned errors = 0;
    unsigned missed = 0;
    unsigned frame_sync_failures = 0;
    unsigned window_errors = 0;  // 지난 통계 줄까지의 errors
    uint64_t last_frame_id = 0;
    bool have_last_frame_id = false;
    StatsWindow window(monotonic_now_ns());

    std::fprintf(stderr, "modeld: live frame ring slots=%u frame=%ux%u, every frame\n", frame_ring.slot_count(),
                 frame_ring.width(), frame_ring.height());

    while (!g_stop) {
        RoadAiFrame meta;
        if (!frame_sub.read_new(&last_frame_seq, &meta, sizeof(meta), 1000)) {
            std::fprintf(stderr, "modeld: waiting for roadAiFrame\n");
            continue;
        }

        if (have_last_frame_id && meta.frame_id > last_frame_id + 1)
            missed += static_cast<unsigned>(meta.frame_id - last_frame_id - 1);
        have_last_frame_id = true;
        last_frame_id = meta.frame_id;

        // GDC가 링 슬롯(CMM)을 직접 읽는다(복사 없음). camerad가 슬롯을 다시 잡는 동안은 물리 주소가 0이다.
        const unsigned long long slot_phys = frame_ring.slot_phys(meta.slot);
        uint64_t slot_seq = 0;
        if (slot_phys == 0 || !frame_ring.read_begin(meta.slot, meta.frame_id, &slot_seq)) {
            ++frame_sync_failures;
            ++errors;
            continue;
        }
        loop.settings.poll();
        ego.poll();
        loop.model.set_desire(ego.desire());
        loop.prepare_frame();
        if (flow) {
            // Y 평면이 슬롯 앞에 있다(NV12). IVE가 복사를 마친 뒤 슬롯이 그대로인지 seqlock으로 본다.
            slot_maps.attach(frame_ring, meta.slot);  // 슬롯마다 한 번 매핑한다
            FlowFrame job;
            job.y_phys = slot_phys;
            job.y_virt = slot_maps.virt[meta.slot];
            job.invalidate = [phys = slot_phys, virt = slot_maps.virt[meta.slot]](size_t offset, size_t bytes) {
                cmm_invalidate(phys + offset, virt + offset, bytes);
            };
            job.stride = static_cast<int>(meta.width);
            job.frame_id = meta.frame_id;
            job.t_s = static_cast<double>(meta.timestamp_ns) * 1e-9;
            job.gate = ego.flow_gate();
            job.lead = last_lead;
            job.still_valid = [&frame_ring, slot = meta.slot, id = meta.frame_id, slot_seq] {
                return frame_ring.read_still_valid(slot, id, slot_seq);
            };
            flow->push(std::move(job));
        }

        // 녹화기(recordd)가 모델이 본 바로 그 프레임을 따라가도록 알린다.
        record_frame_pub.publish(&meta, sizeof(meta));

        const uint64_t t0 = monotonic_now_ns();
        // GDC가 읽는 동안 슬롯이 덮어써졌으면 모델 상태(이미지·특징 큐)에 넣기 전에 버린다.
        const bool ok = loop.model.run_frame_phys(
            slot_phys, raw, [&] { return frame_ring.read_still_valid(meta.slot, meta.frame_id, slot_seq); });
        const uint64_t t1 = monotonic_now_ns();
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw.data());
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            LeadIo lead;
            lead.flow = flow ? flow->lead_state() : LeadFlowState{};
            lead.next = &last_lead;
            lead.tracker = &lead_tracker;
            if (ego.vehicle_fresh()) lead.ego_mps = ego.v_ego();
            loop.publish(model_pub, parsed, meta.frame_id, meta.timestamp_ns, model_ms, ego.v_ego(), ego.engaged(),
                         lead);
            ++processed;
            window.count();
        } else {
            ++errors;
        }

        if (window.close_if_due(monotonic_now_ns())) {
            std::fprintf(stderr, "modeld: fps=%.2f frames=%u missed=%u sync=%u errors=%u(+%u) last_ms=%.2f\n",
                         window.rate(), processed, missed, frame_sync_failures, errors, errors - window_errors,
                         ok ? (t1 - t0) / 1000000.0 : 0.0);
            std::fflush(stderr);
            window_errors = errors;
        }
    }

    std::fprintf(stderr, "modeld done frames=%u missed=%u sync=%u errors=%u fps=%.2f\n", processed, missed,
                 frame_sync_failures, errors, window.total_rate(processed, monotonic_now_ns()));
    return processed > 0 && errors == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char *argv[]) {
    install_stop_signal_handlers(&g_stop);
    log_build_version("modeld");

    try {
        AppConfig config = AppConfig::from_env(argc, argv);
        LatestChannel model_pub;
        LatestChannel record_frame_pub;
        if (!model_pub.open(kModelStateTopic, sizeof(ModelState), true))
            throw std::runtime_error("open modelState ipc failed");
        if (!record_frame_pub.open(kRecordFrameTopic, sizeof(RoadAiFrame), true))
            throw std::runtime_error("open recordFrame ipc failed");

        return run_live(config, model_pub, record_frame_pub);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "modeld error: %s\n", e.what());
        std::fprintf(stderr, "%s\n", AppConfig::usage(argc > 0 ? argv[0] : "modeld").c_str());
        return 1;
    }
}
