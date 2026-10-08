#include "common/app_config.h"
#include "model/calibration_service.h"
#include "model/replay_source.h"
#include "common/utils_process.h"
#include "common/utils_time.h"
#include "common/ipc_channels.h"
#include "common/ipc_messages.h"
#include "model/model_state_fill.h"
#include "common/model_output.h"
#include "model/supercombo_model.h"
#include "common/device_settings.h"
#include "maixcam2/maix_cmm.h"

#include <linux/videodev2.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;

/* EDGEPILOT_RAW_DUMP: replay 중 모델 raw 출력을 SCODMP1로 남긴다.
 * tests/gtest_model_output가 이 포맷을 읽어 보드 출력과 호스트
 * 기준을 프레임 단위로 비교할 수 있다(모델 교체 검증용). */
class RawOutputDump
{
public:
    explicit RawOutputDump(const char *path)
    {
        if (!path || !path[0]) return;
        file_ = std::fopen(path, "wb");
        if (!file_) {
            std::fprintf(stderr, "modeld: cannot open raw dump %s\n", path);
            return;
        }
        std::fwrite("SCODMP1\0", 1, 8, file_);
        const uint32_t placeholder = 0;
        std::fwrite(&placeholder, sizeof(placeholder), 1, file_);  // raw floats
        std::fwrite(&placeholder, sizeof(placeholder), 1, file_);  // frames
    }

    ~RawOutputDump()
    {
        if (!file_) return;
        std::fseek(file_, 8, SEEK_SET);
        std::fwrite(&raw_size_, sizeof(raw_size_), 1, file_);
        std::fwrite(&frames_, sizeof(frames_), 1, file_);
        std::fclose(file_);
        std::fprintf(stderr, "modeld: raw dump wrote %u frames x %u floats\n",
                     frames_, raw_size_);
    }

    void append(const std::vector<float> &raw)
    {
        if (!file_ || raw.empty()) return;
        if (raw_size_ == 0) raw_size_ = static_cast<uint32_t>(raw.size());
        if (raw.size() != raw_size_) return;
        std::fwrite(raw.data(), sizeof(float), raw.size(), file_);
        ++frames_;
    }

private:
    std::FILE *file_ = nullptr;
    uint32_t raw_size_ = 0;
    uint32_t frames_ = 0;
};

/* 1초 창 통계. 두 루프(라이브·리플레이)가 같은 시계로 fps를 센다. 벽시계가 아니라 monotonic_now_ns라
 * NTP가 시각을 옮겨도 창 길이와 전체 fps가 흔들리지 않는다. */
struct RateWindow
{
    uint64_t start_ns = monotonic_now_ns();
    uint64_t last_ns = start_ns;
    unsigned last_processed = 0;
    unsigned last_errors = 0;

    // 1초가 지났으면 창을 닫고 true. window_us는 닫힌 창의 길이.
    bool close_if_due(uint64_t *window_us)
    {
        const uint64_t now_ns = monotonic_now_ns();
        const uint64_t elapsed_us = (now_ns - last_ns) / 1000ULL;
        if (elapsed_us < 1000000ULL) return false;
        *window_us = elapsed_us;
        last_ns = now_ns;
        return true;
    }

    double total_fps(unsigned processed) const
    {
        const uint64_t since_start_us = (monotonic_now_ns() - start_ns) / 1000ULL;
        return since_start_us > 0 ? processed * 1000000.0 / since_start_us : 0.0;
    }
};

/* controlsd 스냅샷에서 모델 입력용 자차 속도와 desire를 읽는다. controlsd가
 * 아직 없으면 열릴 때까지 마지막 값을 유지한다. */
class EgoStateReader
{
public:
    void poll()
    {
        if (!open_) open_ = sub_.open(kControlStateTopic, sizeof(ControlState), false);
        if (!open_) return;
        ControlState control_state;
        if (!sub_.read(&control_state, sizeof(control_state))) return;
        const float ego_speed_kph = control_state.ego_speed_kph > 0.0f
            ? control_state.ego_speed_kph
            : control_state.cluster_speed_kph;
        v_ego_ = std::max(0.0f, ego_speed_kph / 3.6f);
        desire_ = static_cast<int>(control_state.desire);
    }
    float v_ego() const { return v_ego_; }
    int desire() const { return desire_; }

private:
    LatestChannel sub_;
    bool open_ = false;
    float v_ego_ = 0.0f;
    int desire_ = 0;
};

/* 카메라 장착(웹 기기 설정 camera_offset_m·camera_height_m). 라이브는 display.json을 1초마다 보고
 * 바뀐 오프셋은 초당 kRateMps로 옮긴다: 모델은 t−4·t 두 프레임과 4.8초 특징 이력을 보므로 한 번에
 * 바꾸면 차가 옆으로 순간 이동한 것처럼 보여 pose·보정·locationd·경로가 튄다(보정 워프도 같은
 * 이유로 kSmoothCycles에 걸쳐 옮긴다). 시작할 때는 첫 프레임부터 그대로 쓴다. 재생은 보드 설정을
 * 읽지 않고 EDGEPILOT_CAMERA_OFFSET_M·EDGEPILOT_CAMERA_HEIGHT_M(기본 0, 모델 높이)만 쓴다. */
class CameraMount
{
public:
    static constexpr float kRateMps = 0.1f;
    static constexpr float kFrameS = 0.05f;

    explicit CameraMount(bool live) : live_(live)
    {
        if (live_) {
            file_.poll(monotonic_now_ns(), &target_);
        } else {
            target_.camera_offset_m = env_setting("EDGEPILOT_CAMERA_OFFSET_M", &DeviceSettings::camera_offset_m);
            target_.camera_height_m = env_setting("EDGEPILOT_CAMERA_HEIGHT_M", &DeviceSettings::camera_height_m);
        }
        offset_m_ = target_.camera_offset_m;
        log();
    }

    // 매 프레임 실행 직전. 이번 프레임 워프에 쓴 값이 offset_m()·height_m()이다.
    void apply(SupercomboModel &model)
    {
        if (live_ && file_.poll(monotonic_now_ns(), &target_)) log();
        const float step = kRateMps * kFrameS;
        offset_m_ += std::clamp(target_.camera_offset_m - offset_m_, -step, step);
        model.set_camera_mount(offset_m_, target_.camera_height_m);
    }
    float offset_m() const { return offset_m_; }
    float height_m() const { return target_.camera_height_m; }

private:
    static float env_setting(const char *name, float DeviceSettings::*member)
    {
        const DeviceSettings defaults;
        const char *text = std::getenv(name);
        if (!text || !*text) return defaults.*member;
        char *end = nullptr;
        const float value = std::strtof(text, &end);
        if (*end != '\0' || !std::isfinite(value)) throw std::runtime_error(std::string(name) + " is not a number");
        return clamp_device_setting(member, value);
    }
    void log() const
    {
        std::fprintf(stderr, "\nmodeld: camera mount offset=%+.3f m height=%.2f m (%s)\n", target_.camera_offset_m,
                     target_.camera_height_m, live_ ? "display.json" : "replay env");
    }

    bool live_ = false;
    DeviceSettingsFile file_;
    DeviceSettings target_;
    float offset_m_ = 0.0f;
};

bool publish_output(LatestChannel &model_pub, SupercomboModel &model, const ParsedModelOutput &parsed,
                    CalibrationService &calibration, const CameraMount &mount,
                    uint64_t frame_id, uint64_t capture_timestamp_ns, float model_ms,
                    float v_ego)
{
    calibration.update(parsed, v_ego);
    float input_rpy[3];
    calibration.input_rpy(input_rpy);
    model.set_input_calibration(input_rpy);

    const ProjectionState projection = calibration.projection();

    ModelState state;
    fill_model_state(state, parsed, projection, calibration.snapshot(),
                          frame_id, capture_timestamp_ns, model_ms);
    state.camera_offset_m = mount.offset_m();
    state.camera_height_m = mount.height_m();
    return model_pub.publish(&state, sizeof(state));
}

int run_replay(const AppConfig &config, LatestChannel &model_pub)
{
    ReplayNv12Source source(config.replay_nv12_path);
    /* 재생 소스 해상도에 맞춘 기본 워프. GDC 워프도 이 크기로 만들어져야
     * 라이브와 같은 경로를 탄다. */
    AppConfig replay_config = config;
    replay_config.nv12_width = source.width();
    replay_config.nv12_height = source.height();
    replay_config.set_warp_source(source.width(), source.height());
    const unsigned target_frames = config.max_frames > 0
        ? std::min(config.max_frames, source.frame_count())
        : source.frame_count();
    std::fprintf(stderr, "modeld replay input format=NV12 frames=%u file=%s target=%u\n",
                 source.frame_count(), config.replay_nv12_path.c_str(), target_frames);

    SupercomboModel model(config.axmodel_path.c_str(), replay_config);
    CalibrationService calibration(config);
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);
    CameraMount mount(false);

    Nv12Frame frame;
    std::vector<float> raw;
    RawOutputDump raw_dump(std::getenv("EDGEPILOT_RAW_DUMP"));
    unsigned processed = 0;
    unsigned errors = 0;
    RateWindow window;

    while (!g_stop && source.read(frame)) {
        mount.apply(model);
        const uint64_t t0 = monotonic_now_ns();
        const bool ok = model.run_frame_nv12(frame.data.data(), frame.width, frame.height, raw);
        const uint64_t t1 = monotonic_now_ns();
        if (ok) {
            raw_dump.append(raw);
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration, mount,
                                processed, monotonic_now_ns(), model_ms, 0.0f)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            ++processed;
        } else {
            ++errors;
        }

        uint64_t window_us = 0;
        if (window.close_if_due(&window_us)) {
            std::fprintf(stderr, "modeld replay: frames=%u/%u fps=%.2f errors=%u          \r",
                         processed, target_frames, window.total_fps(processed), errors);
            std::fflush(stderr);
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;
    }

    std::fprintf(stderr, "\nmodeld replay done frames=%u errors=%u fps=%.2f\n",
                 processed, errors, window.total_fps(processed));
    return processed > 0 && errors == 0 ? 0 : 1;
}

/* CPU 워프 대체 경로용: CMM 링 슬롯을 캐시 없이 매핑해 링에 붙인다. camerad가
 * 다시 떠서 물리 주소가 바뀌면 다시 매핑한다. */
struct RingSlotMaps {
    unsigned long long phys[kFrameSlots] = {};
    uint8_t *virt[kFrameSlots] = {};
    size_t size = 0;

    void attach(FrameRing &ring, unsigned slot)
    {
        const unsigned long long p = ring.slot_phys(slot);
        if (p == phys[slot] && virt[slot]) return;
        if (virt[slot]) cmm_unmap(virt[slot], size);
        size = ring.frame_bytes();
        virt[slot] = cmm_map(p, size);
        phys[slot] = p;
        ring.attach_slot(slot, virt[slot]);
    }
    ~RingSlotMaps()
    {
        for (unsigned i = 0; i < kFrameSlots; ++i) cmm_unmap(virt[i], size);
    }
};

int run_live(const AppConfig &config, LatestChannel &model_pub,
             LatestChannel &record_frame_pub)
{
    LatestChannel frame_sub;
    FrameRing frame_ring;
    if (!frame_sub.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true))
        throw std::runtime_error("open roadAiFrame ipc failed");

    while (!g_stop && !frame_ring.open(false)) {
        std::fprintf(stderr, "modeld: waiting for road ai frame ring\n");
        usleep(500000);
    }
    if (!frame_ring.valid()) return 1;

    SupercomboModel model(config.axmodel_path.c_str(), config);
    CalibrationService calibration(config);
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);
    CameraMount mount(true);
    EgoStateReader ego;
    std::vector<float> raw;
    uint64_t last_frame_seq = 0;
    unsigned processed = 0;
    unsigned errors = 0;
    unsigned missed = 0;
    unsigned frame_sync_failures = 0;
    uint64_t last_frame_id = 0;
    bool have_last_frame_id = false;
    RateWindow window;
    std::vector<uint8_t> frame_copy(frame_ring.frame_bytes());
    RingSlotMaps slot_maps;

    std::fprintf(stderr, "modeld: live frame ring slots=%u frame=%ux%u, every frame\n",
                 frame_ring.slot_count(), frame_ring.width(), frame_ring.height());

    while (!g_stop) {
        RoadAiFrame meta;
        if (!frame_sub.read_new(&last_frame_seq, &meta, sizeof(meta), 1000)) {
            std::fprintf(stderr, "modeld: waiting for roadAiFrame\n");
            continue;
        }
        if (meta.slot >= frame_ring.slot_count()) {
            ++errors;
            continue;
        }

        if (have_last_frame_id && meta.frame_id > last_frame_id + 1)
            missed += static_cast<unsigned>(meta.frame_id - last_frame_id - 1);
        have_last_frame_id = true;
        last_frame_id = meta.frame_id;

        model.set_chroma_vu(meta.format == V4L2_PIX_FMT_NV21);
        const int frame_w = static_cast<int>(meta.width), frame_h = static_cast<int>(meta.height);
        /* 링 슬롯이 CMM이고 GDC 워프를 쓰면 GDC가 슬롯을 직접 읽는다(복사 없음). 아니면
         * 슬롯을 복사해 CPU 워프로 간다(CMM 슬롯은 캐시 없는 매핑으로 읽는다). */
        const unsigned long long slot_phys = frame_ring.slot_phys(meta.slot);
        const bool zero_copy = slot_phys != 0 && model.input_buffer(frame_w, frame_h) != nullptr;
        uint64_t slot_seq = 0;
        bool frame_ready = false;
        if (zero_copy) {
            frame_ready = frame_ring.read_begin(meta.slot, meta.frame_id, &slot_seq);
        } else {
            if (slot_phys != 0) slot_maps.attach(frame_ring, meta.slot);
            frame_ready = frame_ring.copy_slot(meta.slot, meta.frame_id, frame_copy.data(), frame_copy.size());
        }
        if (!frame_ready) {
            ++frame_sync_failures;
            ++errors;
            continue;
        }
        ego.poll();
        model.set_desire(ego.desire());
        mount.apply(model);

        // 녹화기(recordd)가 모델이 본 바로 그 프레임을 따라가도록 알린다.
        if (!record_frame_pub.publish(&meta, sizeof(meta))) {
            std::fprintf(stderr, "\nmodeld: publish recordFrame failed\n");
        }

        const uint64_t t0 = monotonic_now_ns();
        // GDC가 읽는 동안 슬롯이 덮어써졌으면 모델 상태(이미지·특징 큐)에 넣기 전에 버린다.
        const bool ok = zero_copy
            ? model.run_frame_phys(slot_phys, frame_w, frame_h, raw, [&] {
                  return frame_ring.read_still_valid(meta.slot, meta.frame_id, slot_seq);
              })
            : model.run_frame_nv12(frame_copy.data(), meta.width, meta.height, raw);
        const uint64_t t1 = monotonic_now_ns();
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration, mount,
                                meta.frame_id, meta.timestamp_ns, model_ms, ego.v_ego())) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            ++processed;
        } else {
            ++errors;
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;

        uint64_t window_us = 0;
        if (window.close_if_due(&window_us)) {
            std::fprintf(stderr,
                         "modeld: fps=%.2f frames=%u missed=%u sync=%u errors=%u(+%u) last_ms=%.2f          \r",
                         (processed - window.last_processed) * 1000000.0 / window_us,
                         processed,
                         missed,
                         frame_sync_failures,
                         errors,
                         errors - window.last_errors,
                         ok ? (t1 - t0) / 1000000.0 : 0.0);
            std::fflush(stderr);
            window.last_processed = processed;
            window.last_errors = errors;
        }
    }

    std::fprintf(stderr, "\nmodeld done frames=%u missed=%u sync=%u errors=%u fps=%.2f\n",
                 processed, missed, frame_sync_failures, errors, window.total_fps(processed));
    return processed > 0 && errors == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char *argv[])
{
    install_stop_signal_handlers(&g_stop);

    try {
        AppConfig config = AppConfig::from_env(argc, argv);
        LatestChannel model_pub;
        LatestChannel record_frame_pub;
        if (!model_pub.open(kModelStateTopic, sizeof(ModelState), true))
            throw std::runtime_error("open modelState ipc failed");
        if (!record_frame_pub.open(kRecordFrameTopic, sizeof(RoadAiFrame), true))
            throw std::runtime_error("open recordFrame ipc failed");

        if (config.replay_enabled()) return run_replay(config, model_pub);
        return run_live(config, model_pub, record_frame_pub);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "modeld error: %s\n", e.what());
        std::fprintf(stderr, "%s\n", AppConfig::usage(argc > 0 ? argv[0] : "modeld").c_str());
        return 1;
    }
}
