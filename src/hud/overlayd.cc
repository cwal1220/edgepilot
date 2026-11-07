// MaixCAM2 HUD. LCD의 VO 하드웨어 레이어 둘을 쓴다: 카메라 프레임 링의 최신 프레임은
// 비디오 레이어 0(IVPS로 가운데 4:3을 640x480에), HUD는 렌더러가 640x480 화면 좌표로
// 세로 패널 방향 그림판에 그려 그래픽 레이어 fb0로 옮기고, 합성은 하드웨어가 한다. 알림은
// 화면 카드와 로그에 더해 보드 스피커로 알림음을 낸다(alert_sound.h). 터치스크린은 오른쪽
// 위 상태 알약(네트워크 카드)과 왼쪽 열(진단 카드)을 켜고 끄는 데 쓴다.

#include "common/app_config.h"
#include "common/build_version.h"
#include "common/device_settings.h"
#include "common/ipc_channels.h"
#include "common/ipc_messages.h"
#include "common/projection.h"
#include "common/utils_process.h"
#include "common/utils_time.h"
#include "hud/alert_sound.h"
#include "hud/hud_policy.h"
#include "hud/hud_renderer.h"
#include "hud/hud_state.h"
#include "hud/system_monitor.h"
#include "maixcam2/maix_display.h"
#include "maixcam2/maix_touch.h"

#include <signal.h>

#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace {

volatile sig_atomic_t g_stop = 0;
// 책상 확인용: SIGUSR1을 받을 때마다 알림음을 차례로 하나씩 낸다(pkill -USR1 overlayd).
volatile sig_atomic_t g_test_sound_requests = 0;

void on_test_sound_signal(int) { ++g_test_sound_requests; }

constexpr int kOutW = MaixDisplay::kWidth;
constexpr int kOutH = MaixDisplay::kHeight;
constexpr uint64_t kStateFreshNs = 2'000'000'000ULL;
// 오버레이(HUD 그림, CPU)는 새 모델 출력이 오면 곧바로(20 Hz) 다시 그린다. 모델이 멈춰도 속도·알림이
// 따라가도록 지난번에 그린 지 100 ms가 지나면 그린다. 영상은 하드웨어 레이어라 카메라 프레임마다 올린다.
constexpr uint64_t kMaxDrawIntervalNs = 100'000'000ULL;
// 루프는 새 모델 출력을 이만큼까지 기다리고 돈다(카메라 프레임, 터치, 다른 상태).
constexpr int kPollMs = 5;

struct StageStats {
    uint64_t total_ns = 0;
    uint32_t count = 0;

    void add(uint64_t elapsed_ns) {
        total_ns += elapsed_ns;
        ++count;
    }

    double avg_ms_and_reset() {
        const double avg = count > 0 ? static_cast<double>(total_ns) / static_cast<double>(count) / 1000000.0 : 0.0;
        total_ns = 0;
        count = 0;
        return avg;
    }
};

class HudDisplay {
public:
    explicit HudDisplay(const AppConfig &config) : profile_(config.profile) {
        // 차선 투영도 모델 워프와 같은 카메라 파라미터를 쓴다(EDGEPILOT_CAMERA_INTRINSICS).
        projection_set_camera_intrinsics(config.camera_fx, config.camera_fy, config.camera_cx, config.camera_cy);
    }

    int run() {
        if (!model_state_.open(kModelStateTopic)) throw std::runtime_error("open modelState ipc failed");
        if (!panda_state_.open(kPandaStateTopic)) throw std::runtime_error("open pandaState ipc failed");
        if (!control_state_.open(kControlStateTopic)) throw std::runtime_error("open controlState ipc failed");
        if (!manager_state_.open(kManagerStateTopic)) throw std::runtime_error("open managerState ipc failed");
        if (!frame_sub_.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true))
            throw std::runtime_error("open roadAiFrame ipc failed");
        if (!record_state_.open(kRecordStateTopic)) throw std::runtime_error("open recordState ipc failed");
        if (!learner_state_.open(kLearnerStateTopic)) throw std::runtime_error("open learnerState ipc failed");
        if (!localization_state_.open(kLocalizationStateTopic))
            throw std::runtime_error("open localization ipc failed");
        if (!touch_.open()) std::fprintf(stderr, "overlayd: no touchscreen; tap toggles disabled\n");

        StatsWindow window(monotonic_now_ns());  // fps·CPU를 재는 1초 창(상태 줄 한 줄)
        while (!g_stop) {
            const bool new_model = update_model();
            poll_states();
            // 시계는 루프에 한 번, 스냅샷을 다 받은 뒤에 읽는다(방금 받은 상태가 미래 시각으로 보이지 않게).
            const uint64_t now = monotonic_now_ns();
            refresh_hud_state(now, new_model);
            turn_signal_clock_.update(now, &hud_);
            update_touch(now);
            play_test_sound();
            apply_device_settings(now);
            update_preview();
            if (new_model || now - last_overlay_draw_ns_ >= kMaxDrawIntervalNs) {
                last_overlay_draw_ns_ = now;
                draw_overlay();
            }

            if (window.close_if_due(now)) {
                const double seconds = window.seconds();
                hud_.preview_fps = static_cast<float>(preview_frames_ / seconds);
                hud_.overlay_fps = static_cast<float>(overlay_frames_ / seconds);
                hud_.model_fps = static_cast<float>(model_updates_ / seconds);
                system_monitor_.sample(&hud_);
                char profile_text[96] = "";
                if (profile_)
                    std::snprintf(profile_text, sizeof(profile_text), " draw=%.2fms video=%.2fms lag=%.1fms",
                                  overlay_stats_.avg_ms_and_reset(), present_stats_.avg_ms_and_reset(),
                                  lag_stats_.avg_ms_and_reset());
                std::fprintf(stderr,
                             "overlay: preview=%.2f model=%.2f overlay=%.2f%s cpu=%.1f%% mem=%.1f%% temp=%.1fC "
                             "errors=%u\n",
                             hud_.preview_fps, hud_.model_fps, hud_.overlay_fps, profile_text, hud_.cpu_percent,
                             hud_.memory_percent, hud_.cpu_temp_c, errors_);
                std::fflush(stderr);
                preview_frames_ = overlay_frames_ = model_updates_ = 0;
            }
        }
        return 0;
    }

private:
    // 최신 카메라 프레임을 화면 레이어 0에 올린다. 영상은 가운데 4:3을 IVPS로 잘라
    // 비율을 지키고, 투영(projection.cc)도 같은 영역을 640x480 화면에 담는다.
    void update_preview() {
        RoadAiFrame meta;
        if (!frame_sub_.read_new(&last_frame_seq_, &meta, sizeof(meta), 0)) return;
        if (!frame_ring_.valid() && !frame_ring_.open(false)) return;
        const uint64_t start = profile_ ? monotonic_now_ns() : 0;
        const unsigned long long phys = frame_ring_.slot_phys(meta.slot);
        // IVPS가 CMM 슬롯을 직접 읽는다(복사 없음). 읽는 동안 camerad가 슬롯을
        // 덮어썼으면 화면에 올리기 전에 버린다.
        uint64_t seq = 0;
        const bool shown =
            phys != 0 && frame_ring_.read_begin(meta.slot, meta.frame_id, &seq) &&
            display_.show_video_phys(phys, static_cast<int>(meta.width), static_cast<int>(meta.height),
                                     [&] { return frame_ring_.read_still_valid(meta.slot, meta.frame_id, seq); });
        if (!shown) {
            ++errors_;
            return;
        }
        if (profile_) present_stats_.add(monotonic_now_ns() - start);
        ++preview_frames_;
    }

    // HUD를 세로 패널 방향 그림판에 화면 좌표(640x480)로 그리고(캔버스가 transpose), 바뀐 칸만
    // 그래픽 레이어 fb0로 옮긴다. 회전 하드웨어(TDP)를 거치지 않아 그린 화소가 그대로 나간다.
    void draw_overlay() {
        const uint64_t draw_start = profile_ ? monotonic_now_ns() : 0;
        const MaixDisplay::OverlayBuffer buffer = display_.begin_overlay();
        const HudTarget target{buffer.pixels, static_cast<uint32_t>(kOutW), static_cast<uint32_t>(kOutH),
                               static_cast<uint32_t>(buffer.stride), true};
        // 모델 상태가 없으면 출력이 무효라 투영을 쓰는 장면은 그리지 않는다
        renderer_.draw(target, have_model_state_ ? latest_output_ : ParsedModelOutput{}, latest_projection_, hud_);
        display_.end_overlay(renderer_.last_damage().data(), renderer_.last_tile_shift());
        if (profile_) {
            overlay_stats_.add(monotonic_now_ns() - draw_start);
            // 모델 출력이 나온 때부터 그 출력을 그리기 시작할 때까지
            if (have_model_state_) lag_stats_.add(draw_start - model_state_.latest().model_timestamp_ns);
        }
        ++overlay_frames_;
    }

    // 웹 기기 설정: 백라이트는 overlayd만 쓴다(웹 콘솔은 파일만 쓴다). camerad가 다시 뜰 때마다 overlayd도 다시
    // 뜨므로 시작할 때 켠 /boot/configs 밝기를 곧바로 이 값으로 바꾼다(예전에는 밤에 꺼 둔 백라이트가 다시
    // 켜졌다). 알림음 크기는 바뀌면 적용하고 확인음을 한 번 낸다(시작 때 읽은 값은 소리 없이). HUD 진단 카드는
    // 설정이 아니라 화면 왼쪽 열을 눌러 켜고 끈다(overlayd가 다시 시작하면 꺼진다). 카메라 장착 오프셋은 modeld가
    // 그 프레임에 쓴 값을 ModelState로 받는다(projection_from_model_state).
    void apply_device_settings(uint64_t now_ns) {
        if (!device_settings_file_.poll(now_ns, &device_settings_)) return;
        const float brightness = device_settings_.brightness_percent;
        if (std::isfinite(brightness)) {
            const bool on = device_settings_.display_enabled;
            if (display_.set_backlight(on, brightness))
                std::fprintf(stderr, "overlayd: backlight %s %.0f%%\n", on ? "on" : "off", brightness);
            else
                std::fprintf(stderr, "overlayd: cannot set the backlight\n");
        }
        const float percent = device_settings_.alert_volume_percent;
        if (sound_.enabled() && std::isfinite(percent) && std::fabs(percent - sound_.volume_percent()) > 0.5f) {
            sound_.set_volume_percent(percent);
            std::fprintf(stderr, "overlayd: alert volume %.0f%%\n", sound_.volume_percent());
            if (device_settings_read_) sound_.play(AlertSoundId::Engage);  // 바꾼 크기를 들려준다
        }
        device_settings_read_ = true;
    }

    // 새 모델 출력을 kPollMs까지 기다려 받는다. 받았으면 true.
    bool update_model() {
        if (!model_state_.poll(kPollMs)) return false;
        const ModelState &model = model_state_.latest();
        ++model_updates_;
        latest_output_ = parsed_from_model_state(model);
        latest_projection_ = projection_from_model_state(model);
        return true;
    }

    // 새 panda/control/manager/record/학습기/locationd 스냅샷을 받는다.
    void poll_states() {
        panda_state_.poll();
        control_state_.poll();
        manager_state_.poll();
        record_state_.poll();
        learner_state_.poll();
        localization_state_.poll();
    }

    // 터치로 네트워크·진단 카드를 여닫고(HudTouch) 탭은 로그에 남긴다.
    void update_touch(uint64_t now_ns) {
        int x = 0, y = 0;
        if (touch_.poll_tap(&x, &y)) {
            const char *action = hud_touch_.tap(x, y, kOutW, kOutH, now_ns, &hud_);
            std::fprintf(stderr, "overlayd: tap x=%d y=%d %s\n", x, y, action);
        }
        hud_touch_.expire(now_ns, &hud_);
    }

    // 최신 스냅샷을 HUD 상태로 옮기고, 이 프레임의 알림음을 낸다(HudAlertPolicy). 차선 안 위치는 새 모델
    // 출력마다 한 번 거른다.
    void refresh_hud_state(uint64_t now, bool new_model) {
        auto fresh = [now](uint64_t timestamp_ns) { return timestamp_fresh_ns(timestamp_ns, now, kStateFreshNs); };
        const ModelState &model = model_state_.latest();
        const bool model_fresh = fresh(model.model_timestamp_ns);
        have_model_state_ = model.valid != 0 && model_fresh;
        if (new_model) {
            const float lane_center =
                have_model_state_ ? lane_center_offset_m(latest_output_) : std::numeric_limits<float>::quiet_NaN();
            hud_.lane_center_offset_m = smooth_lane_center_offset(hud_.lane_center_offset_m, lane_center);
        }
        const bool panda_fresh = fresh(panda_state_.latest().timestamp_ns);
        const bool control_fresh = fresh(control_state_.latest().timestamp_ns);
        hud_apply_panda_state(panda_state_.latest(), panda_fresh, &hud_);
        hud_apply_control_state(control_state_.latest(), control_fresh, &hud_);
        hud_apply_model_state(model, model_fresh, &hud_);
        hud_apply_manager_state(manager_state_.latest(), fresh(manager_state_.latest().timestamp_ns), have_model_state_,
                                &hud_);
        hud_apply_record_state(record_state_.latest(), fresh(record_state_.latest().timestamp_ns), &hud_);
        hud_apply_learner_state(learner_state_.latest(), fresh(learner_state_.latest().timestamp_ns), &hud_);
        hud_apply_localization_state(localization_state_.latest(), fresh(localization_state_.latest().timestamp_ns),
                                     &hud_);
        play_alert(alert_policy_.update(control_state_.latest(), control_fresh, panda_state_.latest(), panda_fresh, now,
                                        &hud_));
    }

    void play_test_sound() {
        static constexpr AlertSoundId kOrder[] = {
            AlertSoundId::Engage,        AlertSoundId::Disengage,   AlertSoundId::Unable,
            AlertSoundId::SignalChanged, AlertSoundId::Unavailable,
        };
        const int requests = g_test_sound_requests;
        if (requests == test_sounds_played_) return;
        test_sounds_played_ = requests;
        const AlertSoundId id = kOrder[static_cast<size_t>(requests - 1) % std::size(kOrder)];
        sound_.play(id);
        std::fprintf(stderr, "overlayd: test sound %s\n", alert_sound_name(id));
    }

    // 정책이 고른 알림음을 내고 로그에 남긴다. 거부는 차단 사유, 해제 예고는 그 플래그와 사유를 함께.
    void play_alert(const HudSoundDecision &decision) {
        if (decision.sound == HudSound::None) return;
        sound_.play(hud_sound_id(decision.sound));
        switch (decision.sound) {
            case HudSound::Unable:
                std::fprintf(stderr, "overlayd: alert=unable event=%u block=%s\n", decision.event_id,
                             control_state_.latest().engage_reject_block);
                break;
            case HudSound::TakeControl:
                std::fprintf(stderr, "overlayd: alert=take_control soft_disable=%d steer_saturated=%d block=%s\n",
                             hud_.soft_disabling ? 1 : 0, hud_.steer_saturated ? 1 : 0, hud_.active_block);
                break;
            case HudSound::Unavailable:
                std::fprintf(stderr, "overlayd: alert=unavailable\n");
                break;
            default:
                std::fprintf(stderr, "overlayd: alert=%s event=%u\n", alert_sound_name(hud_sound_id(decision.sound)),
                             decision.event_id);
                break;
        }
    }

    HudRenderer renderer_;
    AlertSound sound_;
    DeviceSettingsFile device_settings_file_;
    DeviceSettings device_settings_;
    bool device_settings_read_ = false;
    int test_sounds_played_ = 0;
    bool profile_ = false;

    Subscription<ModelState> model_state_;
    Subscription<PandaState> panda_state_;
    Subscription<ControlState> control_state_;
    Subscription<ManagerState> manager_state_;
    Subscription<RecordState> record_state_;
    Subscription<LearnerState> learner_state_;
    Subscription<LocalizationState> localization_state_;
    LatestChannel frame_sub_;
    MaixDisplay display_;
    MaixTouch touch_;
    FrameRing frame_ring_;
    uint64_t last_frame_seq_ = 0;
    uint64_t last_overlay_draw_ns_ = 0;

    ParsedModelOutput latest_output_{};
    ProjectionState latest_projection_{};
    bool have_model_state_ = false;
    unsigned errors_ = 0;

    unsigned preview_frames_ = 0;
    unsigned overlay_frames_ = 0;
    unsigned model_updates_ = 0;

    StageStats overlay_stats_;
    StageStats present_stats_;
    StageStats lag_stats_;
    SystemMonitor system_monitor_;
    HudAlertPolicy alert_policy_;
    TurnSignalClock turn_signal_clock_;
    HudTouch hud_touch_;
    HudState hud_;
};

}  // namespace

int main() {
    install_stop_signal_handlers(&g_stop);
    log_build_version("overlayd");
    signal(SIGUSR1, on_test_sound_signal);

    try {
        AppConfig config = AppConfig::from_env_defaults();
        HudDisplay app(config);
        return app.run();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "overlayd error: %s\n", e.what());
        return 1;
    }
}
