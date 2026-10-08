/* HUD 스냅샷·타이밍 도구. 렌더러만 떼어 MaixCAM2 화면과 같은 640x480 BGRA 버퍼에 그리고
 * 시나리오별 프레임을 EDGEARGB 파일로 저장한다. model.bin / control.bin은 녹화 이벤트의
 * ModelState / ControlState 원본 바이트다(tools/ui/hud_tools.py inputs가 만든다). 주지 않으면 합성
 * 장면을 쓰고, 준 파일을 읽지 못하거나 프레임을 쓰지 못하면 1로 끝난다. 호스트와 보드에서 같은
 * 소스로 빌드한다.
 * --portrait는 overlayd처럼 세로 패널 방향 480x640 버퍼에 transpose로 그리고(--flip-x/--flip-y는
 * 보드의 disp_flip/disp_mirror 축 뒤집기), 파일도 그 버퍼 그대로 쓴다. 리팩토링 전후 그림이
 * 바이트 단위로 같은지 비교할 때 쓴다.
 * 사용: hud_snapshot [--model model.bin] [--control control.bin] [--iterations N] [--out PREFIX]
 *                    [--portrait [--flip-x] [--flip-y]] */
#include "hud/hud_state.h"
#include "common/ipc_messages.h"
#include "hud/hud_renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

uint64_t now_ns()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool read_file(const std::string &path, void *dst, size_t size)
{
    FILE *file = std::fopen(path.c_str(), "rb");
    if (!file) return false;
    const size_t got = std::fread(dst, 1, size, file);
    std::fclose(file);
    return got == size;
}

/* EDGEARGB: magic 8 B, u32 width, u32 height, BGRA 픽셀(행 우선). 크기는 버퍼(세로면 480x640)다. */
bool write_frame_file(const std::string &path, const HudTarget &target)
{
    FILE *file = std::fopen(path.c_str(), "wb");
    if (!file) return false;
    const bool transpose = target.orientation.transpose;
    const uint32_t buffer_w = transpose ? target.height : target.width;
    const uint32_t buffer_h = transpose ? target.width : target.height;
    const char magic[8] = {'E', 'D', 'G', 'E', 'A', 'R', 'G', 'B'};
    const uint32_t dims[2] = {buffer_w, buffer_h};
    bool ok = std::fwrite(magic, 1, 8, file) == 8 && std::fwrite(dims, 4, 2, file) == 2;
    for (uint32_t row = 0; ok && row < buffer_h; ++row) {
        const uint8_t *src = static_cast<const uint8_t *>(target.map) +
                             static_cast<size_t>(row) * target.stride;
        ok = std::fwrite(src, 4, buffer_w, file) == buffer_w;
    }
    std::fclose(file);
    return ok;
}

/* 직선 도로 합성 장면. 차선은 plan과 같은 지면에 놓이도록 z를 kModelHeight로 둔다. */
ParsedModelOutput synthetic_output()
{
    ParsedModelOutput output;
    output.valid = true;
    output.plan.valid = true;
    output.plan.probability = 0.9f;
    const float lane_y[4] = {-5.4f, -1.8f, 1.8f, 5.4f};
    const float lane_prob[4] = {0.35f, 0.95f, 0.95f, 0.35f};
    const float edge_y[2] = {-7.2f, 7.2f};
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float x = model_x_idx(i);
        const float curve = 0.0006f * x * x;
        output.plan.points[i] = {x, curve, 0.0f};
        for (int lane = 0; lane < 4; ++lane) {
            output.lanes[lane].valid = true;
            output.lanes[lane].probability = lane_prob[lane];
            output.lanes[lane].points[i] = {x, lane_y[lane] + curve, kModelHeight};
        }
        for (int edge = 0; edge < 2; ++edge) {
            output.road_edges[edge].valid = true;
            output.road_edges[edge].std = 0.3f;
            output.road_edges[edge].points[i] = {x, edge_y[edge] + curve, kModelHeight};
        }
    }
    output.leads.valid = true;
    output.leads.global_probabilities[0] = 0.85f;
    output.leads.predictions[0].points[0] = {42.0f, 0.4f, 16.0f, 0.0f};
    return output;
}

struct Scenario {
    const char *name;
    const ParsedModelOutput *scene;  // nullptr = 모델 출력 없음
    HudState hud;
};

void print_stats(const char *label, std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    double sum = 0.0;
    for (double v : values) sum += v;
    std::printf("  %-8s mean=%.3f med=%.3f p90=%.3f max=%.3f ms\n", label,
                sum / values.size(), values[values.size() / 2],
                values[values.size() * 9 / 10], values.back());
}

} // namespace

int main(int argc, char **argv)
{
    std::string model_path;
    std::string control_path;
    std::string out_prefix = "hud_snapshot";
    int iterations = 50;
    HudOrientation orientation;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
        if (arg == "--model") model_path = next();
        else if (arg == "--control") control_path = next();
        else if (arg == "--iterations") iterations = std::max(1, std::atoi(next()));
        else if (arg == "--out") out_prefix = next();
        else if (arg == "--portrait") orientation.transpose = true;
        else if (arg == "--flip-x") orientation.flip_x = true;
        else if (arg == "--flip-y") orientation.flip_y = true;
        else {
            std::fprintf(stderr,
                         "usage: %s [--model model.bin] [--control control.bin] [--iterations N] "
                         "[--out PREFIX] [--portrait [--flip-x] [--flip-y]]\n", argv[0]);
            return 2;
        }
    }

    ModelState model_state{};
    ControlState control_state{};
    const bool have_model = !model_path.empty() && read_file(model_path, &model_state, sizeof(model_state));
    const bool have_control = !control_path.empty() && read_file(control_path, &control_state, sizeof(control_state));
    if (!model_path.empty() && !have_model) {
        std::fprintf(stderr, "cannot read %s\n", model_path.c_str());
        return 1;
    }
    if (!control_path.empty() && !have_control) {
        std::fprintf(stderr, "cannot read %s\n", control_path.c_str());
        return 1;
    }

    ParsedModelOutput output = have_model ? parsed_from_model_state(model_state) : synthetic_output();
    ProjectionState projection = have_model ? projection_from_model_state(model_state)
                                            : make_projection_state(0.0f, 0.0f, 0.0f);

    HudState idle;
    idle.services_healthy = true;
    idle.network_connected = true;
    idle.cpu_percent = 43.0f;
    idle.memory_percent = 10.3f;
    idle.storage_percent = 4.6f;
    idle.cpu_temp_c = 48.0f;
    idle.preview_fps = 38.5f;
    idle.model_fps = 19.0f;
    idle.overlay_fps = 19.0f;
    idle.wifi_signal_dbm = -61;
    std::snprintf(idle.active_block, sizeof(idle.active_block), "control_stale");
    std::snprintf(idle.network_interface, sizeof(idle.network_interface), "wlan0");
    std::snprintf(idle.network_ipv4, sizeof(idle.network_ipv4), "192.168.219.111");
    std::snprintf(idle.network_ssid, sizeof(idle.network_ssid), "edgepilot-car");

    HudState drive = idle;
    drive.panda_connected = true;
    drive.panda_healthy = true;
    drive.vehicle_fresh = true;
    drive.calibration_available = true;
    drive.calibration_status = 1;
    drive.calibration_valid_blocks = 12;
    drive.calibration_roll_deg = 0.12f;
    drive.calibration_pitch_deg = -2.3f;
    drive.calibration_yaw_deg = 0.85f;
    drive.recording = true;
    drive.lane_center_offset_m = lane_center_offset_m(output);
    std::snprintf(drive.wired_interface, sizeof(drive.wired_interface), "usb0");
    std::snprintf(drive.wired_ipv4, sizeof(drive.wired_ipv4), "192.168.123.100");
    // 학습 카드: paramsd는 쓰는 중, torqued·lagd는 아직 학습 중
    drive.learner_fresh = true;
    drive.params_valid = true;
    drive.vehicle_learned = true;
    drive.steer_ratio = 15.43f;
    drive.stiffness = 1.0f;
    drive.angle_offset_deg = -1.61f;
    drive.angle_offset_fast_deg = -1.58f;
    drive.torque_factor = 3.45f;
    drive.torque_factor_filtered = 2.31f;
    drive.torque_friction = 0.12f;
    drive.torque_offset = -0.20f;
    drive.torque_cal_percent = 74;
    drive.lateral_delay_s = 0.42f;
    drive.lag_blocks = 3;
    drive.lag_estimate_s = 0.21f;
    if (have_control) {
        hud_apply_control_state(control_state, true, &drive);
    } else {
        drive.controller_enabled = drive.controller_engaged = drive.controller_active = true;
        drive.cruise_active = true;
        drive.gear = 5;
        drive.cluster_speed_kph = 65.0f;
        drive.ego_speed_kph = 64.0f;
        drive.cruise_command_speed_kph = 70.0f;
        drive.cruise_max_speed_kph = 70.0f;
        drive.steering_angle_deg = -3.0f;
        drive.desired_torque = 4;
        drive.apply_torque = 4;
        drive.steer_torque_fraction = 4.0f / 384.0f;
        drive.driver_torque = 19;
        drive.tpms_valid = true;
        drive.tpms_pressure_fl = drive.tpms_pressure_fr = 36.0f;
        drive.tpms_pressure_rl = drive.tpms_pressure_rr = 35.0f;
    }

    HudState busy = drive;
    busy.left_blinker = true;
    busy.turn_signal_step = 10;
    busy.brake_hold = true;
    busy.green_light_alert_armed = true;
    std::snprintf(busy.engage_reject_label, sizeof(busy.engage_reject_label), "%s",
                  engage_block_label("seatbelt_unlatched"));

    HudState depart = drive;
    depart.departure_alert_type = DepartureAlertType::green_light;
    depart.cluster_speed_kph = 0.0f;
    depart.brake_hold = true;

    HudState fault = drive;
    fault.steering_fault = true;

    HudState torque = drive;  // 오른쪽 조향 45%
    torque.apply_torque = torque.desired_torque = -173;
    torque.steer_torque_fraction = -0.45f;

    HudState saturated = drive;  // 왼쪽 조향 95% + 조향 한계 경고
    saturated.apply_torque = saturated.desired_torque = 365;
    saturated.steer_torque_fraction = 0.95f;
    saturated.steer_saturated = true;

    HudState debug = drive;  // 웹 기기 설정의 HUD 진단을 켠 주행 화면(torqued·lagd는 아직 학습 중)
    debug.debug_card = true;

    HudState learned = drive;  // 학습값을 다 쓰는 중, 후진
    learned.torque_learned = learned.delay_learned = true;
    learned.lag_blocks = 5;
    learned.lateral_delay_s = 0.22f;
    learned.gear = 7;
    learned.cluster_speed_kph = 4.0f;

    HudState lane_change = drive;  // 왼쪽 깜빡이: 핸들을 밀기를 기다림
    lane_change.left_blinker = true;
    lane_change.turn_signal_step = 8;
    lane_change.lane_change = 1;
    lane_change.lane_change_direction = -1;

    HudState paused = drive;  // 85도 위에서 운전자가 넘겨받은 회전
    paused.steer_paused = true;
    paused.steer_paused_by_driver = true;
    paused.cluster_speed_kph = 9.0f;
    paused.driver_torque_fraction = -0.6f;

    HudState hazard = drive;  // 비상등, 세 자리 속도, 비전 크루즈가 설정보다 낮게
    hazard.left_blinker = hazard.right_blinker = true;
    hazard.turn_signal_step = 10;
    hazard.cluster_speed_kph = 105.0f;
    hazard.cruise_max_speed_kph = 110.0f;
    hazard.cruise_command_speed_kph = 90.0f;

    HudState network = drive;  // 상태 알약을 눌러 연 네트워크 카드, 뜨거운 보드
    network.network_card = true;
    network.cpu_temp_c = 74.0f;

    HudState offline = drive;  // 와이파이가 끊기고 USB 링크만 있을 때의 네트워크 카드
    offline.network_card = true;
    offline.network_connected = false;
    offline.wifi_signal_dbm = 0;
    offline.network_ssid[0] = '\0';
    offline.network_ipv4[0] = '\0';

    HudState warnings = drive;  // 재보정, 낮은·높은 타이어, 저장 공간 부족, 오프라인
    warnings.calibration_status = 3;
    warnings.calibration_valid_blocks = 2;
    warnings.tpms_valid = true;
    warnings.tpms_unit = 0;
    warnings.tpms_pressure_fl = 28.0f;
    warnings.tpms_pressure_fr = 36.0f;
    warnings.tpms_pressure_rl = 35.0f;
    warnings.tpms_pressure_rr = 47.0f;
    warnings.recording = false;
    warnings.storage_full = true;
    warnings.network_connected = false;

    HudState standby = drive;
    standby.controller_engaged = standby.controller_active = false;
    standby.cruise_active = false;
    std::snprintf(standby.active_block, sizeof(standby.active_block), "stopped");
    standby.cluster_speed_kph = 12.0f;

    // 아래는 알림·카드 분기를 고루 그리려고 더한 장면이다(리팩토링 전후 그림 비교용).
    HudState soft_disable = drive;  // 재보정으로 3초 뒤 해제 예고
    soft_disable.soft_disabling = true;
    std::snprintf(soft_disable.active_block, sizeof(soft_disable.active_block), "calibration_recalibrating");

    HudState panda_fault = drive;
    panda_fault.panda_faults = 0x4;

    HudState services = drive;  // 서비스가 덜 떴을 때
    services.services_healthy = false;

    HudState laneless = drive;
    laneless.laneless_mode = true;

    HudState radar_lead = drive;  // 비전 앞차 없이 레이더 앞차만
    radar_lead.radar_lead_valid = true;
    radar_lead.radar_lead_distance_m = 24.0f;
    radar_lead.radar_lead_relative_speed_mps = -1.5f;

    HudState lead_departed = depart;
    lead_departed.departure_alert_type = DepartureAlertType::lead_departed;

    HudState changing = drive;  // 오른쪽으로 차선 변경 중
    changing.right_blinker = true;
    changing.turn_signal_step = 6;
    changing.lane_change = 2;
    changing.lane_change_direction = 1;

    HudState tpms_bar = drive;  // bar 단위, 낮은·높은 타이어, 차량 TPMS 경고
    tpms_bar.tpms_valid = true;
    tpms_bar.tpms_unit = 2;
    tpms_bar.tpms_pressure_fl = 2.4f;
    tpms_bar.tpms_pressure_fr = 1.7f;
    tpms_bar.tpms_pressure_rl = 2.5f;
    tpms_bar.tpms_pressure_rr = 3.4f;
    tpms_bar.tpms_warning = true;

    HudState cal_invalid = drive;
    cal_invalid.calibration_status = 2;

    HudState engaged_blocked = drive;  // 결합은 유지한 채 경로가 없어 쉬는 중
    engaged_blocked.controller_active = false;
    std::snprintf(engaged_blocked.active_block, sizeof(engaged_blocked.active_block), "path_invalid");

    HudState debug_stale = debug;  // 진단 카드에서 학습기 상태가 끊김
    debug_stale.learner_fresh = false;
    debug_stale.lag_blocks = -1;

    ParsedModelOutput no_lead = output;
    no_lead.leads = {};

    const std::vector<Scenario> scenarios = {
        {"idle", nullptr, idle},
        {"standby", &output, standby},
        {"drive", &output, drive},
        {"busy", &output, busy},
        {"depart", &output, depart},
        {"fault", &output, fault},
        {"torque", &output, torque},
        {"saturated", &output, saturated},
        {"debug", &output, debug},
        {"learned", &output, learned},
        {"hazard", &output, hazard},
        {"network", &output, network},
        {"offline", &output, offline},
        {"warnings", &output, warnings},
        {"lane_change", &output, lane_change},
        {"paused", &output, paused},
        {"soft_disable", &output, soft_disable},
        {"panda_fault", &output, panda_fault},
        {"services", &output, services},
        {"laneless", &output, laneless},
        {"radar_lead", &no_lead, radar_lead},
        {"lead_departed", &output, lead_departed},
        {"changing", &output, changing},
        {"tpms_bar", &output, tpms_bar},
        {"cal_invalid", &output, cal_invalid},
        {"engaged_blocked", &output, engaged_blocked},
        {"debug_stale", &output, debug_stale},
    };

    constexpr uint32_t width = 640;
    constexpr uint32_t height = 480;
    std::vector<uint32_t> storage(static_cast<size_t>(width) * height, 0);
    const uint32_t stride = (orientation.transpose ? height : width) * 4;
    const HudTarget target{storage.data(), width, height, stride, orientation};

    HudRenderer renderer;
    std::printf("inputs: model=%s control=%s target=%ux%u%s\n",
                have_model ? model_path.c_str() : "synthetic",
                have_control ? control_path.c_str() : "synthetic", width, height,
                orientation.transpose ? " portrait" : "");

    bool written = true;
    for (const Scenario &scenario : scenarios) {
        const ParsedModelOutput &scene = scenario.scene ? *scenario.scene : ParsedModelOutput{};
        std::vector<double> draw_ms;
        for (int i = 0; i < iterations; ++i) {
            const uint64_t t0 = now_ns();
            renderer.draw(target, scene, projection, scenario.hud);
            draw_ms.push_back((now_ns() - t0) / 1e6);
        }
        std::printf("scenario %s (%d iters)\n", scenario.name, iterations);
        print_stats("draw", draw_ms);
        const std::string path = out_prefix + "_" + scenario.name + ".argb";
        if (!write_frame_file(path, target)) {
            std::fprintf(stderr, "cannot write %s\n", path.c_str());
            written = false;
        }
    }
    return written ? 0 : 1;
}
