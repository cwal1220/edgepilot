#include "controls/control_params.h"

#include "common/utils_json.h"

#include <cstdio>
#include <utility>

namespace {

constexpr JsonBoolField<SteeringParams> kSteeringBools[] = {
    {"enabled", &SteeringParams::enabled},
    {"avoid_lkas_fault_enabled", &SteeringParams::avoid_lkas_fault_enabled},
    {"live_bank_compensation", &SteeringParams::live_bank_compensation},
    {"use_live_vehicle_params", &SteeringParams::use_live_vehicle_params},
    {"use_live_torque_params", &SteeringParams::use_live_torque_params},
    {"use_live_delay", &SteeringParams::use_live_delay},
    {"nn_feedforward", &SteeringParams::nn_feedforward},
    {"use_locationd_learner_inputs", &SteeringParams::use_locationd_learner_inputs},
    {"laneless_mode", &SteeringParams::laneless_mode},
};
constexpr JsonIntField<SteeringParams> kSteeringInts[] = {
    {"steering_pressed_threshold", 0, 500, &SteeringParams::steering_pressed_threshold},
};
constexpr JsonFloatField<SteeringParams> kSteeringFloats[] = {
    {"torque_lat_accel_factor", 0.5f, 5.0f, &SteeringParams::torque_lat_accel_factor},
    {"torque_kp", 0.0f, 10.0f, &SteeringParams::torque_kp},
    {"torque_ki", 0.0f, 2.0f, &SteeringParams::torque_ki},
    {"torque_friction", 0.0f, 0.3f, &SteeringParams::torque_friction},
    {"steering_angle_deadzone_deg", 0.0f, 3.0f, &SteeringParams::steering_angle_deadzone_deg},
    {"torque_straight_kp_scale", 0.3f, 1.0f, &SteeringParams::torque_straight_kp_scale},
    {"steer_ratio", 8.0f, 25.0f, &SteeringParams::steer_ratio},
    {"tire_stiffness_factor", 0.2f, 2.0f, &SteeringParams::tire_stiffness_factor},
    {"steer_actuator_delay", 0.01f, 1.0f, &SteeringParams::steer_actuator_delay},
    // 고장 각도(avoid_lkas_fault_max_angle_deg, 85도) 아래에서 멈춰야 뜻이 있다
    {"avoid_lkas_fault_hold_angle_deg", 0.0f, 84.0f, &SteeringParams::avoid_lkas_fault_hold_angle_deg},
    {"angle_offset_deg", -10.0f, 10.0f, &SteeringParams::angle_offset_deg},
    {"torque_lat_accel_offset", -1.0f, 1.0f, &SteeringParams::torque_lat_accel_offset},
    {"lane_path_weight", 0.5f, 10.0f, &SteeringParams::lane_path_weight},
    {"min_steer_speed_mps", 0.0f, 5.0f, &SteeringParams::min_steer_speed_mps},
    {"lane_change_min_speed_kph", 0.0f, 80.0f, &SteeringParams::lane_change_min_speed_kph},
};

// 파라미터 파일을 stat하는 간격. 웹 콘솔에서 바꾼 값도 이 확인으로 1초 안에 적용된다. 제어 스레드에서 SD를 자주
// 보지 않게 1초로 둔다.
constexpr uint64_t kParamPollIntervalNs = 1'000'000'000ULL;

}  // namespace

bool load_steering_params(const std::string &path, SteeringParams *params, std::string *error) {
    return load_json_param_file(
        path,
        [params](const JsonValue &root) {
            parse_json_fields(root, kSteeringBools, params);
            parse_json_fields(root, kSteeringInts, params);
            parse_json_fields(root, kSteeringFloats, params);
        },
        error);
}

std::string first_steering_param_difference(const SteeringParams &a, const SteeringParams &b) {
    for (const auto &field : kSteeringBools)
        if (a.*field.member != b.*field.member) return field.key;
    for (const auto &field : kSteeringInts)
        if (a.*field.member != b.*field.member) return field.key;
    for (const auto &field : kSteeringFloats)
        if (a.*field.member != b.*field.member) return field.key;
    return "";
}

bool load_control_params(const ControlParamPaths &paths, ControlParams *params, std::string *error) {
    ControlParams candidate = *params;
    std::string load_error;
    if (!load_steering_params(paths.steering, &candidate.steering, &load_error)) {
        if (error) *error = "steering " + paths.steering + ": " + load_error;
        return false;
    }
    if (!load_adaptive_cruise_params(paths.cruise, &candidate.cruise, &load_error)) {
        if (error) *error = "adaptive cruise " + paths.cruise + ": " + load_error;
        return false;
    }
    *params = candidate;
    return true;
}

std::string control_params_summary(const ControlParams &params) {
    char text[160];
    std::snprintf(text, sizeof(text), "adaptiveCruise=%u gap=%.1fm/%.1fs decel=%.1fkph/s",
                  params.cruise.enabled ? 1U : 0U, params.cruise.standstill_gap_m, params.cruise.following_time_s,
                  params.cruise.deceleration_rate_kph_per_s);
    return text;
}

ControlParamsWatcher::ControlParamsWatcher(const ControlParamPaths &paths, uint64_t now_ns)
    : paths_(paths), steering_(paths.steering, kParamPollIntervalNs), cruise_(paths.cruise, kParamPollIntervalNs) {
    steering_.mark_seen(now_ns);
    cruise_.mark_seen(now_ns);
}

bool ControlParamsWatcher::poll(uint64_t now_ns, ControlParams *params) {
    // 둘 다 본다(앞쪽이 바뀌었어도 뒤쪽 지문을 새로 적어 두어야 다음 확인에서 다시 읽지 않는다).
    const bool steering_changed = steering_.changed(now_ns);
    const bool cruise_changed = cruise_.changed(now_ns);
    if (!steering_changed && !cruise_changed) return false;
    std::string error;
    if (!load_control_params(paths_, params, &error)) {
        std::fprintf(stderr, "controlsd: params reload rejected: %s\n", error.c_str());
        return false;
    }
    ++generation_;
    std::fprintf(stderr, "controlsd: params reloaded generation=%u %s\n", generation_,
                 control_params_summary(*params).c_str());
    return true;
}
