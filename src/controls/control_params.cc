#include "controls/control_params.h"

#include "common/utils_json.h"

float SteeringParams::center_to_front_m() const {
  return wheelbase_m * center_to_front_ratio;
}

namespace {

constexpr JsonBoolField<SteeringParams> kSteeringBools[] = {
    {"enabled", &SteeringParams::enabled},
    {"avoid_lkas_fault_enabled", &SteeringParams::avoid_lkas_fault_enabled},
    {"live_bank_compensation", &SteeringParams::live_bank_compensation},
    {"use_live_vehicle_params", &SteeringParams::use_live_vehicle_params},
    {"use_live_torque_params", &SteeringParams::use_live_torque_params},
    {"use_live_delay", &SteeringParams::use_live_delay},
    {"use_locationd_learner_inputs", &SteeringParams::use_locationd_learner_inputs},
};
constexpr JsonIntField<SteeringParams> kSteeringInts[] = {
    {"steering_pressed_threshold", 0, 500, &SteeringParams::steering_pressed_threshold},
};
constexpr JsonFloatField<SteeringParams> kSteeringFloats[] = {
    {"torque_lat_accel_factor", 0.5f, 5.0f, &SteeringParams::torque_lat_accel_factor},
    {"torque_kp", 0.0f, 10.0f, &SteeringParams::torque_kp},
    {"torque_ki", 0.0f, 2.0f, &SteeringParams::torque_ki},
    {"torque_friction", 0.0f, 0.3f, &SteeringParams::torque_friction},
    {"steer_ratio", 8.0f, 25.0f, &SteeringParams::steer_ratio},
    {"tire_stiffness_factor", 0.2f, 2.0f, &SteeringParams::tire_stiffness_factor},
    {"steer_actuator_delay", 0.01f, 1.0f, &SteeringParams::steer_actuator_delay},
    {"avoid_lkas_fault_hold_angle_deg", 0.0f, 180.0f, &SteeringParams::avoid_lkas_fault_hold_angle_deg},
    {"angle_offset_deg", -10.0f, 10.0f, &SteeringParams::angle_offset_deg},
    {"torque_lat_accel_offset", -1.0f, 1.0f, &SteeringParams::torque_lat_accel_offset},
    {"mass_kg", 1000.0f, 2600.0f, &SteeringParams::mass_kg},
    {"wheelbase_m", 2.0f, 3.5f, &SteeringParams::wheelbase_m},
    {"center_to_front_ratio", 0.2f, 0.7f, &SteeringParams::center_to_front_ratio},
    {"path_offset_m", -1.0f, 1.0f, &SteeringParams::path_offset_m},
    {"lane_path_weight", 0.5f, 10.0f, &SteeringParams::lane_path_weight},
    {"min_steer_speed_mps", 0.0f, 5.0f, &SteeringParams::min_steer_speed_mps},
};

constexpr JsonBoolField<DrivingParams> kDrivingBools[] = {
    {"laneless_mode", &DrivingParams::laneless_mode},
    {"turn_desire", &DrivingParams::turn_desire},
};
constexpr JsonFloatField<DrivingParams> kDrivingFloats[] = {
    {"lane_change_min_speed_kph", 0.0f, 80.0f, &DrivingParams::lane_change_min_speed_kph},
};

}  // namespace

bool load_steering_params_json(const std::string &path,
                               SteeringParams *params,
                               std::string *error) {
  if (!params) return false;
  return load_json_param_file(path, [params](const std::string &text) {
    // 2026-09-24 이전 raw 표기. 조용히 기본값으로 떨어지지 않게 거부한다.
    for (const char *key : {"torque_max_lat_accel_raw", "torque_kp_raw", "torque_kf_raw",
                            "torque_ki_raw", "torque_friction_raw"}) {
      if (text.find(std::string("\"") + key + "\"") != std::string::npos)
        throw std::runtime_error(std::string(key) + " was replaced by torque_lat_accel_factor/"
                                 "torque_kp/torque_ki/torque_friction");
    }
    parse_json_fields(text, kSteeringBools, params);
    parse_json_fields(text, kSteeringInts, params);
    parse_json_fields(text, kSteeringFloats, params);
  }, error);
}

bool load_driving_params_json(const std::string &path,
                              DrivingParams *params,
                              std::string *error) {
  if (!params) return false;
  return load_json_param_file(path, [params](const std::string &text) {
    parse_json_fields(text, kDrivingBools, params);
    parse_json_fields(text, kDrivingFloats, params);
  }, error);
}
