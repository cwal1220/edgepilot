#include "controls/lateral_torque.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/utils_math.h"

namespace {

constexpr float kDtCtrl = 0.01f;
constexpr float kFrictionThreshold = 0.2f;
// 저크 선행 마찰: 0.19s 앞의 요청 저크를 1.2Hz LP로 걸러 미리 반영한다.
constexpr int kJerkLookaheadFrames = 19;
constexpr float kJerkGain = 0.3f;
constexpr float kJerkFilterAlpha =
    1.0f / (1.0f + 1.0f / (2.0f * 3.14159265f * 1.2f * 0.01f));
/* openpilot INTERP_SPEEDS. 저속 이득은 이 곡선이 담당하고 LOW_SPEED_Y 곡률항은
 * 쓰지 않는다(상류 PR #36364). 마지막 점만 파라미터(torque_kp)다. */
constexpr float kKpSpeedsMps[] = {1.0f, 1.5f, 2.0f, 3.0f, 5.0f, 7.5f, 10.0f, 15.0f, 30.0f};
constexpr float kKpValues[] = {250.0f, 120.0f, 65.0f, 30.0f, 11.5f, 5.5f, 3.5f, 2.0f};
constexpr float kStdCargoKg = 136.0f;
constexpr float kCivicMass = 1326.0f + kStdCargoKg;
constexpr float kCivicWheelbase = 2.70f;
constexpr float kCivicCenterToFront = kCivicWheelbase * 0.4f;
constexpr float kCivicCenterToRear = kCivicWheelbase - kCivicCenterToFront;
constexpr float kCivicTireStiffnessFront = 192150.0f;
constexpr float kCivicTireStiffnessRear = 202500.0f;
constexpr float kGravity = 9.81f;

// openpilot KP_INTERP. 마지막 점만 파라미터라 interp를 펼쳐 쓴다.
float scheduled_kp(float speed_mps, float kp_top) {
  constexpr int n = static_cast<int>(sizeof(kKpSpeedsMps) / sizeof(kKpSpeedsMps[0]));
  if (speed_mps <= kKpSpeedsMps[0]) return kKpValues[0];
  if (speed_mps >= kKpSpeedsMps[n - 1]) return kp_top;
  for (int i = 1; i < n; ++i) {
    if (speed_mps <= kKpSpeedsMps[i]) {
      const float hi_y = i == n - 1 ? kp_top : kKpValues[i];
      const float p = (speed_mps - kKpSpeedsMps[i - 1]) /
                      (kKpSpeedsMps[i] - kKpSpeedsMps[i - 1]);
      return kKpValues[i - 1] + p * (hi_y - kKpValues[i - 1]);
    }
  }
  return kp_top;
}

float apply_deadzone(float error, float deadzone) {
  if (error > deadzone) return error - deadzone;
  if (error < -deadzone) return error + deadzone;
  return 0.0f;
}

std::pair<float, float> scale_tire_stiffness(float mass, float wheelbase,
                                             float center_to_front,
                                             float tire_stiffness_factor) {
  const float center_to_rear = wheelbase - center_to_front;
  float front = kCivicTireStiffnessFront * tire_stiffness_factor * mass / kCivicMass;
  front *= (center_to_rear / wheelbase) / (kCivicCenterToRear / kCivicWheelbase);
  float rear = kCivicTireStiffnessRear * tire_stiffness_factor * mass / kCivicMass;
  rear *= (center_to_front / wheelbase) / (kCivicCenterToFront / kCivicWheelbase);
  return {front, rear};
}

}  // namespace

// PID와 saturation 상태를 초기화한다.
void TorqueController::reset() {
  p_ = 0.0f;
  i_ = 0.0f;
  f_ = 0.0f;
  normalized_output_ = 0.0f;
  error_ = 0.0f;
  feedforward_ = 0.0f;
  actual_curvature_ = 0.0f;
  // 요청 버퍼/저크 필터는 매 프레임 갱신되므로 비우지 않는다
}

// openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다.
int TorqueController::update(bool active,
                                      float speed_mps,
                                      float desired_curvature,
                                      float steering_angle_deg,
                                      bool steering_pressed,
                                      bool steering_rate_limited,
                                      const SteeringParams &params,
                                      float lat_delay_s,
                                      float yaw_rate_rad_s,
                                      bool yaw_rate_valid,
                                      float road_bank_lat_accel,
                                      const LiveLateralParams &live) {
  const float actual_curvature = estimate_actual_curvature(
      speed_mps, steering_angle_deg, params, yaw_rate_rad_s, yaw_rate_valid, live);

  const float speed_sq = speed_mps * speed_mps;
  const float desired_lat_accel = desired_curvature * speed_sq;
  const float actual_lat_accel = actual_curvature * speed_sq;

  /* 버퍼/저크 필터는 active와 무관하게 매 프레임 갱신한다. inactive에서
   * 얼리면 재engage 첫 delay 구간에 낡은 요청과 비교되어 토크가 튄다. */
  request_head_ = (request_head_ + 1) % kRequestBufferLen;
  lat_accel_request_[request_head_] =
      std::isfinite(desired_lat_accel) ? desired_lat_accel : 0.0f;
  // 상류: delay_frames = int(clip(lat_delay / dt + 1, 1, buffer_len))
  const int delay_frames = clamp_int(
      static_cast<int>((std::isfinite(lat_delay_s) ? lat_delay_s : params.steer_actuator_delay) / kDtCtrl) + 1,
      1, kRequestBufferLen);
  const auto at = [this](int back) {
    return (request_head_ - back + 2 * kRequestBufferLen) % kRequestBufferLen;
  };
  const float expected_lat_accel = lat_accel_request_[at(delay_frames - 1)];

  const int lookahead_back = clamp_int(delay_frames - 1 - kJerkLookaheadFrames,
                                       1, kRequestBufferLen - 2);
  const float raw_jerk = (lat_accel_request_[at(lookahead_back - 1)] -
                          lat_accel_request_[at(lookahead_back + 1)]) /
                         (2.0f * kDtCtrl);
  jerk_filtered_ += kJerkFilterAlpha * (raw_jerk - jerk_filtered_);

  if (!params.enabled || !active || speed_mps < params.min_steer_speed_mps) {
    reset();
    actual_curvature_ = actual_curvature;
    error_ = desired_curvature - actual_curvature;
    return 0;
  }

  const float curvature_deadzone = 0.0f;  // 조향각 deadzone 0 고정

  const float lat_accel_deadzone = curvature_deadzone * speed_sq;

  /* 오차는 횡가속도만으로 낸다. 저속 보강은 LOW_SPEED_Y 곡률항이 아니라
   * 속도별 비례 이득이 맡는다(openpilot PR #36364). */
  const float setpoint = expected_lat_accel;
  const float measurement = actual_lat_accel;
  const float error = setpoint - measurement;

  const Gains g = gains(params, live);
  float feedforward = desired_lat_accel;
  // 상수 편향(offset)은 FF에서 뺀다. bank = -g*sin(도로기울기)이므로
  // 중력의 횡가속 기여(-bank)를 빼려면 bank를 더한다 (2026-08-30 부호 수정)
  feedforward -= g.lat_accel_offset;
  // 학습 롤을 쓰면 상류처럼 roll·g를 빼고, 같은 몫인 편향 추정은 쓰지 않는다
  if (live.use_vehicle) feedforward -= live.roll_rad * kGravity;
  else if (params.live_bank_compensation) feedforward += road_bank_lat_accel;
  const float friction = interp(
      apply_deadzone(error + kJerkGain * jerk_filtered_, lat_accel_deadzone),
      {-kFrictionThreshold, kFrictionThreshold},
      {-g.friction, g.friction});
  feedforward += friction / g.kf;

  const bool freeze_integrator = steering_rate_limited || steering_pressed || speed_mps < 5.0f;
  const float pid_output =
      pid_update(error, feedforward, freeze_integrator, params, g, speed_mps);

  normalized_output_ = clamp_float(static_cast<float>(kTorqueOutputSign) * pid_output, -1.0f, 1.0f);
  error_ = error;
  feedforward_ = feedforward;
  actual_curvature_ = actual_curvature;
  return static_cast<int>(std::lround(normalized_output_ * static_cast<float>(params.steer_max)));
}

// 조향각이 만드는 곡률(차량 모델, 제어 부호).
float TorqueController::curvature_at_angle(float speed_mps,
                                           float steering_angle_deg,
                                           const SteeringParams &params,
                                           const LiveLateralParams &live) {
  // 상류 VM.update_params(max(x, 0.1), max(sr, 0.1)). 강성 배율은 타이어 계수에 곱해진다.
  const SteeringParams *vm = &params;
  float angle_offset_deg = params.angle_offset_deg;
  if (live.use_vehicle) {
    live_vehicle_params_ = params;
    live_vehicle_params_.steer_ratio = std::max(live.steer_ratio, 0.1f);
    live_vehicle_params_.tire_stiffness_factor =
        params.tire_stiffness_factor * std::max(live.stiffness_factor, 0.1f);
    vm = &live_vehicle_params_;
    angle_offset_deg = live.angle_offset_deg;
  }
  /* vm 경로의 부호 반전은 openpilot latcontrol_torque와 동일하다. */
  float curvature = vehicle_model_curvature(
      deg_to_rad(steering_angle_deg - angle_offset_deg), speed_mps, *vm);
  if (live.use_vehicle) curvature += roll_compensation(live.roll_rad, speed_mps);
  return -curvature;
}

/* 현재 조향각·속도에서 차량 모델로 실제 곡률을 낸다(상류 useSteeringAngle, 현대·기아와 같음). ESP12 요레이트
 * 곡률은 주행 로그(curveYaw, fit_lateral_params.py)에 남기려고 함께 계산한다. */
float TorqueController::estimate_actual_curvature(float speed_mps,
                                                           float steering_angle_deg,
                                                           const SteeringParams &params,
                                                           float yaw_rate_rad_s,
                                                           bool yaw_rate_valid,
                                                           const LiveLateralParams &live) {
  actual_curvature_vm_ = 0.0f;
  actual_curvature_yaw_ = 0.0f;
  if (!std::isfinite(speed_mps) || speed_mps < params.min_steer_speed_mps) return 0.0f;
  const float actual_curvature_vm = curvature_at_angle(speed_mps, steering_angle_deg, params, live);
  actual_curvature_vm_ = actual_curvature_vm;
  float actual_curvature_yaw = actual_curvature_vm;
  if (yaw_rate_valid && std::isfinite(yaw_rate_rad_s)) {
    /* K7 ESP12 YAW_RATE는 제어 관례와 부호가 반대라 반전한다. 2026-08-15
     * 실차 수동 주행의 커브 53샘플에서 curveVm/curveYaw 부호가 96% 반대,
     * 크기 비율 평균 1.15로 확인했다. */
    actual_curvature_yaw = -yaw_rate_rad_s / std::max(speed_mps, 1e-3f);
  }
  actual_curvature_yaw_ = actual_curvature_yaw;
  return actual_curvature_vm;
}

TorqueController::Gains TorqueController::gains(const SteeringParams &params,
                                                const LiveLateralParams &live) {
  /* 상류는 PID를 횡가속 공간에서 돌리고 끝에서 latAccelFactor로 나눈다. 여기 PID는
   * 토크 공간이라 FF·P·I에 모두 1/latAccelFactor를 곱한다. 마찰은 상류도 토크 공간이다. */
  const bool learned = live.use_torque && std::isfinite(live.lat_accel_factor) &&
                       live.lat_accel_factor > 0.0f;
  const float factor = learned ? live.lat_accel_factor : params.torque_lat_accel_factor;
  Gains g;
  g.kf = 1.0f / factor;
  g.ki = params.torque_ki / factor;
  g.friction = learned ? live.friction : params.torque_friction;
  g.lat_accel_offset = learned ? live.lat_accel_offset : params.torque_lat_accel_offset;
  return g;
}

// 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
void TorqueController::update_vehicle_model(const SteeringParams &params) {
  const float center_to_front = params.center_to_front_m();
  // 학습 SR·강성은 매 틱 조금씩 바뀌므로 정확히 같을 때만 건너뛴다(상류는 매번 계산)
  if (last_mass_kg_ == params.mass_kg && last_wheelbase_m_ == params.wheelbase_m &&
      last_center_to_front_m_ == center_to_front &&
      last_tire_stiffness_factor_ == params.tire_stiffness_factor &&
      last_steer_ratio_ == params.steer_ratio &&
      last_steer_ratio_rear_ == params.steer_ratio_rear) {
    return;
  }
  const float center_to_rear = params.wheelbase_m - center_to_front;
  const auto stiffness = scale_tire_stiffness(
      params.mass_kg, params.wheelbase_m, center_to_front, params.tire_stiffness_factor);
  const float denominator =
      params.wheelbase_m * params.wheelbase_m * stiffness.first * stiffness.second;
  slip_factor_ = std::fabs(denominator) < 1e-9f
      ? 0.0f
      : params.mass_kg * (stiffness.first * center_to_front -
                          stiffness.second * center_to_rear) / denominator;
  last_mass_kg_ = params.mass_kg;
  last_wheelbase_m_ = params.wheelbase_m;
  last_center_to_front_m_ = center_to_front;
  last_tire_stiffness_factor_ = params.tire_stiffness_factor;
  last_steer_ratio_ = params.steer_ratio;
  last_steer_ratio_rear_ = params.steer_ratio_rear;
}

// 조향각과 속도에서 실제 curvature를 계산한다.
float TorqueController::vehicle_model_curvature(float steering_angle_rad,
                                                         float speed_mps,
                                                         const SteeringParams &params) {
  update_vehicle_model(params);
  float denom = 1.0f - slip_factor_ * speed_mps * speed_mps;
  if (std::fabs(denom) < 1e-6f) denom = denom >= 0.0f ? 1e-6f : -1e-6f;
  const float curvature_factor =
      (1.0f - params.steer_ratio_rear) / denom / params.wheelbase_m;
  return curvature_factor * steering_angle_rad / params.steer_ratio;
}

float TorqueController::roll_compensation(float roll_rad, float speed_mps) const {
  if (std::fabs(slip_factor_) < 1e-6f) return 0.0f;
  return kGravity * roll_rad / ((1.0f / slip_factor_) - speed_mps * speed_mps);
}

// PID 한 스텝을 계산한다.
float TorqueController::pid_update(float error,
                                            float feedforward,
                                            bool freeze_integrator,
                                            const SteeringParams &params,
                                            const Gains &gains,
                                            float speed_mps) {
  /* 이득 곡선은 횡가속도 공간이므로 kf를 곱해 토크 공간으로 옮긴다. */
  p_ = error * scheduled_kp(speed_mps, params.torque_kp) * gains.kf;
  f_ = feedforward * gains.kf;
  const float next_i = i_ + error * gains.ki * kDtCtrl;
  const float control_with_i = p_ + next_i + f_;
  if (((error >= 0.0f && (control_with_i <= 1.0f || next_i < 0.0f)) ||
       (error <= 0.0f && (control_with_i >= -1.0f || next_i > 0.0f))) &&
      !freeze_integrator) {
    i_ = next_i;
  }
  return clamp_float(p_ + i_ + f_, -1.0f, 1.0f);
}
