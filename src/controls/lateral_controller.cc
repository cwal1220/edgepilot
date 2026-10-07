#include "controls/lateral_controller.h"

#include "common/utils_math.h"
#include "common/utils_time.h"
#include "common/model_output.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kSteeringPressedMinCount = 5;
constexpr double kPandaEngageGraceS = 1.0;
// 운전자가 넘겨받은 큰 회전에서 조향을 다시 켜는 각도(carrotpilot lat_suspend_control의 resume_angle)
constexpr float kDriverReleaseAngleDeg = 15.0f;

float interp_lateral(float x, const float *values) {
  if (x <= 0.0f) return values[0];
  for (int i = 1; i < kLateralControlN; ++i) {
    const float high_x = model_t_idx(i);
    if (x <= high_x) {
      const float low_x = model_t_idx(i - 1);
      const float p = (x - low_x) / (high_x - low_x);
      return values[i - 1] + p * (values[i] - values[i - 1]);
    }
  }
  return values[kLateralControlN - 1];
}

// SteeringParams의 토크 제한을 CAN 계층 표현으로 옮긴다.
HyundaiSteeringLimits hyundai_limits(const SteeringParams &params) {
  HyundaiSteeringLimits out;
  out.steer_max = params.steer_max;
  out.steer_delta_up = params.steer_delta_up;
  out.steer_delta_down = params.steer_delta_down;
  out.steer_driver_allowance = params.steer_driver_allowance;
  out.steer_driver_multiplier = params.steer_driver_multiplier;
  out.steer_driver_factor = params.steer_driver_factor;
  return out;
}

}  // namespace

LateralController::LateralController(LateralControllerConfig config)
    : config_(config), clock_(monotonic_now_ns) {}

// 제어 상태를 유지한 채 런타임 파라미터를 즉시 교체한다.
void LateralController::update_params(
    const SteeringParams &steering_params,
    const DrivingParams &driving_params) {
  config_.steering_params = steering_params;
  config_.driving_params = driving_params;
}

void LateralController::set_live_params(const LiveLateralParams &live, bool vehicle_valid,
                                        bool calibrated) {
  live_ = live;
  live_vehicle_valid_ = vehicle_valid;
  live_calibrated_ = calibrated;
}

void LateralController::set_live_delay(float delay_s, bool valid) {
  live_delay_s_ = delay_s;
  live_delay_valid_ = valid && std::isfinite(delay_s);
}

bool LateralController::live_delay_in_use() const {
  return config_.steering_params.use_live_delay && live_delay_valid_;
}

float LateralController::plan_delay_s() const {
  // lagd 지연 범위(lateral_lag.h min_lag~max_lag)
  return live_delay_in_use() ? clamp_float(live_delay_s_, 0.15f, 0.65f)
                             : config_.steering_params.steer_actuator_delay;
}

LiveLateralParams LateralController::live_params() const {
  LiveLateralParams live = live_;
  live.use_vehicle = live_.use_vehicle && config_.steering_params.use_live_vehicle_params;
  live.use_torque = live_.use_torque && config_.steering_params.use_live_torque_params;
  return live;
}

// 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
LateralControlResult LateralController::update(const LateralPath &path,
                                                   const LateralTarget &target,
                                                   const VehicleCanState &vehicle_state,
                                                   double now_s,
                                                   int frame,
                                                   bool panda_ready,
                                                   bool panda_controls_allowed) {
  const bool engage_requested =
      !config_.force_engaged && !engaged_ &&
      vehicle_state.clu_button == 0 && last_button_ == kCruiseButtonSet;
  update_button_state(vehicle_state.clu_button, now_s);
  const bool logical_engaged = config_.force_engaged || engaged_;

  LateralControlResult result;
  result.engaged = logical_engaged;
  const LateralPath gated_path = debounce_path(path, now_s);
  result.path_usable = gated_path.usable_for_steering;
  result.left_lane = path.left_valid;
  result.right_lane = path.right_valid;
  result.seeds_ready = seed_frames_ready(vehicle_state);
  result.vehicle_fresh = vehicle_state_fresh(
      vehicle_state, now_s,
      static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) / 1000.0);
  result.cluster_speed_kph = cluster_speed_kph(vehicle_state);
  result.control_speed_kph = vehicle_speed_kph(
      vehicle_state, now_s,
      static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) / 1000.0);
  const float speed_mps = result.control_speed_kph / 3.6f;
  const LiveLateralParams live = live_params();
  const float plan_age = plan_age_s(target);
  result.active_block = active_block_reason(gated_path, target, vehicle_state,
                                            result.seeds_ready, result.vehicle_fresh,
                                            panda_ready, panda_controls_allowed,
                                            result.control_speed_kph, plan_age);
  const BlockKind kind = block_kind(result.active_block);
  resolve_engagement(kind, logical_engaged, engage_requested, now_s, &result);
  /* 가용성 대기 중에는 토크만 0으로 하고 steer_req/MDPS 속도 스푸프는
   * 유지한다 — 매 정차마다 끊기면 MDPS/클러스터가 천이 경보를 낸다.
   * 결함/해제는 즉시 끊는다. plan 무효는 engage는 거부하되 steer_req는 잡아둔다. */
  steer_availability_hold_ = logical_engaged && !result.active &&
      (kind == BlockKind::Availability ||
       result.active_block == BlockReason::LateralPlanInvalid);
  const bool steering_pressed = update_steering_pressed(vehicle_state.driver_torque);
  result.steering_pressed = steering_pressed;
  result.cut_steer_temp = update_cut_steer_state(vehicle_state);
  result.large_angle_hold = update_large_angle_hold(result.active || steer_availability_hold_,
                                                    vehicle_state.steering_angle_deg, steering_pressed);
  result.large_angle_hold_by_driver = result.large_angle_hold && driver_took_wheel_;
  // 요청을 끈 동안은 활성이라도 조향을 쉰다(상류 latActive=false처럼)
  const bool steering = result.active && !result.large_angle_hold;
  const bool yaw_rate_valid = signal_time_fresh(
                                  vehicle_state.esp12_time_s, now_s,
                                  static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) /
                                      1000.0) &&
                              vehicle_state.yaw_rate_valid;
  update_road_bank(vehicle_state, speed_mps, yaw_rate_valid);

  bool curvature_limited = false;
  result.desired_curvature =
      clip_curvature(speed_mps, prev_desired_curvature_,
                     requested_curvature(target, vehicle_state, speed_mps, plan_age, steering,
                                         steering_pressed, yaw_rate_valid, live),
                     live.use_vehicle ? live.roll_rad : 0.0f, &curvature_limited);
  prev_desired_curvature_ = result.desired_curvature;

  if (steering) {
    steer(target, vehicle_state, speed_mps, now_s, steering_pressed, yaw_rate_valid, curvature_limited, live,
          &result);
  } else {
    sat_time_ = 0.0f;
    // 0을 넘기면 커브 중 engage 시 지연 버퍼가 0-setpoint로 P를 튀게 한다
    torque_controller_.update(false, speed_mps, result.desired_curvature,
                              vehicle_state.steering_angle_deg,
                              false, steer_rate_limited_, config_.steering_params, plan_delay_s(),
                              vehicle_state.yaw_rate_rad_s, yaw_rate_valid,
                              road_bank_lat_accel_, live);
    copy_torque_state(&result);
    result.desired_torque = 0;
    result.apply_torque = 0;
  }

  /* 결합 중이거나 해제 직후(inactive_release_ms) 동안은 토크 0으로 계속 보내 MDPS가 부드럽게 놓게
   * 한다. */
  result.should_send =
      result.seeds_ready &&
      (result.active || result.engaged ||
       now_s - last_disengage_s_ <
           static_cast<double>(config_.driving_params.inactive_release_ms) / 1000.0);
  if (result.should_send) {
    result.frames = build_frames(vehicle_state, result, frame);
  } else {
    lkas11_counter_valid_ = false;
  }

  steer_req_sent_ = result.should_send && (result.active || steer_availability_hold_) &&
                    !result.large_angle_hold && !result.cut_steer_temp;
  last_torque_ = result.apply_torque;
  steer_rate_limited_ = result.desired_torque != result.apply_torque;
  if (!result.active) {
    last_torque_ = 0;
    steer_rate_limited_ = false;
  }
  return result;
}

// path 복귀 디바운스: 무효는 즉시 반영, 무효를 겪은 뒤의 복귀는 0.5s
// 연속 유효를 요구한다(경계 깜빡임 차단). 최초 유효는 바로 통과.
LateralPath LateralController::debounce_path(const LateralPath &path, double now_s) {
  if (!path.usable_for_steering) {
    path_valid_since_s_ = -1.0;
    path_usable_debounced_ = false;
    path_seen_invalid_ = true;
  } else if (!path_usable_debounced_) {
    if (path_valid_since_s_ < 0.0) path_valid_since_s_ = now_s;
    if (!path_seen_invalid_ || now_s - path_valid_since_s_ >= 0.5)
      path_usable_debounced_ = true;
  }
  LateralPath gated_path = path;
  gated_path.usable_for_steering = path_usable_debounced_;
  if (!path_usable_debounced_) gated_path.invalid_reason = "path_invalid";
  return gated_path;
}

/* plan 나이: 근거 프레임 캡처 시각부터 지금까지. lag 보상과 staleness
 * gate가 함께 쓴다. 타임스탬프가 없으면(테스트, 초기값) 0으로 둔다. */
float LateralController::plan_age_s(const LateralTarget &target) const {
  float plan_age_s = 0.0f;
  if (target.valid && target.capture_timestamp_ns != 0) {
    const uint64_t control_now_ns = clock_();
    if (control_now_ns > target.capture_timestamp_ns) {
      plan_age_s = static_cast<float>(
          static_cast<double>(control_now_ns - target.capture_timestamp_ns) * 1e-9);
    }
  }
  return plan_age_s;
}

// 결합과 Panda 유예를 풀고 제어 상태를 처음으로 돌린다(해제·engage 거부·CANCEL 공통).
void LateralController::disengage(double now_s) {
  panda_engage_pending_ = false;
  engaged_ = false;
  reset_control_state();
  last_disengage_s_ = now_s;
}

/* 이번 틱의 차단 사유(kind)로 해제·engage 거부·Panda 유예를 정하고, result의 engaged,
 * soft_disabling, engage_rejected, active를 채운다. */
void LateralController::resolve_engagement(BlockKind kind, bool logical_engaged, bool engage_requested,
                                           double now_s, LateralControlResult *result) {
  /* SoftDisable: engage 중이면 3초간 조향을 유지하며 경고하고, 그 뒤에는 Hard처럼 해제한다. */
  bool soft_disable_expired = false;
  if (logical_engaged && !engage_requested && kind == BlockKind::SoftDisable) {
    if (soft_disable_start_s_ < 0.0) soft_disable_start_s_ = now_s;
    soft_disable_expired = now_s - soft_disable_start_s_ >= kSoftDisableS;
    result->soft_disabling = !soft_disable_expired;
  } else {
    soft_disable_start_s_ = -1.0;
  }
  if (logical_engaged && (kind == BlockKind::Hard || soft_disable_expired)) {
    disengage(now_s);
    result->engaged = config_.force_engaged;
  }

  if (engage_requested) {
    if (kind == BlockKind::Transient) {
      panda_engage_pending_ = true;
      panda_engage_pending_s_ = now_s;
    } else {
      panda_engage_pending_ = false;
    }
  }

  if (engage_requested && (kind == BlockKind::Hard || kind == BlockKind::Reject ||
                           kind == BlockKind::SoftDisable)) {
    // 차량/컨트롤러의 정적 gate는 실제 engage 요청 실패로 처리한다.
    disengage(now_s);
    result->engaged = config_.force_engaged;
    result->engage_rejected = true;
  }

  if (panda_engage_pending_) {
    const bool panda_waiting = kind == BlockKind::Transient;
    const bool grace_elapsed = now_s - panda_engage_pending_s_ >= kPandaEngageGraceS;
    if (kind == BlockKind::None || kind == BlockKind::Availability) {
      // Panda 허가가 도착했고 나머지는 가용성 상태뿐이면 engage를 유지한다.
      panda_engage_pending_ = false;
    } else if (!panda_waiting || grace_elapsed) {
      /* 정적 실패를 저장했다가 gate가 해소되면 조용히 engage하지 않는다. Panda에는
       * 짧은 비동기 health handshake 유예만 허용하며, 완료되지 않으면 실제 차단
       * 사유를 한 번 보고한다. */
      disengage(now_s);
      result->engaged = config_.force_engaged;
      result->engage_rejected = true;
    }
  }
  result->active = kind == BlockKind::None || result->soft_disabling;
}

/* 편경사: 직선(|yaw*v| < 0.4)에서만 갱신, 커브는 홀드 — 커브에서는 차체 롤
 * 중력 누설이 섞인다(2026-08-30 drive10: 커브 방향 반상관 ±0.2~0.5 + 탈출 꼬리).
 * rc 2초. 실측 검증식 (2026-08-27, 직선 -0.117 재현): bank = lat + yaw_rate*v. */
void LateralController::update_road_bank(const VehicleCanState &vehicle_state, float speed_mps,
                                         bool yaw_rate_valid) {
  constexpr float kBankAlpha = 0.01f / (2.0f + 0.01f);
  if (yaw_rate_valid && vehicle_state.lat_accel_valid &&
      std::isfinite(vehicle_state.lat_accel_mps2) && speed_mps > 8.0f &&
      std::fabs(vehicle_state.yaw_rate_rad_s * speed_mps) < 0.4f) {
    const float bank = clamp_float(
        vehicle_state.lat_accel_mps2 + vehicle_state.yaw_rate_rad_s * speed_mps,
        -2.0f, 2.0f);
    if (!road_bank_init_) { road_bank_lat_accel_ = bank; road_bank_init_ = true; }
    road_bank_lat_accel_ += kBankAlpha * (bank - road_bank_lat_accel_);
    road_bank_stale_frames_ = 0;
  } else if (++road_bank_stale_frames_ > 3000) {
    // 30초 넘게 갱신이 없으면 낡은 편경사를 0으로 감쇠하고 재초기화를 허용
    road_bank_lat_accel_ += kBankAlpha * (0.0f - road_bank_lat_accel_);
    road_bank_init_ = false;
  }
}

/* 클립 전 목표 곡률. 상류 controlsd: 활성이면 plan, 비활성이면 실제 곡률을 클립에 넣는다. 재활성 때
 * 목표가 실제 곡률에서 한계 안으로 출발하고, 비활성 중의 잘못된 plan 값이 넘어오지 않는다. */
float LateralController::requested_curvature(const LateralTarget &target, const VehicleCanState &vehicle_state,
                                             float speed_mps, float plan_age_s, bool steering,
                                             bool steering_pressed, bool yaw_rate_valid,
                                             const LiveLateralParams &live) {
  const SteeringParams &control_params = config_.steering_params;
  float requested_curvature = steering
      ? lag_adjusted_curvature(target, speed_mps, plan_age_s, plan_delay_s())
      : torque_controller_.estimate_actual_curvature(speed_mps, vehicle_state.steering_angle_deg,
                                                     control_params, vehicle_state.yaw_rate_rad_s,
                                                     yaw_rate_valid, live);
  /* 운전자가 핸들을 잡지 않았을 때는 목표를 avoid_lkas_fault_hold_angle_deg(80도) 조향각이 내는
   * 곡률 안으로 묶는다(같은 차량 모델, 학습 SR·강성·오프셋·롤). 컨트롤러가 스스로 85도를 넘겨
   * 0.89초 뒤 토크가 빠지고 핸들이 풀렸다 다시 잡는 반복 대신, 그 각도에서 토크를 끊김 없이
   * 유지한다. 운전자가 조향 중이면 묶지 않는다: 교차로에서 더 감는 운전자를 80도 쪽으로 밀지
   * 않고, 85도 위에서는 시간 규칙이 그대로 적용된다. 횡가속 한계가 더 좁은 약 36 km/h 위에서는
   * 걸리지 않는다. */
  const float hold_angle_deg = control_params.avoid_lkas_fault_hold_angle_deg;
  if (steering && !steering_pressed && control_params.avoid_lkas_fault_enabled &&
      hold_angle_deg > 0.0f && std::isfinite(speed_mps) && std::isfinite(requested_curvature)) {
    const float left = torque_controller_.curvature_at_angle(speed_mps, hold_angle_deg,
                                                             control_params, live);
    const float right = torque_controller_.curvature_at_angle(speed_mps, -hold_angle_deg,
                                                              control_params, live);
    requested_curvature = std::clamp(requested_curvature, std::min(left, right),
                                     std::max(left, right));
  }
  return requested_curvature;
}

// 토크 제어기의 진단값을 결과에 옮긴다.
void LateralController::copy_torque_state(LateralControlResult *result) const {
  result->actual_curvature = torque_controller_.actual_curvature();
  result->actual_curvature_vm = torque_controller_.actual_curvature_vm();
  result->actual_curvature_yaw = torque_controller_.actual_curvature_yaw();
  result->curvature_error = result->desired_curvature - result->actual_curvature;
  result->normalized_output = torque_controller_.normalized_output();
  result->feedforward = torque_controller_.feedforward();
}

// 조향 중: 토크를 계산하고 85도 고장 회피·회전 desire·Panda 한계를 적용한 뒤 포화를 본다.
void LateralController::steer(const LateralTarget &target, const VehicleCanState &vehicle_state,
                              float speed_mps, double now_s, bool steering_pressed, bool yaw_rate_valid,
                              bool curvature_limited, const LiveLateralParams &live,
                              LateralControlResult *result) {
  const SteeringParams &control_params = config_.steering_params;
  /* 85도 위에서도 컨트롤러 토크를 그대로 내되, 요청을 끄는 프레임(avoid_lkas_fault_max_frames,
   * update_large_angle_hold)에 0에 닿도록 크기를 steer_delta_down × 남은 프레임으로 묶는다.
   * 하강 레이트 한계로 내려와도 늦지 않는 만큼만 남기는 것이라, 최대 토크로 85도를 넘으면
   * 약 0.35초는 그대로 두고 0.55초에 걸쳐 내린다(2026-10-03까지는 0.69초 선형 램프로 0).
   * 요청은 토크가 0일 때 꺼지므로 어시스트가 빠지는 "탁"이 없다. 적분기는 85도 위에서 얼린다. */
  const bool above_fault_angle =
      control_params.avoid_lkas_fault_enabled &&
      std::fabs(vehicle_state.steering_angle_deg) >=
          control_params.avoid_lkas_fault_max_angle_deg;
  int torque_cap = control_params.steer_max;
  if (result->large_angle_hold) {
    torque_cap = 0;
  } else if (fault_angle_frames_ > 0) {
    torque_cap = std::min(torque_cap, control_params.steer_delta_down *
        std::max(0, control_params.avoid_lkas_fault_max_frames - fault_angle_frames_));
  }
  const int raw_torque = torque_controller_.update(
      true, speed_mps, result->desired_curvature, vehicle_state.steering_angle_deg,
      steering_pressed, steer_rate_limited_ || above_fault_angle, control_params, plan_delay_s(),
      vehicle_state.yaw_rate_rad_s, yaw_rate_valid, road_bank_lat_accel_, live);
  result->desired_torque = std::clamp(raw_torque, -torque_cap, torque_cap);
  /* 회전 desire 중 운전자가 깜빡이 방향으로 돌리고 있으면(운전자 토크 > steer_driver_allowance)
   * 그 반대 방향 토크는 내지 않는다. 걷는 속도에서 모델 계획이 회전을 놓치면 회전에 들어가는
   * 운전자를 밀었다(2026-10-03 8:16·8:48). panda 운전자 클램프도 반대 토크를 줄이지만 운전자
   * 토크가 242를 넘어야 0이 된다. 토크와 운전자 토크는 같은 부호계다(+ = 왼쪽). */
  if (target.turn_desire != 0) {
    const int toward = target.turn_desire == 1 ? 1 : -1;
    if (vehicle_state.driver_torque * toward > control_params.steer_driver_allowance)
      result->desired_torque = toward > 0 ? std::max(result->desired_torque, 0)
                                          : std::min(result->desired_torque, 0);
  }
  copy_torque_state(result);
  result->apply_torque = apply_hyundai_steer_torque_limits(
      result->desired_torque, last_torque_, vehicle_state.driver_torque,
      hyundai_limits(control_params));
  result->steer_saturated = update_saturation(*result, speed_mps, now_s, steering_pressed, curvature_limited);
}

/* openpilot LatControl._check_saturation + selfdrived steerSaturated. 출력이 한계에
 * 붙었거나 곡률이 횡가속 한계에 잘렸는데, 안전 한계(토크 레이트)나 운전자 때문이
 * 아닐 때 0.4초 누적되면 포화다. 커브에서 목표 횡가속이 실제의 1.2배를 넘고 1 m/s²
 * 이상이며 최근 2초 안에 핸들을 잡지 않았으면 경고한다. */
bool LateralController::update_saturation(const LateralControlResult &result, float speed_mps, double now_s,
                                          bool steering_pressed, bool curvature_limited) {
  constexpr float kDt = 0.01f;
  const bool saturated_now = std::fabs(result.normalized_output) >= 1.0f - 1e-3f ||
                             curvature_limited;
  if (saturated_now && speed_mps > kSatCheckMinSpeedMps && !steer_rate_limited_ &&
      !steering_pressed)
    sat_time_ += kDt;
  else
    sat_time_ -= kDt;
  sat_time_ = clamp_float(sat_time_, 0.0f, kSteerLimitTimerS);
  const bool lac_saturated = sat_time_ > kSteerLimitTimerS - 1e-3f;
  if (steering_pressed) last_steering_pressed_s_ = now_s;
  const bool recent_steer_pressed = now_s - last_steering_pressed_s_ < 2.0;
  const float clipped_speed = std::max(speed_mps, 0.3f);
  const float desired_lat_accel = result.desired_curvature * clipped_speed * clipped_speed;
  const float actual_lat_accel = result.actual_curvature * clipped_speed * clipped_speed;
  const bool undershooting =
      std::fabs(desired_lat_accel) / std::fabs(1e-3f + actual_lat_accel) > 1.2f;
  const bool turning = std::fabs(desired_lat_accel) > 1.0f;
  return !recent_steer_pressed && undershooting && turning && lac_saturated;
}

// CLU 버튼 edge로 engage/disengage 상태를 갱신한다.
void LateralController::update_button_state(int button, double now_s) {
  if (button == last_button_) return;
  if (button == kCruiseButtonCancel) {
    disengage(now_s);
  } else if (button == 0 && last_button_ == kCruiseButtonSet) {
    engaged_ = true;
  }
  last_button_ = button;
}

// avoid_lkas_fault를 끈 경우: MDPS 오류가 이어지면 steer 요청을 잠깐 끊는다(openpilot 방식).
bool LateralController::update_cut_steer_state(const VehicleCanState &vehicle_state) {
  const SteeringParams &params = config_.steering_params;
  if (params.avoid_lkas_fault_enabled) return false;
  if (vehicle_state.mdps_error_count > params.avoid_lkas_fault_max_frames) {
    cut_steer_ = true;
  } else if (cut_steer_frames_ >= std::max(1, params.avoid_lkas_fault_cut_frames)) {
    cut_steer_frames_ = 0;
    cut_steer_ = false;
  }
  if (!cut_steer_) return false;
  ++cut_steer_frames_;
  return true;
}

/* 큰 조향각 고장 회피. K7 MDPS는 steer 요청이 켜진 채 |조향각|이 85도 위에 0.98~1.12초 머물면
 * 토크와 무관하게 ToiFlt+FailStat을 내고, 85도 아래로 와야 푼다(2026-09-18 실측). 요청을 내는
 * 동안(active, 또는 정차 같은 가용성 대기) 85도 위 체류를 세어 avoid_lkas_fault_max_frames에 닿으면
 * 요청을 끈다. 그 프레임까지 토크는 update가 0으로 내려 둔다. openpilot식 2프레임 컷은 85도 위에서
 * 요청을 다시 켜는 순간 3~14 ms 뒤 고장을 냈고(2026-10-03, 4번 중 4번), 정차 대기 중에는 세지도
 * 않아 1초 뒤 고장이 났다. 그래서 85도 위에서는 요청을 새로 켜지 않는다(결합이나 차단 해제가
 * 85도 위에서 일어나도 같다).
 * 끈 요청은 85도 아래로 오면 다시 켠다. 다만 그 체류 중 운전자가 핸들을 돌렸으면(넘겨받은 회전)
 * 핸들이 kDriverReleaseAngleDeg(15도) 아래로 오고 손을 뗄 때까지 끈 채로 둔다(carrotpilot
 * lat_suspend_control과 같은 해제 조건). 회전을 빠져나오며 핸들을 펴는 운전자를 시스템이 미리
 * 밀지 않는다(2026-10-03 2:54 탈출에서 최대 213). */
bool LateralController::update_large_angle_hold(bool steer_requested, float steering_angle_deg,
                                                bool steering_pressed) {
  const SteeringParams &params = config_.steering_params;
  const float angle = std::fabs(steering_angle_deg);
  const bool above = params.avoid_lkas_fault_enabled && angle >= params.avoid_lkas_fault_max_angle_deg;
  if (above && steer_requested && steering_pressed) driver_took_wheel_ = true;
  if (!above) {
    fault_angle_frames_ = 0;
    if (large_angle_hold_ && driver_took_wheel_ &&
        (angle >= kDriverReleaseAngleDeg || steering_pressed))
      return true;
    large_angle_hold_ = false;
    driver_took_wheel_ = false;
    return false;
  }
  if (!steer_requested) {
    fault_angle_frames_ = 0;
    return large_angle_hold_;
  }
  if (!steer_req_sent_) {
    large_angle_hold_ = true;
  } else if (!large_angle_hold_ &&
             ++fault_angle_frames_ >= params.avoid_lkas_fault_max_frames) {
    large_angle_hold_ = true;
  }
  return large_angle_hold_;
}

// 노이즈가 있는 운전자 조향 토크를 openpilot 방식으로 필터링한다.
bool LateralController::update_steering_pressed(int driver_torque) {
  const bool pressed =
      std::abs(driver_torque) > config_.steering_params.steering_pressed_threshold;
  steering_pressed_counter_ += pressed ? 1 : -1;
  steering_pressed_counter_ =
      std::clamp(steering_pressed_counter_, 0, kSteeringPressedMinCount * 2 + 1);
  return steering_pressed_counter_ > kSteeringPressedMinCount;
}

// 제어 내부 상태를 초기값으로 되돌린다.
void LateralController::reset_control_state() {
  last_torque_ = 0;
  steer_rate_limited_ = false;
  fault_angle_frames_ = 0;
  large_angle_hold_ = false;
  driver_took_wheel_ = false;
  cut_steer_frames_ = 0;
  cut_steer_ = false;
  torque_controller_.reset();
}

// active를 막는 현재 gate reason을 계산한다.
BlockReason LateralController::active_block_reason(
    const LateralPath &path,
    const LateralTarget &target,
    const VehicleCanState &vehicle_state,
    bool seeds_ready,
    bool vehicle_fresh,
    bool panda_ready,
    bool panda_controls_allowed,
    float speed_kph,
    float plan_age_s) const {
  /* 순서 규칙: 데이터 유효성 -> 차량 결함(hard disengage) -> 핸드셰이크 ->
   * 가용성 대기. 결함이 뒤로 밀리면 앞선 일시적 사유가 결함을 가리고, 그
   * 사이 engage가 유예되어 톤만 울렸다가 해제된다. */
  if (!config_.force_engaged && !engaged_) return BlockReason::NotEngaged;
  if (!config_.steering_params.enabled) return BlockReason::ControllerDisabled;
  if (!seeds_ready) return BlockReason::SeedsMissing;
  if (!vehicle_fresh) return BlockReason::VehicleStateStale;
  if (!std::isfinite(speed_kph)) return BlockReason::SpeedInvalid;
  if (vehicle_state.door_open) return BlockReason::DoorOpen;
  if (vehicle_state.seatbelt_unlatched) return BlockReason::SeatbeltUnlatched;
  if (vehicle_state.esp_disabled) return BlockReason::EspDisabled;
  if (vehicle_state.park_brake) return BlockReason::ParkBrake;
  if (vehicle_state.brake_error) return BlockReason::BrakeError;
  if (vehicle_state.gear != kGearDrive) return BlockReason::GearNotDrive;
  if (vehicle_state.steering_fault) return BlockReason::MdpsFault;
  if (live_params().use_vehicle && !live_vehicle_valid_ && live_calibrated_)
    return BlockReason::ParamsdInvalid;
  /* Panda 핸드셰이크는 차량 결함 뒤에 온다. 앞에 두면 시동 직후 health가
   * 도착하기 전의 engage 요청이 panda_not_ready(일시적)로 분류되어 유예되고,
   * 안전벨트/기어 같은 하드 결함이 가려진 채 engage 톤이 울린 뒤 해제된다. */
  if (!panda_ready) return BlockReason::PandaNotReady;
  if (!panda_controls_allowed) return BlockReason::PandaControlsOff;
  if (!path.usable_for_steering) {
    /* 정지에서는 plan이 원래 짧아 path 무효가 정상이다. 오류가 아니라
     * 대기로 보고한다. 이 속도 밑은 min_steer_speed로 토크도 0이다. */
    if (speed_kph / 3.6f < config_.steering_params.min_steer_speed_mps)
      return BlockReason::Stopped;
    return BlockReason::PathInvalid;
  }
  if (!target.valid || !target.mpc_solution_valid) return BlockReason::LateralPlanInvalid;
  /* 모델 경로 gate는 모델 발행 시각만 본다. 플래너 스레드가 멈춰 target이
   * 갱신되지 않는 경우까지 근거 프레임 캡처 시각으로 함께 막는다. */
  if (target.capture_timestamp_ns != 0 &&
      plan_age_s > static_cast<float>(config_.driving_params.model_timeout_ms) /
                       1000.0f) {
    return BlockReason::LateralPlanStale;
  }
  // openpilot calibrationIncomplete/Recalibrating/Invalid. SoftDisable이라 맨 끝에 둔다.
  if (calibration_status_ == 0) return BlockReason::CalibrationIncomplete;
  if (calibration_status_ == 3) return BlockReason::CalibrationRecalibrating;
  if (calibration_status_ != 1) return BlockReason::CalibrationInvalid;
  return BlockReason::None;
}

float lag_adjusted_curvature(const LateralTarget &target, float speed_mps, float plan_age_s,
                             float steer_actuator_delay_s) {
  /* plan은 카메라 캡처 시점 기준이므로 소비 시점까지의 실측 나이를 actuator
   * delay에 더해 보간한다. 부수 효과로 desired curvature가 20Hz 계단 대신
   * 매 tick plan 위를 따라 전진한다. */
  const float delay = std::max(0.01f, steer_actuator_delay_s) +
      clamp_float(plan_age_s, 0.0f, kMaxPlanAgeCompS);
  const float current_curvature = target.curvatures[0];
  const float psi = interp_lateral(delay, target.psis);
  // openpilot drive_helpers.MIN_SPEED. 하한이 낮으면 psi/(v*delay)가 정지
  // 부근에서 발산해 작은 plan 오차가 곡률 상한까지 증폭된다.
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  const float curvature_from_psi = psi / (speed * delay);
  return current_curvature + 2.0f * (curvature_from_psi - current_curvature);
}

float clip_curvature(float speed_mps, float prev_curvature, float new_curvature,
                     float roll_rad, bool *limited) {
  if (limited) *limited = false;
  // 바퀴 속도가 끊기면 NaN이다. 그대로 두면 clamp_float가 -kMaxCurvature를 낸다.
  if (!std::isfinite(speed_mps) || !std::isfinite(new_curvature)) return prev_curvature;
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  /* ISO 횡저크 한계를 직전 출력 기준 틱당 변화율로 건다. 플랜 노드 기준 창이던
   * v0.9.4와 달리 틱간 계단을 실제로 막아, 변화율 제한된 와이어가 예산을
   * 노이즈에 쓰지 않는다. */
  const float max_curvature_rate = kMaxLateralJerk / (speed * speed);
  float curvature = clamp_float(
      new_curvature,
      prev_curvature - max_curvature_rate * kCurvatureRateWindowS,
      prev_curvature + max_curvature_rate * kCurvatureRateWindowS);
  const float roll_lat_accel = roll_rad * 9.81f;
  const float rate_limited = curvature;
  curvature = clamp_float(curvature, (-kMaxLateralAccel + roll_lat_accel) / (speed * speed),
                          (kMaxLateralAccel + roll_lat_accel) / (speed * speed));
  curvature = clamp_float(curvature, -kMaxCurvature, kMaxCurvature);
  if (limited) *limited = curvature != rate_limited;
  return curvature;
}

float lag_adjusted_desired_curvature(const LateralTarget &target, float speed_mps,
                                     float plan_age_s, float steer_actuator_delay_s,
                                     float prev_curvature, float roll_rad) {
  if (!target.valid) return 0.0f;
  return clip_curvature(speed_mps, prev_curvature,
                        lag_adjusted_curvature(target, speed_mps, plan_age_s,
                                               steer_actuator_delay_s),
                        roll_rad);
}

// 최종 송신 frame 묶음을 만든다.
std::vector<CanFrame> LateralController::build_frames(
    const VehicleCanState &vehicle_state,
    const LateralControlResult &result,
    int frame) {
  HyundaiLkasCommand command;
  command.apply_steer = result.apply_torque;
  command.steer_req = (result.active || steer_availability_hold_) && !result.large_angle_hold;
  command.cut_steer_temp = result.cut_steer_temp;
  /* 클러스터 LKAS 표시(양쪽 차선 보임): 3 작동, 4 대기. 클러스터는 sys_state 천이마다 부저를
   * 울리므로 active(정차 대기 등 가용성)가 아니라 engaged를 따라 enable/disable에서만 울린다.
   * steer_req는 별도 비트로 매 프레임 정확히 나간다. */
  command.sys_state = result.engaged ? 3 : 4;
  command.sys_warning = false;
  command.left_lane = result.left_lane;
  command.right_lane = result.right_lane;
  command.lkas_msg_count = next_lkas11_counter(vehicle_state);
  command.ldws_fix = false;  // 이 K7은 LDWS 전용이지만 LdwsOpt_USM 3은 효과가 없었다(2026-09-24)

  // should_send면 세 seed가 다 있다(seed_frames_ready).
  return build_lateral_can_frames(decode_lkas11(vehicle_state.lkas11_seed), decode_clu11(vehicle_state.clu11_seed),
                                  vehicle_state.mdps12_seed, command,
                                  config_.driving_params.mdps_speed_spoof_kph,
                                  result.active || steer_availability_hold_, vehicle_state.speed_unit_mph, frame);
}

int LateralController::next_lkas11_counter(const VehicleCanState &vehicle_state) {
  if (!lkas11_counter_valid_) {
    const HyundaiLkas11Values seed = decode_lkas11(vehicle_state.lkas11_seed);
    lkas11_counter_ = (seed.msg_count + 1) & 0xf;
    lkas11_counter_valid_ = true;
  }
  const int counter = lkas11_counter_ & 0xf;
  lkas11_counter_ = (lkas11_counter_ + 1) & 0xf;
  return counter;
}
