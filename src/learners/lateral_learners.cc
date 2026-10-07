#include "learners/lateral_learners.h"

#include <algorithm>
#include <cmath>

#include "common/utils_time.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
double rad(double deg) { return deg * kPi / 180.0; }

}  // namespace

// ---------------------------------------------------------------- YawBiasEstimator

double YawBiasEstimator::update(double t_s, double speed_mps, bool yaw_valid,
                                double yaw_rate_rad_s) {
  constexpr double kStandstillMps = 0.05;
  constexpr double kSettleS = 2.0;
  constexpr double kMaxStandstillYaw = 0.5 * kPi / 180.0;
  constexpr double kAlpha = 0.01 / (2.0 + 0.01);  // 100 Hz, 2초
  const bool standing = std::fabs(speed_mps) < kStandstillMps;
  if (standing && !standing_) standing_since_s_ = t_s;
  standing_ = standing;
  if (standing && t_s - standing_since_s_ >= kSettleS && yaw_valid &&
      std::isfinite(yaw_rate_rad_s) && std::fabs(yaw_rate_rad_s - bias_) < kMaxStandstillYaw)
    bias_ += kAlpha * (yaw_rate_rad_s - bias_);
  return yaw_rate_rad_s - bias_;
}

// ---------------------------------------------------------------- LateralLearners

namespace {

VehicleParamsInit restore_or_prior(const std::string &json, const VehicleModelConstants &c) {
  if (!json.empty()) return restore_vehicle_params(json, c);
  VehicleParamsInit init;
  init.steer_ratio = c.steer_ratio;
  return init;
}

TorqueTuning torque_tuning(const SteeringParams &params) {
  TorqueTuning t;  // CP 값은 Float32
  t.lat_accel_factor = params.torque_lat_accel_factor;
  t.friction = params.torque_friction;
  return t;
}

}  // namespace

VehicleModelConstants LateralLearners::constants(const SteeringParams &params) {
  VehicleModelConstants c;
  c.mass_kg = params.mass_kg;
  c.wheelbase_m = params.wheelbase_m;
  c.center_to_front_m = params.center_to_front_m();
  c.tire_stiffness_factor = params.tire_stiffness_factor;
  c.steer_ratio = params.steer_ratio;
  return c;
}

LateralLearners::LateralLearners(const SteeringParams &params, const std::string &vehicle_json,
                                 const std::string &torque_cache, uint64_t seed,
                                 const VehicleParamsOptions &options)
    : constants_(constants(params)),
      init_(restore_or_prior(vehicle_json, constants_)),
      vehicle_restored_(init_.restored),
      vehicle_restore_rejected_(!vehicle_json.empty() && !init_.restored),
      bias_(init_.yaw_bias_rad_s),
      vehicle_(constants_, init_.steer_ratio, init_.stiffness_factor, rad(init_.angle_offset_deg),
               options),
      torque_(torque_tuning(params), params.steer_actuator_delay, seed, torque_cache,
              params.use_locationd_learner_inputs),
      steer_max_(std::max(1, params.steer_max)) {}

void LateralLearners::set_localizer(bool use, const LocalizerSample &sample) {
  use_localizer_ = use;
  if (!use || !std::isfinite(sample.t_s) || !std::isfinite(sample.yaw_rate_rad_s) ||
      !std::isfinite(sample.roll_rad))
    return;
  if (!samples_.empty() && sample.t_s <= samples_.back().t_s) return;  // 같은 표본을 또 읽었다
  samples_.push_back(sample);
  while (samples_.size() > 32) samples_.pop_front();  // 3초면 지연(0.25초)에 넉넉하다
}

/* 표본 사이를 선형 보간한다. 표본 간격이 벌어졌거나(정상 0.1초) 앞뒤로 0.1초 넘게 벗어나면 없음. */
bool LateralLearners::localizer_at(double t_s, LocalizerSample *out) const {
  constexpr double kMaxHoldS = 0.1;
  constexpr double kMaxGapS = 0.25;
  if (samples_.empty()) return false;
  if (t_s <= samples_.front().t_s) {
    if (samples_.front().t_s - t_s > kMaxHoldS) return false;
    *out = samples_.front();
  } else if (t_s >= samples_.back().t_s) {
    if (t_s - samples_.back().t_s > kMaxHoldS) return false;
    *out = samples_.back();
  } else {
    size_t i = samples_.size() - 1;
    while (samples_[i - 1].t_s > t_s) --i;  // samples_[i-1].t_s <= t_s < samples_[i].t_s
    const LocalizerSample &a = samples_[i - 1], &b = samples_[i];
    if (b.t_s - a.t_s > kMaxGapS) return false;
    const double w = (t_s - a.t_s) / (b.t_s - a.t_s);
    auto mix = [w](double x, double y) { return x + w * (y - x); };
    out->yaw_rate_rad_s = mix(a.yaw_rate_rad_s, b.yaw_rate_rad_s);
    out->yaw_rate_std_rad_s = mix(a.yaw_rate_std_rad_s, b.yaw_rate_std_rad_s);
    out->roll_rad = mix(a.roll_rad, b.roll_rad);
    out->roll_std_rad = mix(a.roll_std_rad, b.roll_std_rad);
    out->pose_ok = a.pose_ok && b.pose_ok;
    out->roll_ok = a.roll_ok && b.roll_ok;
  }
  out->t_s = t_s;
  return true;
}

void LateralLearners::update(const VehicleCanState &vehicle, double now_s, double timeout_s,
                             bool lat_active, int apply_torque, bool steering_pressed) {
  const float speed_kph = vehicle_speed_kph(vehicle, now_s, timeout_s);
  const bool esp_fresh = signal_time_fresh(vehicle.esp12_time_s, now_s, timeout_s);
  Tick tick;
  VehicleParamsInput &in = tick.vin;
  in.t_s = now_s;
  in.inputs_fresh = vehicle_state_fresh(vehicle, now_s, timeout_s) && esp_fresh &&
                    std::isfinite(speed_kph);
  in.steering_angle_deg = vehicle.steering_angle_deg;
  in.speed_mps = std::isfinite(speed_kph) ? speed_kph / 3.6 : 0.0;
  in.gear = vehicle.gear;
  in.yaw_rate_valid = esp_fresh && vehicle.yaw_rate_valid;
  // 바이어스 추정은 locationd를 쓰는 동안에도 정차마다 따라간다(ESP12로 돌아갈 때를 위해)
  in.yaw_rate_rad_s = bias_.update(now_s, in.speed_mps, in.yaw_rate_valid, vehicle.yaw_rate_rad_s);
  in.lat_accel_valid = esp_fresh && vehicle.lat_accel_valid;
  in.lat_accel_mps2 = -vehicle.lat_accel_mps2;  // 반전 저장돼 있다

  /* torqued는 컨트롤러 관례(우측 양수)로 돌린다. 보낸 토크는 출력 부호를 되돌리고
   * ESP12 요레이트(좌측 양수)는 뒤집는다. 그래야 학습 절편이 torque_lat_accel_offset과
   * 같은 부호다. 롤은 feed에서 paramsd 갱신 뒤에 채운다. */
  TorqueEstimatorInput &tin = tick.tin;
  tin.t_s = now_s;
  tin.inputs_fresh = in.inputs_fresh;
  tin.lat_active = lat_active;
  tin.steer_torque = static_cast<double>(kTorqueOutputSign * apply_torque) / steer_max_;
  tin.speed_mps = in.speed_mps;
  tin.steer_override = steering_pressed;

  vehicle_published_ = torque_published_ = false;
  vehicle_persist_due_ = torque_persist_due_ = false;
  if (!use_localizer_ && pending_.empty()) {
    feed(tick);
    return;
  }
  pending_.push_back(tick);
  // ESP12로 돌아가면 쌓인 틱을 바로 다 넣는다(표본이 덮는 틱은 그대로 locationd 값)
  const double release_s = use_localizer_ ? now_s - kLocalizerDelayS : now_s;
  while (!pending_.empty() && pending_.front().vin.t_s <= release_s + 1e-6) {
    feed(pending_.front());
    pending_.pop_front();
  }
}

void LateralLearners::feed(Tick tick) {
  VehicleParamsInput &in = tick.vin;
  TorqueEstimatorInput &tin = tick.tin;
  LocalizerSample loc;
  const bool localizer = localizer_at(in.t_s, &loc) && loc.pose_ok;
  if (localizer) {
    in.yaw_rate_valid = true;  // 필터 유효(set_localizer의 use 조건)
    in.yaw_rate_rad_s = -loc.yaw_rate_rad_s;  // 좌측 양수(ESP12 관례)로
    in.yaw_rate_std_rad_s = loc.yaw_rate_std_rad_s;
    in.localizer_roll_given = true;
    in.localizer_roll_valid = loc.roll_ok;
    in.localizer_roll_rad = loc.roll_rad;
    in.localizer_roll_std_rad = loc.roll_std_rad;
  }
  last_vehicle_input_ = in;
  const bool vehicle_published = vehicle_.update(in);
  vehicle_published_ = vehicle_published_ || vehicle_published;
  vehicle_persist_due_ = vehicle_persist_due_ || vehicle_.persist_due();
  const VehicleParams &vp = vehicle_.params();
  if (vehicle_published) {
    live_.use_vehicle = true;
    live_.steer_ratio = static_cast<float>(vp.steer_ratio);
    live_.stiffness_factor = static_cast<float>(vp.stiffness_factor);
    live_.angle_offset_deg = static_cast<float>(vp.angle_offset_deg);
    live_.roll_rad = static_cast<float>(vp.roll_rad);
  }

  /* torqued 점은 시작할 때 정한 출처(캐시에 남는다)의 틱만 쓴다. 출처가 섞이면 점의 횡가속도가
   * 롤 출처 차이만큼 어긋난다. 상류 torqued는 자세 롤을 그대로 쓴다. */
  if (localizer) {
    tin.pose_valid = loc.roll_ok && torque_.localizer_source();
    tin.yaw_rate_rad_s = loc.yaw_rate_rad_s;
    tin.roll_rad = loc.roll_rad;
  } else {
    tin.pose_valid = in.yaw_rate_valid && !torque_.localizer_source();
    tin.yaw_rate_rad_s = -in.yaw_rate_rad_s;
    tin.roll_rad = vp.roll_rad;
  }
  last_torque_input_ = tin;
  const bool torque_published = torque_.update(tin);
  torque_published_ = torque_published_ || torque_published;
  torque_persist_due_ = torque_persist_due_ || torque_.persist_due();
  const TorqueParams &tp = torque_.params();
  if (torque_published && tp.inputs_ok && tp.use_params) {
    live_.use_torque = true;
    live_.lat_accel_factor = static_cast<float>(tp.lat_accel_factor);
    live_.lat_accel_offset = static_cast<float>(tp.lat_accel_offset);
    live_.friction = static_cast<float>(tp.friction);
  }
}

std::string LateralLearners::vehicle_persist_json() const {
  return persist_vehicle_params(vehicle_.params(), constants_, bias_.bias());
}
