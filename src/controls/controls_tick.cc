#include "controls/controls_tick.h"

#include "localization/lateral_lag.h"
#include "learners/localizer_inputs.h"
#include "common/model_output.h"
#include "common/utils_file.h"
#include "common/utils_time.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

constexpr uint64_t kAlertModelTimeoutNs = 500000000ULL;
constexpr uint64_t kMaxCanRxAgeNs = 100000000ULL;
constexpr int kParamPollIntervalMs = 100;

void apply_can_batch(const CanBatch &batch, double now_s,
                     VehicleCanState *vehicle) {
  if (!batch.valid) return;
  const uint32_t count = std::min<uint32_t>(batch.count, kCanBatchMaxFrames);
  for (uint32_t i = 0; i < count; ++i) {
    const IpcCanFrame &frame = batch.frames[i];
    if (frame.flags != 0 || frame.data_len > 8 || frame.src > 7) continue;
    std::array<uint8_t, 8> data = {};
    std::copy_n(frame.data, frame.data_len, data.begin());
    update_vehicle_can_state(vehicle, frame.address, data,
                             static_cast<uint8_t>(frame.data_len),
                             static_cast<uint8_t>(frame.src), now_s);
  }
}

float vehicle_speed_mps(const VehicleCanState &vehicle, double now_s,
                        double timeout_s) {
  return std::max(0.0f, vehicle_speed_kph(vehicle, now_s, timeout_s) / 3.6f);
}

/* 모델 lead 출력을 알림/크루즈 입력으로 환산한다. signal_valid는 값이 유효한지,
 * valid는 확률 문턱까지 넘었는지. 거리는 레이더 기준점으로 옮긴다. */
struct VisionLead {
  bool model_fresh = false;
  bool signal_valid = false;
  bool valid = false;
  float distance_m = 0.0f;
  float relative_speed_mps = 0.0f;
};

VisionLead observe_vision_lead(const ModelState &model, uint64_t now_ns,
                               float ego_speed_mps) {
  VisionLead lead;
  lead.model_fresh = model.valid != 0 &&
                     timestamp_fresh_ns(model.model_timestamp_ns, now_ns, kAlertModelTimeoutNs);
  lead.signal_valid =
      lead.model_fresh && model.lead.valid != 0 &&
      std::isfinite(model.lead.x) && std::isfinite(model.lead.velocity);
  lead.valid = lead.signal_valid && model.lead.probability >= kLeadProbabilityThreshold;
  lead.distance_m = lead.signal_valid ? model.lead.x - kRadarToCameraDistanceM : 0.0f;
  lead.relative_speed_mps = lead.signal_valid ? model.lead.velocity - ego_speed_mps : 0.0f;
  return lead;
}

DepartureAlertInput make_alert_input(double now_s, const VehicleCanState &vehicle,
                                     const LateralControlResult &result,
                                     const ModelState &model, bool model_updated,
                                     const VisionLead &lead, float ego_speed_mps) {
  DepartureAlertInput input;
  input.now_s = now_s;
  input.vehicle_valid = result.vehicle_fresh;
  input.gear = vehicle.gear;
  input.speed_mps = ego_speed_mps;
  input.gas_pressed = vehicle.gas_pressed;
  input.lead_updated = model_updated;
  input.lead_valid = lead.valid;
  input.lead_distance_m = lead.valid ? lead.distance_m : 0.0f;
  input.lead_relative_speed_mps = lead.valid ? lead.relative_speed_mps : 0.0f;
  input.model_updated = model_updated;
  input.model_valid = lead.model_fresh;
  input.plan_distance_m = lead.model_fresh ? model.plan[kTrajectorySize - 1].x : 0.0f;
  input.gas_press_prob = lead.model_fresh ? model.gas_press_probs[1] : 0.0f;  // 2초 뒤
  input.turn_signal_on = vehicle.left_blinker || vehicle.right_blinker;
  return input;
}

AdaptiveCruiseInput make_adaptive_input(double now_s, bool enabled,
                                        const VehicleCanState &vehicle,
                                        const LateralControlResult &result,
                                        const PandaGateOutput &panda,
                                        const ModelState &model, bool model_updated,
                                        const VisionLead &lead, float ego_speed_kph) {
  AdaptiveCruiseInput input;
  input.now_s = now_s;
  input.enabled = enabled;
  input.controls_ready = result.active && panda.ready && panda.controls_allowed &&
                         vehicle.has_clu11_seed;
  input.cruise_active = vehicle.cruise_active;
  input.brake_pressed = vehicle.brake_pressed;
  input.gas_pressed = vehicle.gas_pressed;
  input.driver_accelerator_override = vehicle.driver_override != 0;
  input.speed_unit_mph = vehicle.speed_unit_mph;
  input.driver_button = vehicle.clu_button;
  input.driver_main_button = vehicle.clu_main_button;
  input.ego_speed_kph = ego_speed_kph;
  input.cluster_speed_kph = result.cluster_speed_kph;
  input.driver_set_speed_kph = cruise_set_speed_kph(vehicle);
  input.vision_lead_updated = model_updated;
  input.vision_lead_valid = lead.signal_valid;
  input.vision_lead_probability = model.lead.probability;
  input.vision_lead_distance_m = lead.distance_m;
  input.vision_lead_relative_speed_mps = lead.relative_speed_mps;
  return input;
}

// overlayd/recordd가 읽는 100 Hz 스냅샷. 필드 순서는 ipc_messages.h가 고정한다. timestamp_ns는 main이 찍는다.
ControlState make_control_state(const LateralControllerConfig &config,
                                const LateralControlResult &result,
                                const LateralTarget &target,
                                const VehicleCanState &vehicle,
                                const AdaptiveCruiseOutput &adaptive_cruise,
                                const DepartureAlertOutput &departure_alert,
                                const EngageEvents &events, bool radar_lead_fresh,
                                float ego_speed_kph, double now_s) {
  ControlState state;
  state.enabled = config.steering_params.enabled ? 1U : 0U;
  state.engaged = result.engaged ? 1U : 0U;
  state.active = result.active ? 1U : 0U;
  state.should_send = result.should_send ? 1U : 0U;
  state.path_usable = result.path_usable ? 1U : 0U;
  state.hud_flags =
      (target.laneless_mode ? kHudFlagLaneless : 0U) |
      (result.vehicle_fresh && vehicle.brake_hold ? kHudFlagBrakeHold : 0U) |
      (result.soft_disabling ? kHudFlagSoftDisabling : 0U) |
      (result.steer_saturated ? kHudFlagSteerSaturated : 0U) |
      (target.lane_change_state == 1 ? kHudFlagLaneChangePending : 0U) |
      (target.lane_change_state >= 2 ? kHudFlagLaneChanging : 0U) |
      (target.lane_change_direction > 0 ? kHudFlagLaneChangeRight : 0U) |
      (result.large_angle_hold ? kHudFlagSteerPaused : 0U) |
      (result.large_angle_hold_by_driver ? kHudFlagSteerPausedByDriver : 0U) |
      (target.turn_desire == 1 ? kHudFlagTurnLeft : 0U) |
      (target.turn_desire == 2 ? kHudFlagTurnRight : 0U) |
      (result.vehicle_fresh && brake_lights_on(vehicle, now_s) ? kHudFlagBrakeLights : 0U);
  state.seeds_ready = result.seeds_ready ? 1U : 0U;
  state.vehicle_fresh = result.vehicle_fresh ? 1U : 0U;
  // 결합 중 MDPS 일시 고장(상류도 결합 중에만 경고한다)
  state.steering_fault = result.engaged && result.steer_fault ? 1U : 0U;
  state.left_blinker = vehicle.left_blinker ? 1U : 0U;
  state.right_blinker = vehicle.right_blinker ? 1U : 0U;
  state.cruise_active = vehicle.cruise_active ? 1U : 0U;
  state.gear = vehicle.gear;
  state.cluster_speed_kph = result.cluster_speed_kph;
  const float driver_set_speed_kph = cruise_set_speed_kph(vehicle);
  state.cruise_max_speed_kph = adaptive_cruise.session_valid
      ? adaptive_cruise.maximum_speed_kph : driver_set_speed_kph;
  state.cruise_command_speed_kph = adaptive_cruise.session_valid
      ? adaptive_cruise.commanded_speed_kph : driver_set_speed_kph;
  state.steering_angle_deg = vehicle.steering_angle_deg;
  state.desired_curvature = result.desired_curvature;
  state.actual_curvature = result.actual_curvature;
  state.normalized_output = result.normalized_output;
  state.desired_torque = result.desired_torque;
  state.apply_torque = result.apply_torque;
  state.driver_torque = vehicle.driver_torque;
  state.desire = static_cast<uint32_t>(target.desire);
  std::snprintf(state.active_block, sizeof(state.active_block), "%s",
                block_reason_name(result.active_block));
  state.radar_lead_valid = radar_lead_fresh && vehicle.radar_lead_valid ? 1U : 0U;
  state.radar_lead_distance_m = vehicle.radar_lead_distance_m;
  state.radar_lead_relative_speed_mps = vehicle.radar_lead_relative_speed_mps;
  state.departure_alert_type = static_cast<uint32_t>(departure_alert.type);
  state.departure_alert_event_id = departure_alert.event_id;
  state.green_light_alert_armed = departure_alert.green_light_armed ? 1U : 0U;
  state.tpms_valid = tpms_state_fresh(vehicle, now_s) ? 1U : 0U;
  state.tpms_unit = static_cast<uint32_t>(vehicle.tpms_unit);
  state.tpms_pressure_fl = vehicle.tpms_pressure_fl;
  state.tpms_pressure_fr = vehicle.tpms_pressure_fr;
  state.tpms_pressure_rl = vehicle.tpms_pressure_rl;
  state.tpms_pressure_rr = vehicle.tpms_pressure_rr;
  state.tpms_warning = vehicle.tpms_warning ? 1U : 0U;
  state.engage_event_id = events.engage_id;
  state.disengage_event_id = events.disengage_id;
  state.engage_reject_event_id = events.reject_id;
  std::memcpy(state.engage_reject_block, events.reject_block,
              sizeof(state.engage_reject_block));
  state.ego_speed_kph = ego_speed_kph;
  return state;
}

}  // namespace

bool load_control_params(const ControlParamPaths &paths, ControlParams *params, std::string *error) {
  ControlParams candidate = *params;
  std::string load_error;
  if (!load_steering_params_json(paths.steering, &candidate.steering, &load_error)) {
    if (error) *error = "steering " + paths.steering + ": " + load_error;
    return false;
  }
  if (!load_driving_params_json(paths.driving, &candidate.driving, &load_error)) {
    if (error) *error = "driving " + paths.driving + ": " + load_error;
    return false;
  }
  if (!load_adaptive_cruise_params_json(paths.cruise, &candidate.cruise, &load_error)) {
    if (error) *error = "adaptive cruise " + paths.cruise + ": " + load_error;
    return false;
  }
  *params = candidate;
  return true;
}

std::string control_params_summary(const ControlParams &params) {
  char text[160];
  std::snprintf(text, sizeof(text), "mdpsSpoof=%.1fkph adaptiveCruise=%u gap=%.1fm/%.1fs decel=%.1fkph/s",
                params.driving.mdps_speed_spoof_kph, params.cruise.enabled ? 1U : 0U,
                params.cruise.standstill_gap_m, params.cruise.following_time_s,
                params.cruise.deceleration_rate_kph_per_s);
  return text;
}

ControlParamsWatcher::ControlParamsWatcher(ControlParamPaths paths, std::chrono::steady_clock::time_point now)
    : paths_(std::move(paths)), next_check_(now + std::chrono::milliseconds(kParamPollIntervalMs)) {
  stamp();
}

std::optional<ControlParams> ControlParamsWatcher::poll(std::chrono::steady_clock::time_point now,
                                                        bool reload_requested, const ControlParams &current) {
  if (!reload_requested && now < next_check_) return std::nullopt;
  next_check_ = now + std::chrono::milliseconds(kParamPollIntervalMs);
  const FileStamp steering = file_stamp(paths_.steering);
  const FileStamp driving = file_stamp(paths_.driving);
  const FileStamp cruise = file_stamp(paths_.cruise);
  const bool changed = steering != steering_stamp_ || driving != driving_stamp_ || cruise != cruise_stamp_;
  if (!reload_requested && !changed) return std::nullopt;
  steering_stamp_ = steering;
  driving_stamp_ = driving;
  cruise_stamp_ = cruise;
  ControlParams candidate = current;
  std::string error;
  if (!load_control_params(paths_, &candidate, &error)) {
    std::fprintf(stderr, "controlsd: params reload rejected: %s\n", error.c_str());
    return std::nullopt;
  }
  ++generation_;
  std::fprintf(stderr, "controlsd: params reloaded generation=%u %s\n", generation_,
               control_params_summary(candidate).c_str());
  return candidate;
}

void ControlParamsWatcher::stamp() {
  steering_stamp_ = file_stamp(paths_.steering);
  driving_stamp_ = file_stamp(paths_.driving);
  cruise_stamp_ = file_stamp(paths_.cruise);
}

LateralPlannerWorker::LateralPlannerWorker(const SteeringParams &params,
                                           const DrivingParams &driving)
    : planner_(params, driving), pending_steering_(params),
      pending_driving_(driving),
      thread_(&LateralPlannerWorker::run, this) {}

LateralPlannerWorker::~LateralPlannerWorker() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  condition_.notify_one();
  thread_.join();
}

void LateralPlannerWorker::submit(const ModelState &model, const VehicleCanState &vehicle,
                                  float v_ego, float measured_curvature, bool active) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    request_.model = model;
    request_.vehicle = vehicle;
    request_.v_ego = v_ego;
    request_.measured_curvature = measured_curvature;
    request_.active = active;
    pending_ = true;
  }
  condition_.notify_one();
}

void LateralPlannerWorker::update_params(const SteeringParams &params,
                                         const DrivingParams &driving) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_steering_ = params;
    pending_driving_ = driving;
    params_pending_ = true;
  }
  condition_.notify_one();
}

LateralTarget LateralPlannerWorker::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

void LateralPlannerWorker::run() {
  while (true) {
    Request request;
    SteeringParams steering;
    DrivingParams driving;
    bool has_request = false;
    bool apply_params = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] {
        return stop_ || pending_ || params_pending_;
      });
      if (stop_) return;
      if (params_pending_) {
        steering = pending_steering_;
        driving = pending_driving_;
        params_pending_ = false;
        apply_params = true;
      }
      if (pending_) {
        request = request_;
        pending_ = false;
        has_request = true;
      }
    }
    if (apply_params) planner_.update_params(steering, driving);
    if (!has_request) continue;
    const LateralTarget result = planner_.update(
        request.model, request.vehicle, request.v_ego,
        request.measured_curvature, request.active);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = result;
    }
  }
}

void EngageEvents::update(const LateralControlResult &result, const VehicleCanState &vehicle,
                          const PandaGateOutput &panda, const PandaState &panda_state,
                          const PathHoldOutput &held, const ModelState &model,
                          uint64_t now_ns) {
  if (result.engage_rejected) {
    if (++reject_id == 0) reject_id = 1;
    std::snprintf(reject_block, sizeof(reject_block), "%s",
                  block_reason_name(result.active_block));
    std::fprintf(stderr,
                 "controlsd: engage rejected block=%s event=%u\n",
                 reject_block, reject_id);
  } else if (have_previous && result.engaged != previous_engaged) {
    if (result.engaged) {
      if (++engage_id == 0) engage_id = 1;
    } else {
      if (++disengage_id == 0) disengage_id = 1;
    }
    std::fprintf(stderr,
                 "controlsd: engaged transition %u->%u "
                 "active=%u block=%s button=%d gear=%d "
                 "panda=%u/%u\n",
                 previous_engaged ? 1U : 0U, result.engaged ? 1U : 0U,
                 result.active ? 1U : 0U,
                 block_reason_name(result.active_block), vehicle.clu_button,
                 vehicle.gear, panda.ready ? 1U : 0U,
                 panda.controls_allowed ? 1U : 0U);
  }
  if (have_previous && result.active != previous_active) {
    std::fprintf(stderr,
                 "controlsd: active transition %u->%u "
                 "engaged=%u block=%s raw=%s rawPoints=%d rawReachM=%.1f "
                 "pathPoints=%d hold=%u modelAgeMs=%llu panda=%u/%u "
                 "state=%u/%u/%u/%u safety=%u:%u hb=%u fresh=%u\n",
                 previous_active ? 1U : 0U, result.active ? 1U : 0U,
                 result.engaged ? 1U : 0U,
                 block_reason_name(result.active_block),
                 held.raw.invalid_reason.empty() ? "none" :
                     held.raw.invalid_reason.c_str(),
                 held.raw.point_count, static_cast<double>(held.raw.reach_m),
                 held.path.point_count,
                 held.hold_applied ? 1U : 0U,
                 model.model_timestamp_ns != 0 && now_ns >= model.model_timestamp_ns
                     ? static_cast<unsigned long long>(
                           (now_ns - model.model_timestamp_ns) / 1000000ULL)
                     : 0ULL,
                 panda.ready ? 1U : 0U,
                 panda.controls_allowed ? 1U : 0U,
                 panda_state.connected, panda_state.comms_healthy,
                 panda_state.tx_enabled, panda_state.controls_allowed,
                 panda_state.safety_mode, panda_state.safety_param,
                 panda_state.heartbeat_lost, panda.state_fresh ? 1U : 0U);
  }
  previous_engaged = result.engaged;
  previous_active = result.active;
  have_previous = true;
}

namespace {

// 학습기·컨트롤러 상태를 HUD와 웹 콘솔이 읽는 LearnerState로(timestamp_ns 제외).
LearnerState make_learner_state(const LateralLearners &learners,
                                const SteeringParams &params, float road_bank_lat_accel,
                                bool live_delay_in_use, float plan_delay_s) {
  const VehicleParams &v = learners.vehicle_params();
  const TorqueParams &t = learners.torque_params();
  const LiveLateralParams live = learners.live();
  LearnerState state;
  state.flags = (v.inputs_ok ? kLearnerVehicleInputsOk : 0U) |
                (v.valid ? kLearnerVehicleValid : 0U) |
                (v.sensor_valid ? kLearnerSensorValid : 0U) |
                (v.steer_ratio_valid ? kLearnerSteerRatioValid : 0U) |
                (v.stiffness_factor_valid ? kLearnerStiffnessValid : 0U) |
                (v.angle_offset_average_valid ? kLearnerOffsetAverageValid : 0U) |
                (v.angle_offset_valid ? kLearnerOffsetValid : 0U) |
                (t.inputs_ok ? kLearnerTorqueInputsOk : 0U) |
                (t.valid ? kLearnerTorqueValid : 0U) |
                (live.use_vehicle && params.use_live_vehicle_params ? kLearnerUseVehicle : 0U) |
                (live.use_torque && params.use_live_torque_params ? kLearnerUseTorque : 0U) |
                (learners.vehicle_restored() ? kLearnerVehicleRestored : 0U) |
                (learners.torque_restore_status() == TorqueRestore::Restored
                     ? kLearnerTorqueRestored : 0U) |
                (live_delay_in_use ? kLearnerUseDelay : 0U) |
                (learners.localizer_inputs() ? kLearnerLocalizerInputs : 0U);
  state.steer_ratio = static_cast<float>(v.steer_ratio);
  state.stiffness_factor = static_cast<float>(v.stiffness_factor);
  state.roll_rad = static_cast<float>(v.roll_rad);
  state.angle_offset_average_deg = static_cast<float>(v.angle_offset_average_deg);
  state.angle_offset_deg = static_cast<float>(v.angle_offset_deg);
  state.steer_ratio_std = static_cast<float>(v.steer_ratio_std);
  state.stiffness_factor_std = static_cast<float>(v.stiffness_factor_std);
  state.angle_offset_average_std = static_cast<float>(v.angle_offset_average_std);
  state.angle_offset_fast_std = static_cast<float>(v.angle_offset_fast_std);
  state.yaw_bias_rad_s = static_cast<float>(learners.yaw_bias_rad_s());
  state.lat_accel_factor_raw = static_cast<float>(t.lat_accel_factor_raw);
  state.lat_accel_offset_raw = static_cast<float>(t.lat_accel_offset_raw);
  state.friction_raw = static_cast<float>(t.friction_raw);
  state.lat_accel_factor = static_cast<float>(t.lat_accel_factor);
  state.lat_accel_offset = static_cast<float>(t.lat_accel_offset);
  state.friction = static_cast<float>(t.friction);
  state.decay = static_cast<float>(t.decay);
  state.max_resets = static_cast<float>(t.max_resets);
  state.total_bucket_points = t.total_bucket_points;
  state.cal_perc = t.cal_perc;
  state.road_bank_lat_accel = road_bank_lat_accel;
  state.prior_steer_ratio = static_cast<float>(learners.prior_steer_ratio());
  state.prior_lat_accel_factor = static_cast<float>(learners.torque_estimator().tuning().lat_accel_factor);
  state.prior_friction = static_cast<float>(learners.torque_estimator().tuning().friction);
  for (int i = 0; i < TorqueEstimator::kBuckets; ++i)
    state.bucket_points[i] = static_cast<int16_t>(learners.torque_estimator().bucket_size(i));
  state.plan_delay_s = plan_delay_s;
  return state;
}

LateralControllerConfig controller_config(const ControlParams &params, bool force_engaged) {
  LateralControllerConfig config;
  config.force_engaged = force_engaged;
  config.steering_params = params.steering;
  config.driving_params = params.driving;
  return config;
}

}  // namespace

ControlsTick::ControlsTick(const ControlParams &params, bool force_engaged, PlannerPort &planner,
                           const std::string &vehicle_learn_json, const std::string &torque_learn_cache,
                           uint64_t seed)
    : config_(controller_config(params, force_engaged)),
      cruise_(params.cruise),
      planner_(planner),
      controller_(config_),
      learners_(params.steering, vehicle_learn_json, torque_learn_cache, seed),
      adaptive_cruise_controller_(params.cruise) {}

void ControlsTick::apply_params(const ControlParams &params) {
  cruise_ = params.cruise;
  config_.steering_params = params.steering;
  config_.driving_params = params.driving;
  controller_.update_params(config_.steering_params, config_.driving_params);
  planner_.update_params(config_.steering_params, config_.driving_params);
  adaptive_cruise_controller_.update_config(cruise_);
}

bool ControlsTick::on_can_batch(const CanBatch &batch, uint64_t can_now_ns, double now_s) {
  if (!can_batch_is_fresh(batch, can_now_ns, kMaxCanRxAgeNs)) return false;
  apply_can_batch(batch, now_s, &vehicle_);
  return true;
}

void ControlsTick::on_model(const ModelState &model, double now_s) {
  model_ = model;
  model_updated_ = true;
  planner_.submit(
      model_, vehicle_, vehicle_speed_mps(
          vehicle_, now_s,
          static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) / 1000.0),
      last_result_.actual_curvature, last_result_.active);
}

void ControlsTick::on_localization(const LocalizationRead &localization, uint64_t can_now_ns, double now_s) {
  /* lagd(locationd) 추정 지연. 확정(5블록)이고 2초 안의 값만 쓴다. */
  if (!localization.open) {
    learners_.set_localizer(false, LocalizerSample{});
    return;
  }
  const LocalizationState &state = localization.state;
  const uint64_t age_ns = localization.read_ns - state.timestamp_ns;
  const bool fresh = localization.read && age_ns < 2'000'000'000ULL;
  controller_.set_live_delay(state.lateral_delay_s,
                             fresh && state.lag_status ==
                                 static_cast<uint32_t>(LateralLagStatus::Estimated));
  /* 상류 paramsd·torqued 입력(localizer_sample_from). 표본 시각은 학습기 시계(now_s)로
   * 옮긴다. 읽기가 쓰기와 겹쳐 실패한 틱은 학습기 입력을 직전 값으로 두고, lagd 지연은 그 틱만
   * 쓰지 않는다(위 set_live_delay가 무효로 받는다). */
  if (localization.read) {
    const double age_s =
        static_cast<double>(static_cast<int64_t>(can_now_ns - state.timestamp_ns)) * 1e-9;
    LocalizerSample sample;
    const bool use = localizer_sample_from(
        state, config_.steering_params.use_locationd_learner_inputs, age_s, now_s - age_s, &sample);
    learners_.set_localizer(use, sample);
  }
}

ControlState ControlsTick::step(double now_s, uint64_t now_ns) {
  lateral_target_ = planner_.latest();
  panda_ = panda_gate_.update(panda_state_, now_ns, config_.force_engaged);
  const uint64_t model_timeout_ns =
      static_cast<unsigned long long>(config_.driving_params.model_timeout_ms) *
      1000000ULL;
  const PathHoldOutput held = path_gate_.update(model_, now_ns, model_timeout_ns);
  const int frame = control_frame_++;
  last_result_ = controller_.update(held.path, lateral_target_, vehicle_, now_s,
                                    frame, panda_.ready, panda_.controls_allowed);
  events_.update(last_result_, vehicle_, panda_, panda_state_, held, model_, now_ns);

  const bool radar_lead_fresh = signal_time_fresh(vehicle_.scc11_time_s, now_s, 0.5);
  /* 크루즈·이탈 경보·발행 속도는 휠 속도를 0.5 s까지만 믿는다. 조향(control_speed_kph)은 설정한
   * vehicle_state_timeout_ms를 쓰므로 그 값을 늘려도 크루즈 버튼이 낡은 속도로 나가지 않는다. */
  const float ego_speed_kph = vehicle_speed_kph(vehicle_, now_s);
  const float ego_speed_mps = ego_speed_kph / 3.6f;
  const VisionLead lead = observe_vision_lead(model_, now_ns, ego_speed_mps);
  alert_input_ = make_alert_input(
      now_s, vehicle_, last_result_, model_, model_updated_, lead, ego_speed_mps);
  adaptive_cruise_ = adaptive_cruise_controller_.update(make_adaptive_input(
      now_s, cruise_.enabled, vehicle_, last_result_, panda_, model_,
      model_updated_, lead, ego_speed_kph));

  if (adaptive_cruise_.command_button != 0) {
    last_result_.frames.push_back(
        create_cruise_button_frame(decode_clu11(vehicle_.clu11_seed), adaptive_cruise_.command_button, frame));
    last_result_.should_send = true;
  }

  const DepartureAlertOutput departure_alert =
      departure_alert_detector_.update(alert_input_);
  if (departure_alert.event_id != 0 &&
      departure_alert.event_id != last_logged_alert_event_id_) {
    last_logged_alert_event_id_ = departure_alert.event_id;
    std::fprintf(
        stderr,
        "controlsd: departure alert=%s event=%u "
        "visionLead=%.1fm rel=%.1fm/s p=%.2f plan=%.1fm gas2=%.2f\n",
        departure_alert_name(departure_alert.type),
        departure_alert.event_id,
        alert_input_.lead_distance_m,
        alert_input_.lead_relative_speed_mps,
        model_.lead.probability,
        alert_input_.plan_distance_m,
        alert_input_.gas_press_prob);
  }
  model_updated_ = false;
  return make_control_state(config_, last_result_, lateral_target_, vehicle_, adaptive_cruise_,
                            departure_alert, events_, radar_lead_fresh, ego_speed_kph, now_s);
}

LearnerOutputs ControlsTick::update_learners(double now_s) {
  /* 학습기는 이번 틱에 실제로 보낸 토크로 갱신하고, 컨트롤러는 다음 틱에 쓴다. torqued 지연은
   * 컨트롤러와 같은 조향 지연(상류는 lateralDelay). */
  learners_.set_lateral_delay(controller_.plan_delay_s());
  learners_.update(vehicle_, now_s,
                   static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) / 1000.0,
                   last_result_.active, last_result_.apply_torque, last_result_.steering_pressed);
  controller_.set_live_params(learners_.live(), learners_.vehicle_valid(),
                              model_.calibration.status == 1U);
  controller_.set_calibration_status(model_.calibration.status);
  LearnerOutputs out;
  if (learners_.vehicle_persist_due()) {
    out.vehicle_json = learners_.vehicle_persist_json();
    const VehicleParams &v = learners_.vehicle_params();
    const TorqueParams &t = learners_.torque_params();
    std::fprintf(stderr,
                 "controlsd: learners sr=%.2f stiffness=%.2f offset=%.2f/%.2f "
                 "roll=%.2f valid=%d | torque points=%d cal=%d%% factor=%.2f/%.2f "
                 "offset=%.3f friction=%.3f/%.3f valid=%d\n",
                 v.steer_ratio, v.stiffness_factor, v.angle_offset_average_deg,
                 v.angle_offset_deg, v.roll_rad * 57.29577951308232, v.valid ? 1 : 0,
                 t.total_bucket_points, t.cal_perc, t.lat_accel_factor_raw,
                 t.lat_accel_factor, t.lat_accel_offset, t.friction_raw, t.friction,
                 t.valid ? 1 : 0);
  }
  if (learners_.torque_persist_due()) out.torque_cache = learners_.torque_cache();
  if (learners_.vehicle_published()) {
    out.learner_state =
        make_learner_state(learners_, config_.steering_params, controller_.road_bank_lat_accel(),
                           controller_.live_delay_in_use(), controller_.plan_delay_s());
  }
  return out;
}
