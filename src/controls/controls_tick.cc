#include "controls/controls_tick.h"

#include "common/model_output.h"
#include "common/utils_time.h"
#include "learners/localizer_inputs.h"
#include "localization/lateral_lag.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

constexpr uint64_t kAlertModelTimeoutNs = 500'000'000ULL;
constexpr uint64_t kMaxCanRxAgeNs = 100'000'000ULL;
constexpr double kRadarLeadTimeoutS = 0.5;                   // SCC11 레이더 앞차

// 배치는 신선함(valid 포함)을 본 뒤에 들어오고, 프레임 수는 큐에서 꺼낼 때 이미 묶였다.
void apply_can_batch(const CanBatch &batch, double now_s, VehicleCanState *vehicle) {
    for (uint32_t i = 0; i < batch.count; ++i) {
        const IpcCanFrame &frame = batch.frames[i];
        if (!is_vehicle_rx_frame(frame.flags, frame.data_len, frame.src)) continue;
        // 8바이트를 통째로 옮긴다: data_len 뒤는 0이다(fill_can_batch가 프레임을 비우고 채운다)
        std::array<uint8_t, 8> data;
        std::copy_n(frame.data, data.size(), data.begin());
        update_vehicle_can_state(vehicle, frame.address, data, static_cast<uint8_t>(frame.data_len),
                                 static_cast<uint8_t>(frame.src), now_s);
    }
}

// 모델 lead 출력을 알림/크루즈 입력으로 환산한다. signal_valid는 값이 유효한지,
// valid는 확률 문턱까지 넘었는지. 거리는 레이더 기준점으로 옮긴다.
struct VisionLead {
    bool model_fresh = false;
    bool signal_valid = false;
    bool valid = false;
    float distance_m = 0.0f;
    float relative_speed_mps = 0.0f;
};

VisionLead observe_vision_lead(const ModelState &model, uint64_t now_ns, float ego_speed_mps) {
    VisionLead lead;
    lead.model_fresh = model.valid != 0 && timestamp_fresh_ns(model.model_timestamp_ns, now_ns, kAlertModelTimeoutNs);
    lead.signal_valid =
        lead.model_fresh && model.lead.valid != 0 && std::isfinite(model.lead.x) && std::isfinite(model.lead.velocity);
    lead.valid = lead.model_fresh && lead_usable(model.lead);
    lead.distance_m = lead.signal_valid ? model.lead.x - kRadarToCameraDistanceM : 0.0f;
    lead.relative_speed_mps = lead.signal_valid ? model.lead.velocity - ego_speed_mps : 0.0f;
    return lead;
}

// 비전 크루즈의 앞차. modeld의 앞차 추적(model/lead_tracker: 모델 거리와 광류 확대율을 섞은 값)이 있으면
// 거리와 상대 속도를 그것으로 바꾼다. 모델 거리는 프레임마다 0.6~1.6 m 흔들리고 상대 속도는 25 m 밖에서
// 실제의 절반쯤만 읽는데, 추적은 흔들림을 1/4 안팎으로 줄이고 접근 속도를 기울기 0.8로 읽는다(대신 0.35초
// 늦다, docs/lead-flow.md). 쓸지 말지(확률 문턱)는 모델 앞차 그대로 정한다. 출발 알림은 늦으면 안
// 되고 정차 중 모델 앞차가 이미 고르므로 observe_vision_lead를 그대로 쓴다.
VisionLead observe_cruise_lead(VisionLead lead, const LeadTrackState &track) {
    if (lead.signal_valid && track.valid != 0 && std::isfinite(track.distance_m) &&
        std::isfinite(track.relative_speed_mps)) {
        lead.distance_m = track.distance_m - kRadarToCameraDistanceM;
        lead.relative_speed_mps = track.relative_speed_mps;
    }
    return lead;
}

DepartureAlertInput make_alert_input(double now_s, const VehicleCanState &vehicle, const LateralControlOutput &result,
                                     const ModelState &model, bool model_updated, const VisionLead &lead,
                                     float ego_speed_mps) {
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

AdaptiveCruiseInput make_adaptive_input(double now_s, bool enabled, const VehicleCanState &vehicle,
                                        const LateralControlOutput &result, const PandaGateOutput &panda,
                                        bool model_updated, const VisionLead &lead, float ego_speed_kph) {
    AdaptiveCruiseInput input;
    input.now_s = now_s;
    input.enabled = enabled;
    input.controls_ready = result.active && panda.ready && panda.controls_allowed && vehicle.has_clu11_seed;
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
    input.vision_lead_valid = lead.valid;
    input.vision_lead_distance_m = lead.distance_m;
    input.vision_lead_relative_speed_mps = lead.relative_speed_mps;
    return input;
}

// 학습 파일 내용이 마지막으로 넘긴 것과 다르면 그것을 기억하고 out에 담는다. torqued 캐시는 첫 갱신 전에는 비어 있다.
void emit_if_changed(const std::string &content, std::string *saved, std::optional<std::string> *out) {
    if (content.empty() || content == *saved) return;
    *saved = content;
    *out = content;
}

}  // namespace

// 필드 순서는 ipc_messages.h가 고정한다. timestamp_ns는 main이 찍는다.
ControlState ControlsTick::make_control_state(const DepartureAlertOutput &departure_alert, bool radar_lead_fresh,
                                              float ego_speed_kph, double now_s) const {
    const LateralControlOutput &result = last_result_;
    const LateralTarget &target = lateral_target_;
    const VehicleCanState &vehicle = vehicle_;
    const EngageEvents &events = events_;
    ControlState state;
    state.enabled = steering().enabled ? 1U : 0U;
    state.engaged = result.engaged ? 1U : 0U;
    state.active = result.active ? 1U : 0U;
    state.should_send = result.should_send ? 1U : 0U;
    state.path_usable = result.path_usable ? 1U : 0U;
    state.hud_flags = (target.laneless_mode ? kHudFlagLaneless : 0U) |
                      (result.vehicle_fresh && vehicle.brake_hold ? kHudFlagBrakeHold : 0U) |
                      (result.soft_disabling ? kHudFlagSoftDisabling : 0U) |
                      (result.steer_saturated ? kHudFlagSteerSaturated : 0U) |
                      (target.lane_change_state == LaneChangeState::PreLaneChange ? kHudFlagLaneChangePending : 0U) |
                      (target.lane_change_state >= LaneChangeState::Starting ? kHudFlagLaneChanging : 0U) |
                      (target.lane_change_direction > 0 ? kHudFlagLaneChangeRight : 0U) |
                      (result.large_angle_hold ? kHudFlagSteerPaused : 0U) |
                      (result.large_angle_hold_by_driver ? kHudFlagSteerPausedByDriver : 0U) |
                      (result.vehicle_fresh && brake_lights_on(vehicle, now_s) ? kHudFlagBrakeLights : 0U) |
                      (result.nn_feedforward_used ? kHudFlagNnFeedforward : 0U) |
                      (result.steering_pressed ? kHudFlagSteeringPressed : 0U) |
                      (result.lat_saturated ? kHudFlagLatSaturated : 0U) | kHudFlagLagInputs;
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
    state.cruise_max_speed_kph =
        adaptive_cruise_.session_valid ? adaptive_cruise_.maximum_speed_kph : driver_set_speed_kph;
    state.cruise_command_speed_kph =
        adaptive_cruise_.session_valid ? adaptive_cruise_.commanded_speed_kph : driver_set_speed_kph;
    state.steering_angle_deg = vehicle.steering_angle_deg;
    state.desired_curvature = result.desired_curvature;
    state.actual_curvature = result.actual_curvature;
    state.normalized_output = result.normalized_output;
    state.desired_torque = result.desired_torque;
    state.apply_torque = result.apply_torque;
    state.driver_torque = vehicle.driver_torque;
    state.desire = static_cast<uint32_t>(target.desire);
    // 새 스냅샷(0으로 채워짐)이라 끝의 NUL은 그대로다. 매 틱 snprintf보다 싸다.
    std::strncpy(state.active_block, block_reason_name(result.active_block), sizeof(state.active_block) - 1);
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
    std::memcpy(state.engage_reject_block, events.reject_block, sizeof(state.engage_reject_block));
    state.ego_speed_kph = ego_speed_kph;
    return state;
}

// 결합·해제·거부 이벤트 id를 올리고 결합·활성 전이를 로그에 남긴다.
void ControlsTick::update_events(const PathHoldOutput &held, uint64_t now_ns) {
    const LateralControlOutput &result = last_result_;
    const VehicleCanState &vehicle = vehicle_;
    const PandaGateOutput &panda = panda_;
    const PandaState &panda_state = panda_state_;
    const ModelState &model = model_;
    EngageEvents &e = events_;
    if (result.engage_rejected) {
        if (++e.reject_id == 0) e.reject_id = 1;
        // 녹화 바이트가 직전 이름의 꼬리를 남기지 않게 비우고 쓴다
        std::memset(e.reject_block, 0, sizeof(e.reject_block));
        std::snprintf(e.reject_block, sizeof(e.reject_block), "%s", block_reason_name(result.active_block));
        std::fprintf(stderr, "controlsd: engage rejected block=%s event=%u\n", e.reject_block, e.reject_id);
    } else if (e.have_previous && result.engaged != e.previous_engaged) {
        if (result.engaged) {
            if (++e.engage_id == 0) e.engage_id = 1;
        } else {
            if (++e.disengage_id == 0) e.disengage_id = 1;
        }
        std::fprintf(stderr,
                     "controlsd: engaged transition %u->%u "
                     "active=%u block=%s button=%d gear=%d "
                     "panda=%u/%u\n",
                     e.previous_engaged ? 1U : 0U, result.engaged ? 1U : 0U, result.active ? 1U : 0U,
                     block_reason_name(result.active_block), vehicle.clu_button, vehicle.gear, panda.ready ? 1U : 0U,
                     panda.controls_allowed ? 1U : 0U);
    }
    if (e.have_previous && result.active != e.previous_active) {
        std::fprintf(stderr,
                     "controlsd: active transition %u->%u "
                     "engaged=%u block=%s raw=%s rawPoints=%d rawReachM=%.1f "
                     "pathPoints=%d hold=%u modelAgeMs=%llu panda=%u/%u "
                     "state=%u/%u/%u/%u safety=%u:%u hb=%u fresh=%u\n",
                     e.previous_active ? 1U : 0U, result.active ? 1U : 0U, result.engaged ? 1U : 0U,
                     block_reason_name(result.active_block),
                     held.raw.usable_for_steering ? "none"
                     : held.raw.model_lost        ? "model_stale"
                                                  : "path_invalid",
                     held.raw.point_count, static_cast<double>(held.raw.reach_m), held.path.point_count,
                     held.hold_applied ? 1U : 0U,
                     model.model_timestamp_ns != 0 && now_ns >= model.model_timestamp_ns
                         ? static_cast<unsigned long long>((now_ns - model.model_timestamp_ns) / 1'000'000ULL)
                         : 0ULL,
                     panda.ready ? 1U : 0U, panda.controls_allowed ? 1U : 0U, panda_state.connected,
                     panda_state.comms_healthy, panda_state.tx_enabled, panda_state.controls_allowed,
                     panda_state.safety_mode, panda_state.safety_param, panda_state.heartbeat_lost,
                     panda.state_fresh ? 1U : 0U);
    }
    e.previous_engaged = result.engaged;
    e.previous_active = result.active;
    e.have_previous = true;
}

LearnerState ControlsTick::make_learner_state() const {
    const LateralLearners &learners = learners_;
    const SteeringParams &params = steering();
    const VehicleParams &v = learners.vehicle_params();
    const TorqueParams &t = learners.torque_params();
    const LiveLateralParams live = learners.live();
    LearnerState state;
    state.flags =
        (v.inputs_ok ? kLearnerVehicleInputsOk : 0U) | (v.valid ? kLearnerVehicleValid : 0U) |
        (v.sensor_valid ? kLearnerSensorValid : 0U) | (v.steer_ratio_valid ? kLearnerSteerRatioValid : 0U) |
        (v.stiffness_factor_valid ? kLearnerStiffnessValid : 0U) |
        (v.angle_offset_average_valid ? kLearnerOffsetAverageValid : 0U) |
        (v.angle_offset_valid ? kLearnerOffsetValid : 0U) | (t.inputs_ok ? kLearnerTorqueInputsOk : 0U) |
        (t.valid ? kLearnerTorqueValid : 0U) |
        (live.use_vehicle && params.use_live_vehicle_params ? kLearnerUseVehicle : 0U) |
        (live.use_torque && params.use_live_torque_params ? kLearnerUseTorque : 0U) |
        (learners.vehicle_restored() ? kLearnerVehicleRestored : 0U) |
        (learners.torque_restore_status() == TorqueRestore::Restored ? kLearnerTorqueRestored : 0U) |
        (learners.torque_restore_status() == TorqueRestore::SourceChanged ? kLearnerTorquePartlyRestored : 0U) |
        (controller_.live_delay_in_use() ? kLearnerUseDelay : 0U) |
        (learners.localizer_inputs() ? kLearnerLocalizerInputs : 0U) |
        (controller_.lateral_nn_loaded() ? kLearnerLateralNnLoaded : 0U);
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
    state.road_bank_lat_accel = controller_.road_bank_lat_accel();
    state.prior_steer_ratio = static_cast<float>(learners.prior_steer_ratio());
    state.prior_lat_accel_factor = static_cast<float>(learners.torque_estimator().tuning().lat_accel_factor);
    state.prior_friction = static_cast<float>(learners.torque_estimator().tuning().friction);
    for (int i = 0; i < TorqueEstimator::kBuckets; ++i)
        state.bucket_points[i] = static_cast<int16_t>(learners.torque_estimator().bucket_size(i));
    state.plan_delay_s = controller_.plan_delay_s();
    return state;
}

ControlsTick::ControlsTick(const ControlParams &params, bool force_engaged, const std::string &vehicle_learn_json,
                           const std::string &torque_learn_cache)
    : cruise_(params.cruise),
      planner_(params.steering),
      controller_(LateralControllerConfig{force_engaged, params.steering}),
      learners_(params.steering, vehicle_learn_json, torque_learn_cache),
      adaptive_cruise_controller_(params.cruise),
      saved_vehicle_data_(vehicle_learn_json),
      saved_torque_data_(torque_learn_cache) {}

void ControlsTick::update_params(const ControlParams &params) {
    cruise_ = params.cruise;
    controller_.update_params(params.steering);
    planner_.update_params(params.steering);
    adaptive_cruise_controller_.update_params(cruise_);
}

bool ControlsTick::on_can_batch(const CanBatch &batch, uint64_t now_ns, double now_s) {
    if (!can_batch_is_fresh(batch, now_ns, kMaxCanRxAgeNs)) return false;
    apply_can_batch(batch, now_s, &vehicle_);
    return true;
}

void ControlsTick::on_model(const ModelState &model) {
    model_ = model;
    model_updated_ = true;
    // 상류 플래너처럼 거른 vEgo를 준다(직전 틱 값). 휠 속도가 낡았으면 0이다. 결과는 이번 틱 update가 쓴다.
    LateralPlannerInput planner_in;
    planner_in.v_ego = std::max(0.0f, last_result_.control_speed_kph / 3.6f);
    planner_in.measured_curvature = last_result_.actual_curvature;
    planner_in.lat_active = last_result_.lat_active;
    planner_in.steering_pressed = last_result_.steering_pressed;
    lateral_target_ = planner_.update(model_, vehicle_, planner_in);
}

void ControlsTick::on_localization(const LocalizationRead &localization, double now_s) {
    // lagd(locationd) 추정 지연. 확정(5블록)인 값을 쓴다. 상류처럼 locationd가 멈춰도 마지막 값을 쓴다.
    if (!localization.open) {
        learners_.set_localizer(false, LocalizerSample{});
        return;
    }
    // 읽기가 쓰기와 겹쳐 실패한 틱은 상류 SubMaster처럼 마지막으로 온전히 읽은 상태를 쓴다. 그 틱에 lagd
    // 지연을 버리면 토크 요청 버퍼를 읽는 위치가 바뀌어 보낸 토크가 튄다.
    if (localization.read) {
        localization_ = localization.state;
        localization_seen_ = true;
    }
    if (!localization_seen_) return;
    const LocalizationState &state = localization_;
    controller_.set_live_delay(state.lateral_delay_s,
                               state.lag_status == static_cast<uint32_t>(LateralLagStatus::Estimated));
    // 상류 paramsd·torqued 입력(localizer_sample_from). 표본 시각은 학습기 시계(now_s)로
    // 옮긴다. 새로 읽은 틱에만 넘기고, 못 읽은 틱은 학습기가 직전 표본을 그대로 쓴다.
    if (localization.read) {
        const double age_s =
            static_cast<double>(static_cast<int64_t>(localization.read_ns - state.timestamp_ns)) * 1e-9;
        LocalizerSample sample;
        const bool use =
            localizer_sample_from(state, steering().use_locationd_learner_inputs, age_s, now_s - age_s, &sample);
        learners_.set_localizer(use, sample);
    }
}

ControlState ControlsTick::update(double now_s, uint64_t now_ns) {
    panda_ = panda_gate_.update(panda_state_, now_ns, controller_.config().force_engaged);
    const PathHoldOutput held = path_gate_.update(model_, now_ns, kModelTimeoutNs);
    const int frame = control_frame_++;
    last_result_ = controller_.update(held.path, lateral_target_, vehicle_, now_s, frame, panda_, now_ns);
    update_events(held, now_ns);

    const bool radar_lead_fresh = signal_time_fresh(vehicle_.scc11_time_s, now_s, kRadarLeadTimeoutS);
    // 크루즈·이탈 경보·발행 속도(modeld 보정, locationd도 읽는다)도 컨트롤러가 거른 vEgo를 쓴다. 휠 속도가
    // kVehicleStateTimeoutS(0.5 s)보다 낡으면 NaN이다.
    const float ego_speed_kph = last_result_.control_speed_kph;
    const float ego_speed_mps = ego_speed_kph / 3.6f;
    const VisionLead lead = observe_vision_lead(model_, now_ns, ego_speed_mps);
    alert_input_ = make_alert_input(now_s, vehicle_, last_result_, model_, model_updated_, lead, ego_speed_mps);
    cruise_input_ = make_adaptive_input(now_s, cruise_.enabled, vehicle_, last_result_, panda_, model_updated_,
                                        observe_cruise_lead(lead, model_.lead_track), ego_speed_kph);
    adaptive_cruise_ = adaptive_cruise_controller_.update(cruise_input_);

    if (adaptive_cruise_.command_button != 0) {
        last_result_.frames.push_back(
            create_cruise_button_frame(decode_clu11(vehicle_.clu11_seed), adaptive_cruise_.command_button, frame));
        last_result_.should_send = true;
    }

    const DepartureAlertOutput departure_alert = departure_alert_detector_.update(alert_input_);
    if (departure_alert.event_id != 0 && departure_alert.event_id != last_logged_alert_event_id_) {
        last_logged_alert_event_id_ = departure_alert.event_id;
        std::fprintf(stderr,
                     "controlsd: departure alert=%s event=%u "
                     "visionLead=%.1fm rel=%.1fm/s p=%.2f plan=%.1fm gas2=%.2f\n",
                     departure_alert_name(departure_alert.type), departure_alert.event_id, alert_input_.lead_distance_m,
                     alert_input_.lead_relative_speed_mps, model_.lead.probability, alert_input_.plan_distance_m,
                     alert_input_.gas_press_prob);
    }
    // 유한하지 않은 입력으로 토크를 0으로 낸 틱이 늘면 한 번 남긴다(원인은 학습값·모델·차량 상태 쪽에 있다)
    const uint64_t nonfinite = controller_.torque_nonfinite_ticks();
    if (nonfinite != logged_nonfinite_ticks_) {
        std::fprintf(stderr, "controlsd: non-finite torque input, torque 0 (%llu ticks so far)\n",
                     static_cast<unsigned long long>(nonfinite));
        logged_nonfinite_ticks_ = nonfinite;
    }
    model_updated_ = false;
    return make_control_state(departure_alert, radar_lead_fresh, ego_speed_kph, now_s);
}

LearnerOutput ControlsTick::update_learners(double now_s) {
    // 학습기는 이번 틱에 실제로 보낸 토크로 갱신하고, 컨트롤러는 다음 틱에 쓴다. torqued 지연은
    // 컨트롤러와 같은 조향 지연(상류는 lateralDelay). 조향 여부는 상류 latActive다: MDPS 일시 고장·정차·큰 조향각
    // 요청 끔 동안의 점은 torqued가 거른다(예전에는 active를 넘겨 그 점도 들어갔다).
    learners_.set_lateral_delay(controller_.plan_delay_s());
    learners_.update(vehicle_, now_s, last_result_.vehicle_fresh, last_result_.esp12_fresh,
                     last_result_.control_speed_kph / 3.6f, last_result_.lat_active, last_result_.apply_torque,
                     last_result_.steering_pressed);
    controller_.set_live_params(learners_.live(), learners_.vehicle_valid());
    controller_.set_calibration_status(static_cast<CalibrationStatus>(model_.calibration.status));
    LearnerOutput out;
    if (learners_.vehicle_persist_due()) {
        emit_if_changed(learners_.vehicle_persist_data(), &saved_vehicle_data_, &out.vehicle_persist_data);
        const VehicleParams &v = learners_.vehicle_params();
        const TorqueParams &t = learners_.torque_params();
        std::fprintf(stderr,
                     "controlsd: learners sr=%.2f stiffness=%.2f offset=%.2f/%.2f "
                     "roll=%.2f valid=%d | torque points=%d cal=%d%% factor=%.2f/%.2f "
                     "offset=%.3f friction=%.3f/%.3f valid=%d\n",
                     v.steer_ratio, v.stiffness_factor, v.angle_offset_average_deg, v.angle_offset_deg,
                     v.roll_rad * 57.29577951308232, v.valid ? 1 : 0, t.total_bucket_points, t.cal_perc,
                     t.lat_accel_factor_raw, t.lat_accel_factor, t.lat_accel_offset, t.friction_raw, t.friction,
                     t.valid ? 1 : 0);
    }
    if (learners_.torque_persist_due())
        emit_if_changed(learners_.torque_persist_data(), &saved_torque_data_, &out.torque_persist_data);
    if (learners_.vehicle_published()) out.learner_state = make_learner_state();
    return out;
}

LearnerOutput ControlsTick::final_learner_outputs() {
    LearnerOutput out;
    emit_if_changed(learners_.vehicle_persist_data(), &saved_vehicle_data_, &out.vehicle_persist_data);
    emit_if_changed(learners_.torque_persist_data(), &saved_torque_data_, &out.torque_persist_data);
    return out;
}
