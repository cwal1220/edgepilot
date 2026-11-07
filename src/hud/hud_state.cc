#include "hud/hud_state.h"

#include "car/hyundai_can.h"
#include "common/utils_math.h"
#include "controls/control_block.h"

#include <algorithm>
#include <cstdio>
#include <limits>

const char *engage_block_label(const char *block) {
    const BlockReason reason = block_reason_from_name(block);
    return reason == BlockReason::Count ? block : block_reason_label(reason);
}

HudAlertCard hud_select_alert(const HudState &hud, bool model_ok) {
    char links[96];
    std::snprintf(links, sizeof(links), "MODEL %s   CAR %s   PANDA %s", model_ok ? "OK" : "--",
                  hud.vehicle_fresh ? "OK" : "--", hud.panda_connected ? "OK" : "--");
    if (hud.engage_reject_label[0] != '\0')
        return {"UNABLE TO ENGAGE", hud.engage_reject_label, HudAlertLevel::Caution};
    if (hud.soft_disabling) return {"TAKE CONTROL", engage_block_label(hud.active_block), HudAlertLevel::Critical};
    if (hud.steering_fault) return {"STEERING FAULT", links, HudAlertLevel::Critical};
    if (hud.panda_faults != 0) return {"PANDA FAULT", links, HudAlertLevel::Critical};
    if (hud.controller_active && hud.steer_paused)
        return {
            "STEERING PAUSED",
            hud.steer_paused_by_driver ? "Resumes on the plan or below 15\xb0" : "Wheel past 85\xb0, resumes below it",
            HudAlertLevel::Caution};
    if (hud.steer_saturated) return {"TAKE CONTROL", "Turn exceeds steering limit", HudAlertLevel::Caution};
    if (!hud.services_healthy) return {"WAITING FOR SERVICES", links, HudAlertLevel::Caution};
    if (hud.controller_active && hud.lane_change == HudLaneChange::Pending) {
        const bool left = hud.lane_change_direction < 0;
        return {"LANE CHANGE", left ? "Steer left to start once safe" : "Steer right to start once safe",
                HudAlertLevel::Notice, left ? -1 : 1};
    }
    if (hud.departure_alert_type == DepartureAlertType::LeadDeparted)
        return {"LEAD VEHICLE MOVING", "Check the road and proceed", HudAlertLevel::Proceed};
    if (hud.departure_alert_type == DepartureAlertType::GreenLight)
        return {"GREEN LIGHT", "Check the road and proceed", HudAlertLevel::Proceed};
    return {};
}

// ---- 공유 상태 → HUD 상태 ----

// 낡은 스냅샷은 빈 스냅샷(모든 값 0)으로 읽는다.
void hud_apply_panda_state(const PandaState &panda, bool fresh, HudState *hud) {
    static const PandaState kStale{};
    const PandaState &p = fresh ? panda : kStale;
    hud->panda_connected = p.connected != 0;
    hud->panda_healthy = p.comms_healthy != 0;
    hud->panda_faults = p.faults;
}

void hud_apply_control_state(const ControlState &control, bool fresh, HudState *hud) {
    static const ControlState kStale{};
    const ControlState &c = fresh ? control : kStale;
    hud->controller_enabled = c.enabled != 0;
    hud->controller_engaged = c.engaged != 0;
    hud->controller_active = c.active != 0;
    hud->vehicle_fresh = c.vehicle_fresh != 0;
    hud->steering_fault = c.steering_fault != 0;
    hud->left_blinker = c.left_blinker != 0;
    hud->right_blinker = c.right_blinker != 0;
    hud->cruise_active = c.cruise_active != 0;
    const uint32_t flags = c.hud_flags;
    hud->laneless_mode = (flags & kHudFlagLaneless) != 0;
    hud->brake_hold = (flags & kHudFlagBrakeHold) != 0;
    hud->soft_disabling = (flags & kHudFlagSoftDisabling) != 0;
    hud->steer_saturated = (flags & kHudFlagSteerSaturated) != 0;
    hud->lane_change = flags & kHudFlagLaneChanging        ? HudLaneChange::Changing
                       : flags & kHudFlagLaneChangePending ? HudLaneChange::Pending
                                                           : HudLaneChange::None;
    hud->lane_change_direction = flags & kHudFlagLaneChangeRight ? 1 : -1;
    hud->steer_paused = (flags & kHudFlagSteerPaused) != 0;
    hud->steer_paused_by_driver = (flags & kHudFlagSteerPausedByDriver) != 0;
    hud->gear = c.gear;
    hud->cluster_speed_kph = c.cluster_speed_kph;
    hud->ego_speed_kph = c.ego_speed_kph;
    hud->cruise_max_speed_kph = c.cruise_max_speed_kph;
    hud->cruise_command_speed_kph = c.cruise_command_speed_kph;
    hud->radar_lead_valid = c.radar_lead_valid != 0;
    hud->radar_lead_distance_m = c.radar_lead_distance_m;
    hud->radar_lead_relative_speed_mps = c.radar_lead_relative_speed_mps;
    hud->departure_alert_type = static_cast<DepartureAlertType>(c.departure_alert_type);
    hud->green_light_alert_armed = c.green_light_alert_armed != 0;
    hud->tpms_valid = c.tpms_valid != 0;
    hud->tpms_unit = static_cast<int>(c.tpms_unit);
    hud->tpms_pressure_fl = c.tpms_pressure_fl;
    hud->tpms_pressure_fr = c.tpms_pressure_fr;
    hud->tpms_pressure_rl = c.tpms_pressure_rl;
    hud->tpms_pressure_rr = c.tpms_pressure_rr;
    hud->tpms_warning = c.tpms_warning != 0;
    hud->steering_angle_deg = c.steering_angle_deg;
    hud->normalized_output = c.normalized_output;
    hud->desired_torque = c.desired_torque;
    hud->apply_torque = c.apply_torque;
    hud->driver_torque = c.driver_torque;
    // steer_max는 panda가 강제하는 고정값이라 런타임에서 바뀌지 않는다(car/hyundai_can.h). 운전자 토크는 이를 넘기도
    // 한다(녹화). 녹화로 확인한 부호: apply_torque 양수 = 왼쪽 조향(조향각·모델 plan과 같은 방향).
    const float steer_max = static_cast<float>(kHyundaiSteerMax);
    hud->steer_torque_fraction = std::clamp(static_cast<float>(c.apply_torque) / steer_max, -1.0f, 1.0f);
    hud->driver_torque_fraction = std::clamp(static_cast<float>(c.driver_torque) / steer_max, -1.0f, 1.0f);
    std::snprintf(hud->active_block, sizeof(hud->active_block), "%s", fresh ? c.active_block : "control_stale");
}

void hud_apply_model_state(const ModelState &model, bool fresh, HudState *hud) {
    const CalibrationState &calibration = model.calibration;
    hud->calibration_available = fresh;
    hud->calibration_status = calibration.status;
    hud->calibration_valid_blocks = calibration.valid_blocks;
    hud->calibration_roll_deg = rad_to_deg(calibration.roll);
    hud->calibration_pitch_deg = rad_to_deg(calibration.pitch);
    hud->calibration_yaw_deg = rad_to_deg(calibration.yaw);
    hud->calibration_flow = model.calibration_source.source == static_cast<uint32_t>(CalibrationSource::Flow);
}

void hud_apply_record_state(const RecordState &record, bool fresh, HudState *hud) {
    static const RecordState kStale{};
    const RecordState &r = fresh ? record : kStale;
    hud->recording = r.active != 0;
    hud->storage_full = r.storage_blocked == kStorageBlockedSdReserve;
    hud->storage_slow = r.storage_blocked == kStorageBlockedStaging;
}

void hud_apply_learner_state(const LearnerState &learner, bool fresh, HudState *hud) {
    constexpr uint32_t kParamsValid = kLearnerSteerRatioValid | kLearnerStiffnessValid | kLearnerOffsetAverageValid;
    hud->learner_fresh = fresh;
    hud->params_valid = fresh && (learner.flags & kParamsValid) == kParamsValid;
    hud->steer_ratio = learner.steer_ratio;
    hud->stiffness = learner.stiffness_factor;
    hud->angle_offset_deg = learner.angle_offset_average_deg;
    hud->torque_valid = fresh && (learner.flags & kLearnerTorqueValid) != 0;
    hud->torque_factor = learner.lat_accel_factor_raw;
    hud->torque_friction = learner.friction_raw;
    hud->torque_offset = learner.lat_accel_offset_raw;
    hud->torque_cal_percent = learner.cal_perc;
    hud->lateral_delay_s = learner.plan_delay_s;
    hud->vehicle_learned = fresh && (learner.flags & kLearnerUseVehicle) != 0;
    hud->torque_learned = fresh && (learner.flags & kLearnerUseTorque) != 0;
    hud->delay_learned = fresh && (learner.flags & kLearnerUseDelay) != 0;
    hud->angle_offset_fast_deg = learner.angle_offset_deg;
    hud->torque_factor_filtered = learner.lat_accel_factor;
}

void hud_apply_localization_state(const LocalizationState &localization, bool fresh, HudState *hud) {
    hud->lag_blocks = fresh ? localization.lag_valid_blocks : -1;
    hud->lag_estimate_s = localization.lag_estimate_s;
}

float lane_center_offset_m(const ParsedModelOutput &output) {
    constexpr float kMinProbability = 0.5f;
    const ParsedLaneLine &left = output.lanes[1], &right = output.lanes[2];
    if (!output.valid || left.probability < kMinProbability || right.probability < kMinProbability)
        return std::numeric_limits<float>::quiet_NaN();
    return (left.points[0].y + right.points[0].y) / 2.0f;
}

void hud_apply_manager_state(const ManagerState &manager, bool fresh, bool model_ok, HudState *hud) {
    const unsigned total = fresh ? manager.process_count : 0;
    unsigned running = 0;
    for (unsigned i = 0; i < total; ++i) running += manager.processes[i].running ? 1U : 0U;
    hud->services_healthy = fresh && total >= 3 && running == total && model_ok;
}

// ---- ModelState → ParsedModelOutput·ProjectionState ----

ParsedModelOutput parsed_from_model_state(const ModelState &state) {
    ParsedModelOutput parsed;
    parsed.valid = state.valid != 0;
    for (int i = 0; i < kTrajectorySize; ++i) {
        parsed.plan.points[i] = {state.plan[i].x, state.plan[i].y, state.plan[i].z};
        for (int lane = 0; lane < 4; ++lane) {
            parsed.lanes[lane].probability = state.lane_probabilities[lane];
            parsed.lanes[lane].points[i] = {model_x_idx(i), state.lanes[lane][i].y, state.lanes[lane][i].z};
        }
        for (int edge = 0; edge < 2; ++edge) {
            parsed.road_edges[edge].std = state.road_edge_stds[edge];
            parsed.road_edges[edge].points[i] = {model_x_idx(i), state.road_edges[edge][i].y,
                                                 state.road_edges[edge][i].z};
        }
    }
    if (state.lead.valid) {
        parsed.leads.global_probabilities[0] = state.lead.probability;
        parsed.leads.predictions[0].points[0] = {
            state.lead.x,
            state.lead.y,
            state.lead.velocity,
            state.lead.acceleration,
        };
    }
    return parsed;
}

ProjectionState projection_from_model_state(const ModelState &state) {
    ProjectionState projection =
        make_projection_state(state.calibration.roll, state.calibration.pitch, state.calibration.yaw);
    projection.lateral_offset_m = state.camera_offset_m;
    return projection;
}
