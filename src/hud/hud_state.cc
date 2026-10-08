#include "hud/hud_state.h"

#include "controls/control_block.h"
#include "controls/control_params.h"
#include "common/utils_math.h"

#include <algorithm>
#include <cstdio>
#include <limits>

const char *engage_block_label(const char *block)
{
    if (!block || block[0] == '\0') return nullptr;
    const BlockReason reason = block_reason_from_name(block);
    return reason == BlockReason::Count ? nullptr : block_reason_label(reason);
}

HudAlertCard hud_select_alert(const HudState &hud, bool model_ok)
{
    char links[96];
    std::snprintf(links, sizeof(links), "MODEL %s   CAR %s   PANDA %s", model_ok ? "OK" : "--",
                  hud.vehicle_fresh ? "OK" : "--", hud.panda_connected ? "OK" : "--");
    if (hud.engage_reject_label[0] != '\0')
        return {"UNABLE TO ENGAGE", hud.engage_reject_label, HudAlertLevel::caution};
    if (hud.soft_disabling) {
        const char *label = engage_block_label(hud.active_block);
        return {"TAKE CONTROL", label ? label : "Disengaging", HudAlertLevel::critical};
    }
    if (hud.steering_fault) return {"STEERING FAULT", links, HudAlertLevel::critical};
    if (hud.panda_faults != 0) return {"PANDA FAULT", links, HudAlertLevel::critical};
    if (hud.controller_active && hud.steer_paused)
        return {"STEERING PAUSED",
                hud.steer_paused_by_driver ? "Resumes when you let go below 15\xb0" : "Wheel past 85\xb0, resumes below it",
                HudAlertLevel::caution};
    if (hud.steer_saturated) return {"TAKE CONTROL", "Turn exceeds steering limit", HudAlertLevel::caution};
    if (!hud.services_healthy) return {"WAITING FOR SERVICES", links, HudAlertLevel::caution};
    if (hud.controller_active && hud.lane_change == 1) {
        const bool left = hud.lane_change_direction < 0;
        return {"LANE CHANGE", left ? "Steer left to start once safe" : "Steer right to start once safe",
                HudAlertLevel::notice, left ? -1 : 1};
    }
    if (hud.departure_alert_type == DepartureAlertType::lead_departed)
        return {"LEAD VEHICLE MOVING", "Check the road and proceed", HudAlertLevel::proceed};
    if (hud.departure_alert_type == DepartureAlertType::green_light)
        return {"GREEN LIGHT", "Check the road and proceed", HudAlertLevel::proceed};
    return {};
}

/* ---- 공유 상태 → HUD 상태 ---- */

void hud_apply_panda_state(const PandaState &panda, bool fresh, HudState *hud)
{
    hud->panda_connected = fresh && panda.connected != 0;
    hud->panda_healthy = fresh && panda.comms_healthy != 0;
    hud->panda_faults = fresh ? panda.faults : 0;
}

void hud_apply_control_state(const ControlState &c, bool fresh, HudState *hud)
{
    hud->controller_enabled = fresh && c.enabled != 0;
    hud->controller_engaged = fresh && c.engaged != 0;
    hud->controller_active = fresh && c.active != 0;
    hud->laneless_mode = fresh && (c.hud_flags & kHudFlagLaneless) != 0;
    hud->vehicle_fresh = fresh && c.vehicle_fresh != 0;
    hud->steering_fault = fresh && c.steering_fault != 0;
    hud->left_blinker = fresh && c.left_blinker != 0;
    hud->right_blinker = fresh && c.right_blinker != 0;
    hud->cruise_active = fresh && c.cruise_active != 0;
    hud->brake_hold = fresh && (c.hud_flags & kHudFlagBrakeHold) != 0;
    hud->soft_disabling = fresh && (c.hud_flags & kHudFlagSoftDisabling) != 0;
    hud->steer_saturated = fresh && (c.hud_flags & kHudFlagSteerSaturated) != 0;
    const uint32_t flags = fresh ? c.hud_flags : 0U;
    hud->lane_change = flags & kHudFlagLaneChanging ? 2 : flags & kHudFlagLaneChangePending ? 1 : 0;
    hud->lane_change_direction = flags & kHudFlagLaneChangeRight ? 1 : -1;
    hud->steer_paused = (flags & kHudFlagSteerPaused) != 0;
    hud->steer_paused_by_driver = (flags & kHudFlagSteerPausedByDriver) != 0;
    hud->gear = fresh ? c.gear : 0;
    hud->cluster_speed_kph = fresh ? c.cluster_speed_kph : 0.0f;
    hud->ego_speed_kph = fresh ? c.ego_speed_kph : 0.0f;
    hud->cruise_max_speed_kph = fresh ? c.cruise_max_speed_kph : 0.0f;
    hud->cruise_command_speed_kph = fresh ? c.cruise_command_speed_kph : 0.0f;
    hud->radar_lead_valid = fresh && c.radar_lead_valid != 0;
    hud->radar_lead_distance_m = fresh ? c.radar_lead_distance_m : 0.0f;
    hud->radar_lead_relative_speed_mps = fresh ? c.radar_lead_relative_speed_mps : 0.0f;
    hud->departure_alert_type = fresh
        ? static_cast<DepartureAlertType>(c.departure_alert_type)
        : DepartureAlertType::none;
    hud->green_light_alert_armed = fresh && c.green_light_alert_armed != 0;
    hud->tpms_valid = fresh && c.tpms_valid != 0;
    hud->tpms_unit = fresh ? static_cast<int>(c.tpms_unit) : 0;
    hud->tpms_pressure_fl = fresh ? c.tpms_pressure_fl : 0.0f;
    hud->tpms_pressure_fr = fresh ? c.tpms_pressure_fr : 0.0f;
    hud->tpms_pressure_rl = fresh ? c.tpms_pressure_rl : 0.0f;
    hud->tpms_pressure_rr = fresh ? c.tpms_pressure_rr : 0.0f;
    hud->tpms_warning = fresh && c.tpms_warning != 0;
    hud->steering_angle_deg = fresh ? c.steering_angle_deg : 0.0f;
    hud->normalized_output = fresh ? c.normalized_output : 0.0f;
    hud->desired_torque = fresh ? c.desired_torque : 0;
    hud->apply_torque = fresh ? c.apply_torque : 0;
    /* steer_max는 panda가 강제하는 고정값이라 런타임에서 바뀌지 않는다(control_params.h).
     * 녹화로 확인한 부호: apply_torque 양수 = 왼쪽 조향(조향각·모델 plan과 같은 방향). */
    hud->steer_torque_fraction = fresh
        ? std::clamp(static_cast<float>(c.apply_torque) / static_cast<float>(SteeringParams{}.steer_max),
                     -1.0f, 1.0f)
        : 0.0f;
    hud->driver_torque = fresh ? c.driver_torque : 0;
    hud->driver_torque_fraction = fresh
        ? std::clamp(static_cast<float>(c.driver_torque) / static_cast<float>(SteeringParams{}.steer_max), -1.0f, 1.0f)
        : 0.0f;
    std::snprintf(hud->active_block, sizeof(hud->active_block), "%s",
                  fresh ? c.active_block : "control_stale");
}

void hud_apply_model_state(const ModelState &model, bool fresh, HudState *hud)
{
    const CalibrationState &calibration = model.calibration;
    hud->calibration_available = fresh;
    hud->calibration_status = calibration.status;
    hud->calibration_valid_blocks = calibration.valid_blocks;
    hud->calibration_roll_deg = rad_to_deg(calibration.roll);
    hud->calibration_pitch_deg = rad_to_deg(calibration.pitch);
    hud->calibration_yaw_deg = rad_to_deg(calibration.yaw);
}

void hud_apply_record_state(const RecordState &record, bool fresh, HudState *hud)
{
    hud->recording = fresh && record.active != 0;
    hud->storage_full = fresh && record.storage_blocked != 0;
}

void hud_apply_learner_state(const LearnerState &learner, bool fresh, HudState *hud)
{
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

void hud_apply_localization_state(const LocalizationState &localization, bool fresh, HudState *hud)
{
    hud->lag_blocks = fresh ? localization.lag_valid_blocks : -1;
    hud->lag_estimate_s = localization.lag_estimate_s;
}

float lane_center_offset_m(const ParsedModelOutput &output)
{
    constexpr float kMinProbability = 0.5f;
    const ParsedLaneLine &left = output.lanes[1], &right = output.lanes[2];
    if (!output.valid || !left.valid || !right.valid || left.probability < kMinProbability ||
        right.probability < kMinProbability)
        return std::numeric_limits<float>::quiet_NaN();
    return (left.points[0].y + right.points[0].y) / 2.0f;
}

void hud_apply_manager_state(const ManagerState &manager, bool fresh, bool model_ok,
                             HudState *hud)
{
    const unsigned total = fresh
        ? std::min<unsigned>(manager.process_count, kMaxProcesses) : 0;
    unsigned running = 0;
    for (unsigned i = 0; i < total; ++i) running += manager.processes[i].running ? 1U : 0U;
    hud->services_healthy = fresh && total >= 3 && running == total && model_ok;
}

/* ---- ModelState → ParsedModelOutput·ProjectionState ---- */

ParsedModelOutput parsed_from_model_state(const ModelState &state)
{
    ParsedModelOutput parsed;
    parsed.valid = state.valid != 0;
    parsed.plan.valid = parsed.valid;
    parsed.plan.best_index = state.best_plan;
    parsed.plan.probability = state.plan_probability;
    for (int i = 0; i < kTrajectorySize; ++i) {
        parsed.plan.points[i] = {state.plan[i].x, state.plan[i].y, state.plan[i].z};
        parsed.plan.yaw[i] = state.plan_yaw[i];
        parsed.plan.yaw_rate[i] = state.plan_yaw_rate[i];
        for (int lane = 0; lane < 4; ++lane) {
            parsed.lanes[lane].valid = parsed.valid;
            parsed.lanes[lane].probability = state.lane_probabilities[lane];
            parsed.lanes[lane].std = state.lane_stds[lane];
            parsed.lanes[lane].points[i] = {
                state.lanes[lane][i].x,
                state.lanes[lane][i].y,
                state.lanes[lane][i].z,
            };
        }
        for (int edge = 0; edge < 2; ++edge) {
            parsed.road_edges[edge].valid = parsed.valid;
            parsed.road_edges[edge].std = state.road_edge_stds[edge];
            parsed.road_edges[edge].points[i] = {
                state.road_edges[edge][i].x,
                state.road_edges[edge][i].y,
                state.road_edges[edge][i].z,
            };
        }
    }
    for (int i = 0; i < kDesireLen; ++i)
        parsed.meta.desire_state[i] = state.desire_state[i];
    for (int i = 0; i < kMetaPressHorizons; ++i) {
        parsed.meta.gas_press[i] = state.gas_press_probs[i];
        parsed.meta.brake_press[i] = state.brake_press_probs[i];
    }

    if (state.lead.valid) {
        parsed.leads.valid = true;
        parsed.leads.global_probabilities[0] = state.lead.probability;
        parsed.leads.predictions[0].points[0] = {
            state.lead.x,
            state.lead.y,
            state.lead.velocity,
            state.lead.acceleration,
        };
    }

    parsed.has_pose = state.pose.valid != 0;
    if (parsed.has_pose) {
        for (int i = 0; i < 3; ++i) {
            parsed.pose.trans[i] = state.pose.trans[i];
            parsed.pose.rot[i] = state.pose.rot[i];
            parsed.pose.trans_std[i] = state.pose.trans_std[i];
            parsed.pose.rot_std[i] = state.pose.rot_std[i];
        }
    }
    return parsed;
}

ProjectionState projection_from_model_state(const ModelState &state)
{
    ProjectionState projection =
        make_projection_state(state.calibration.roll, state.calibration.pitch, state.calibration.yaw);
    projection.lateral_offset_m = state.camera_offset_m;
    return projection;
}
