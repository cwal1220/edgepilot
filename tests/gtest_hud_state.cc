// hud_state: 공유 상태 스냅샷 → HUD 상태 매핑과 알림 카드 선택(우선순위). 보드 없이 돈다.
#include "car/hyundai_can.h"
#include "controls/control_block.h"
#include "hud/hud_state.h"

#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

TEST(HudState, AlertCardPriority) {
    HudState hud;
    std::snprintf(hud.engage_reject_label, sizeof(hud.engage_reject_label), "DOOR OPEN");
    hud.soft_disabling = true;
    std::snprintf(hud.active_block, sizeof(hud.active_block), "calibration_invalid");
    hud.steering_fault = true;
    hud.panda_faults = 1;
    hud.controller_active = true;
    hud.steer_paused = true;
    hud.steer_saturated = true;
    hud.services_healthy = false;
    hud.lane_change = HudLaneChange::Pending;
    hud.lane_change_direction = -1;
    hud.departure_alert_type = DepartureAlertType::LeadDeparted;

    // 위에서부터 하나씩 끄면 다음 카드가 나온다
    HudAlertCard alert = hud_select_alert(hud, true);
    EXPECT_EQ(alert.title, "UNABLE TO ENGAGE");
    EXPECT_EQ(alert.detail, "DOOR OPEN");
    EXPECT_EQ(alert.level, HudAlertLevel::Caution);
    hud.engage_reject_label[0] = '\0';
    alert = hud_select_alert(hud, true);
    EXPECT_EQ(alert.title, "TAKE CONTROL");
    EXPECT_EQ(alert.detail, "CALIB INVALID") << "해제 예고는 사유 라벨";
    EXPECT_EQ(alert.level, HudAlertLevel::Critical);
    hud.soft_disabling = false;
    alert = hud_select_alert(hud, true);
    EXPECT_EQ(alert.title, "STEERING FAULT");
    EXPECT_EQ(alert.detail, "MODEL OK   CAR --   PANDA --") << "결함 카드는 연결 상태";
    EXPECT_EQ(alert.level, HudAlertLevel::Critical);
    hud.steering_fault = false;
    EXPECT_EQ(hud_select_alert(hud, false).title, "PANDA FAULT");
    hud.panda_faults = 0;
    alert = hud_select_alert(hud, true);
    EXPECT_EQ(alert.title, "STEERING PAUSED");
    EXPECT_EQ(alert.detail, "Wheel past 85\xb0, resumes below it");
    EXPECT_EQ(alert.level, HudAlertLevel::Caution);
    hud.steer_paused = false;
    alert = hud_select_alert(hud, true);
    EXPECT_EQ(alert.title, "TAKE CONTROL");
    EXPECT_EQ(alert.detail, "Turn exceeds steering limit") << "조향 한계는 주황";
    EXPECT_EQ(alert.level, HudAlertLevel::Caution);
    hud.steer_saturated = false;
    EXPECT_EQ(hud_select_alert(hud, true).title, "WAITING FOR SERVICES");
    hud.services_healthy = true;
    alert = hud_select_alert(hud, true);
    EXPECT_EQ(alert.title, "LANE CHANGE");
    EXPECT_EQ(alert.arrow, -1);
    EXPECT_EQ(alert.level, HudAlertLevel::Notice);
    hud.lane_change = HudLaneChange::None;
    alert = hud_select_alert(hud, true);
    EXPECT_EQ(alert.title, "LEAD VEHICLE MOVING");
    EXPECT_EQ(alert.level, HudAlertLevel::Proceed);
    hud.departure_alert_type = DepartureAlertType::GreenLight;
    EXPECT_EQ(hud_select_alert(hud, true).title, "GREEN LIGHT");
    hud.departure_alert_type = DepartureAlertType::None;
    EXPECT_TRUE(hud_select_alert(hud, true).empty());

    // 조건이 붙은 카드
    HudState idle;
    idle.services_healthy = true;
    idle.steer_paused = true;
    idle.lane_change = HudLaneChange::Pending;
    EXPECT_TRUE(hud_select_alert(idle, true).empty()) << "조향 쉼과 차선 변경 대기는 활성일 때만";
    idle.soft_disabling = true;
    std::snprintf(idle.active_block, sizeof(idle.active_block), "calibration_invalid");
    EXPECT_EQ(hud_select_alert(idle, true).detail, "CALIB INVALID") << "해제 예고 사유";
    idle.soft_disabling = false;
    idle.controller_active = true;
    idle.steer_paused_by_driver = true;
    EXPECT_EQ(hud_select_alert(idle, true).detail, "Resumes on the plan or below 15\xb0");
}

TEST(HudState, ControlStateMapping) {
    ControlState c;
    c.enabled = 1;
    c.engaged = 1;
    c.cluster_speed_kph = 63.5f;
    c.hud_flags = kHudFlagLaneless;
    c.departure_alert_type = static_cast<uint32_t>(DepartureAlertType::GreenLight);
    c.apply_torque = -120;
    std::snprintf(c.active_block, sizeof(c.active_block), "%s", "not_engaged");

    HudState hud;
    hud_apply_control_state(c, true, &hud);
    // 신선한 제어 스냅샷은 필드 그대로 옮긴다
    ASSERT_TRUE(hud.controller_enabled);
    ASSERT_TRUE(hud.controller_engaged);
    ASSERT_TRUE(hud.laneless_mode);
    ASSERT_EQ(hud.cluster_speed_kph, 63.5f);
    ASSERT_EQ(hud.apply_torque, -120);
    ASSERT_EQ(hud.departure_alert_type, DepartureAlertType::GreenLight);
    ASSERT_STREQ(hud.active_block, "not_engaged");
    hud_apply_control_state(c, false, &hud);
    // 낡은 스냅샷은 HUD를 비우고 control_stale로 표시한다
    ASSERT_FALSE(hud.controller_enabled);
    ASSERT_FALSE(hud.laneless_mode);
    ASSERT_EQ(hud.cluster_speed_kph, 0.0f);
    ASSERT_EQ(hud.apply_torque, 0);
    ASSERT_EQ(hud.departure_alert_type, DepartureAlertType::None);
    ASSERT_STREQ(hud.active_block, "control_stale");
    // 결합 차단 라벨: 표에 없는 이름(다른 빌드의 controlsd)은 그대로
    ASSERT_STREQ(engage_block_label("panda_not_ready"), "PANDA NOT READY");
    ASSERT_STREQ(engage_block_label("no_such_reason"), "no_such_reason");
    for (const BlockReasonRow &row : kBlockReasons) {
        if (row.reason == BlockReason::None) continue;
        const char *label = engage_block_label(row.name);
        // 모든 차단 사유가 제 라벨로 HUD에 뜬다
        ASSERT_NE(label, nullptr);
        ASSERT_NE(label[0], '\0');
        ASSERT_STREQ(label, row.label);
    }
}

TEST(HudState, ManeuverFlagsMapping) {
    ControlState c;
    c.hud_flags = kHudFlagLaneChangePending | kHudFlagLaneChangeRight;
    c.driver_torque = -96;
    HudState hud;
    hud_apply_control_state(c, true, &hud);
    // 차선 변경 대기(오른쪽)와 운전자 토크 눈금
    ASSERT_EQ(hud.lane_change, HudLaneChange::Pending);
    ASSERT_EQ(hud.lane_change_direction, 1);
    ASSERT_FALSE(hud.steer_paused);
    ASSERT_NEAR(hud.driver_torque_fraction, -96.0f / kHyundaiSteerMax, 1e-6f);
    c.hud_flags = kHudFlagLaneChanging | kHudFlagSteerPaused | kHudFlagSteerPausedByDriver;
    hud_apply_control_state(c, true, &hud);
    ASSERT_EQ(hud.lane_change, HudLaneChange::Changing);
    ASSERT_EQ(hud.lane_change_direction, -1);
    ASSERT_TRUE(hud.steer_paused);
    ASSERT_TRUE(hud.steer_paused_by_driver);
    hud_apply_control_state(c, false, &hud);
    // 낡은 스냅샷은 조작 표시를 끈다
    ASSERT_EQ(hud.lane_change, HudLaneChange::None);
    ASSERT_FALSE(hud.steer_paused);
    ASSERT_EQ(hud.driver_torque_fraction, 0.0f);
}

TEST(HudState, LearnerAndLaneMapping) {
    LearnerState learner;
    learner.flags = kLearnerSteerRatioValid | kLearnerStiffnessValid | kLearnerOffsetAverageValid;
    learner.steer_ratio = 15.43f;
    learner.lat_accel_factor_raw = 3.45f;
    learner.cal_perc = 74;
    learner.plan_delay_s = 0.42f;
    HudState hud;
    hud_apply_learner_state(learner, true, &hud);
    // paramsd 세 값이 다 유효해야 유효, torqued는 따로
    ASSERT_TRUE(hud.learner_fresh);
    ASSERT_TRUE(hud.params_valid);
    ASSERT_FALSE(hud.torque_valid);
    ASSERT_EQ(hud.steer_ratio, 15.43f);
    ASSERT_EQ(hud.torque_factor, 3.45f);
    ASSERT_EQ(hud.torque_cal_percent, 74);
    learner.flags &= ~kLearnerStiffnessValid;
    hud_apply_learner_state(learner, true, &hud);
    ASSERT_FALSE(hud.params_valid);

    // 학습 카드: 제어가 쓰는 형태의 값(빠른 영점, 필터한 토크 계수)과 쓰는지
    ASSERT_FALSE(hud.vehicle_learned || hud.torque_learned || hud.delay_learned);
    learner.flags |= kLearnerUseVehicle | kLearnerUseTorque | kLearnerUseDelay;
    learner.angle_offset_deg = -1.58f;
    learner.lat_accel_factor = 2.31f;
    hud_apply_learner_state(learner, true, &hud);
    ASSERT_TRUE(hud.vehicle_learned && hud.torque_learned && hud.delay_learned);
    ASSERT_EQ(hud.angle_offset_fast_deg, -1.58f);
    ASSERT_EQ(hud.torque_factor_filtered, 2.31f);
    hud_apply_learner_state(learner, false, &hud);
    ASSERT_FALSE(hud.learner_fresh);
    ASSERT_FALSE(hud.vehicle_learned || hud.torque_learned || hud.delay_learned)
        << "멈춘 controlsd의 값은 쓰는 값이 아니다";

    LocalizationState localization;
    localization.lag_valid_blocks = 3;
    hud_apply_localization_state(localization, true, &hud);
    ASSERT_EQ(hud.lag_blocks, 3);
    hud_apply_localization_state(localization, false, &hud);
    ASSERT_EQ(hud.lag_blocks, -1) << "locationd가 멈추면 lagd 없음";

    ParsedModelOutput output;
    output.valid = true;
    output.lanes[1].probability = output.lanes[2].probability = 0.9f;
    output.lanes[1].points[0].y = -1.70f;  // 왼쪽 선
    output.lanes[2].points[0].y = 1.86f;   // 오른쪽 선
    // 차선 중앙이 0.08 m 오른쪽 = 차가 왼쪽에 있다
    ASSERT_NEAR(lane_center_offset_m(output), 0.08f, 1e-6f);
    output.lanes[2].probability = 0.3f;
    ASSERT_TRUE(std::isnan(lane_center_offset_m(output))) << "한쪽 선이 불확실하면 모른다";
}

TEST(HudState, CalibrationMapping) {
    ModelState m;
    m.calibration.status = 1;
    m.calibration.valid_blocks = 12;
    m.calibration.yaw = -0.03f * 3.14159265f / 180.0f;
    HudState hud;
    hud_apply_model_state(m, true, &hud);
    EXPECT_TRUE(hud.calibration_available);
    EXPECT_EQ(hud.calibration_status, 1u);
    EXPECT_NEAR(hud.calibration_yaw_deg, -0.03f, 1e-5f);
    EXPECT_FALSE(hud.calibration_flow) << "출처 0 = 모델 포즈";
    m.calibration_source.source = 1;
    hud_apply_model_state(m, true, &hud);
    EXPECT_TRUE(hud.calibration_flow) << "광류 추정을 쓰면 CAL 카드가 FLOW";
}

TEST(HudState, RecordStateMapping) {
    RecordState r;
    r.active = 1;
    HudState hud;
    hud_apply_record_state(r, true, &hud);
    // route를 쓰는 중이면 REC
    ASSERT_TRUE(hud.recording);
    ASSERT_FALSE(hud.storage_full);
    r.active = 0;
    r.storage_blocked = kStorageBlockedSdReserve;
    hud_apply_record_state(r, true, &hud);
    // 저장 공간 때문에 멈추면 REC 대신 저장 공간 경고
    ASSERT_FALSE(hud.recording);
    ASSERT_TRUE(hud.storage_full);
    ASSERT_FALSE(hud.storage_slow);
    r.storage_blocked = kStorageBlockedStaging;
    hud_apply_record_state(r, true, &hud);
    // 스테이징이 차서(SD가 느려서) 멈추면 STORAGE FULL이 아니라 SD CARD SLOW
    ASSERT_FALSE(hud.storage_full);
    ASSERT_TRUE(hud.storage_slow);
    r.active = 1;
    hud_apply_record_state(r, false, &hud);
    // recordd가 멈춰 스냅샷이 낡으면 다 끈다
    ASSERT_FALSE(hud.recording);
    ASSERT_FALSE(hud.storage_full);
    ASSERT_FALSE(hud.storage_slow);
}

}  // namespace
