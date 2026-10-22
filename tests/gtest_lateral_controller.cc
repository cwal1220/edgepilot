// 횡제어기(LateralController): 결합 게이트와 홀드, 토크 한계와 MDPS 고장 회피, 곡률 제한, Panda
// 게이트와 넘겨받기, LKAS HUD, 그리고 학습값(paramsd·torqued)과 lagd 지연 소비. CAN 신호는 gtest_car, 토크
// 제어기는 gtest_lateral_torque, 홀드는 gtest_control_holds, 플래너는 gtest_lateral_planner가 본다.
#include "car/hyundai_can.h"
#include "car/vehicle_can.h"
#include "common/model_output.h"
#include "controls/control_params.h"
#include "controls/lateral_controller.h"
#include "controls/lateral_torque.h"

#include <gtest/gtest.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

// Panda는 준비됐지만 조향을 허락하지 않은 상태(controls_allowed 꺼짐)
PandaGateOutput panda_without_controls() {
    PandaGateOutput out = panda_allows_control();
    out.controls_allowed = false;
    return out;
}

// 컨트롤러가 보는 수신 메시지 시각을 한 번에 t로 맞춘다.
void stamp_can_times(VehicleCanState *vehicle, double t) { stamp_received_messages(vehicle, t); }

VehicleCanState ready_vehicle(double timestamp_s = 1.0) {
    VehicleCanState vehicle;
    vehicle.has_lkas11_seed = true;
    vehicle.has_clu11_seed = true;
    vehicle.has_mdps12_seed = true;
    stamp_can_times(&vehicle, timestamp_s);
    // 저속 조향 게이트를 넘는 주행 상태가 헬퍼의 기본값이다.
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
    vehicle.cluster_speed_raw = 63.0f;
    vehicle.gear = 5;
    return vehicle;
}

LateralPath replay_path() {
    LateralPath path;
    path.usable_for_steering = true;
    path.point_count = 30;
    path.reach_m = 60.0f;
    return path;
}

LateralTarget replay_target() {
    LateralTarget target;
    target.valid = true;
    target.mpc_solution_valid = true;
    for (int i = 0; i < kLateralControlN; ++i) {
        target.psis[i] = 0.0008f * 20.0f * (10.0f * i * i / (32.0f * 32.0f));
        target.curvatures[i] = 0.0008f;
    }
    return target;
}

TEST(LateralController, BrakingDoesNotDisengage) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음
    vehicle.brake_light = true;

    const auto brake_light_result = controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
    ASSERT_TRUE(brake_light_result.active) << "DriverBraking 없는 브레이크등만으로는 활성을 유지한다";

    vehicle.brake_pressed = true;
    const auto brake_pressed_result = controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
    ASSERT_TRUE(brake_pressed_result.active) << "DriverBraking도 횡제어를 해제하지 않는다";
}

// K7 MDPS는 steer 요청이 켜진 채 85도 위에 0.98초 넘게 머물면 토크와 무관하게 fault를 낸다
// (2026-09-18 실측). 85도 위에서도 컨트롤러 토크는 그대로 내되, 크기를 kHyundaiSteerDeltaDown × 남은
// 프레임으로 묶어 89번째 프레임에 0에 닿게 하고, 그때부터 85도 아래로 돌아올 때까지 steer 요청을
// ToiFlt 없이 끈다. 2프레임 컷처럼 85도 위에서 요청을 다시 켜지 않는다(2026-10-03: 다시 켜는
// 순간 고장). 85도 아래로 오면 요청을 켜고 토크를 0부터 올린다.
TEST(LateralController, LargeAngleFaultAvoidance) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    const SteeringParams &sp = config.steering_params;
    VehicleCanState vehicle = ready_vehicle();
    vehicle.cluster_speed_raw = 72.0f;
    int frame = 0;
    auto step = [&]() {
        const int f = frame++;
        stamp_can_times(&vehicle, 1.0 + f * 0.01);  // 4초 넘게 돌리므로 CAN 신선도를 유지한다
        return controller.update(replay_path(), replay_target(), vehicle, 1.0 + f * 0.01, f);
    };
    auto lkas = [](const LateralControlOutput &r) {
        EXPECT_FALSE(r.frames.empty()) << "LKAS11을 보낸다";
        return r.frames.empty() ? HyundaiLkas11Values{} : decode_lkas11(r.frames.front().data);
    };

    vehicle.steering_angle_deg = 20.0f;
    LateralControlOutput result;
    for (int i = 0; i < 100; ++i) result = step();
    // MDPS 고장 각도 아래에서는 토크가 나간다
    ASSERT_TRUE(result.active);
    ASSERT_NE(result.apply_torque, 0);

    vehicle.steering_angle_deg = 100.0f;
    int full_until = 0;
    for (int since = 1; since < sp.avoid_lkas_fault_max_frames; ++since) {
        result = step();
        const int cap = kHyundaiSteerDeltaDown * (sp.avoid_lkas_fault_max_frames - since);
        // 요청을 끄는 프레임 전까지는 steer 요청을 유지하고, 토크는 그때 0에 닿을 만큼만 남긴다
        ASSERT_TRUE(result.active);
        ASSERT_FALSE(result.large_angle_hold) << since;
        ASSERT_TRUE(lkas(result).steer_req) << since;
        ASSERT_LE(std::abs(result.desired_torque), cap) << since;
        ASSERT_LE(std::abs(result.apply_torque), cap) << since;
        if (std::abs(result.desired_torque) == kHyundaiSteerMax) full_until = since;
    }
    ASSERT_GE(full_until, sp.avoid_lkas_fault_max_frames - kHyundaiSteerMax / kHyundaiSteerDeltaDown - 1)
        << "최대 요청은 하강에 필요한 프레임 전까지 그대로 둔다(예전 램프는 첫 프레임부터 줄였다)";

    result = step();
    // 89번째 프레임: 토크 0에서 요청을 끈다. 해제가 아니고 ToiFlt도 세우지 않는다
    ASSERT_TRUE(result.active);
    ASSERT_TRUE(result.large_angle_hold);
    ASSERT_EQ(result.apply_torque, 0);
    HyundaiLkas11Values off = lkas(result);
    ASSERT_FALSE(off.steer_req);
    ASSERT_FALSE(off.toi_fault);
    ASSERT_EQ(off.steer_torque, 0);
    for (int i = 0; i < 300; ++i) {
        result = step();
        off = lkas(result);
        // 85도 위에 있는 동안은 요청을 다시 켜지 않는다
        ASSERT_TRUE(result.large_angle_hold) << i;
        ASSERT_FALSE(off.steer_req) << i;
        ASSERT_EQ(result.apply_torque, 0) << i;
    }

    vehicle.steering_angle_deg = 20.0f;
    result = step();
    // 85도 아래로 오면 요청을 켜고 토크를 0부터 올린다
    ASSERT_FALSE(result.large_angle_hold);
    ASSERT_TRUE(lkas(result).steer_req);
    ASSERT_LE(std::abs(result.apply_torque), kHyundaiSteerDeltaUp);
    for (int i = 0; i < 30; ++i) result = step();
    ASSERT_TRUE(result.active);
    ASSERT_NE(result.apply_torque, 0);
}

// avoid_lkas_fault를 끄면 상류 hyundai carcontroller의 common_fault_avoidance다: 85도 위에서 요청을 89프레임
// 낸 뒤 2프레임은 요청을 끄고 ToiFlt를 켜며(토크는 그대로) 다시 센다.
TEST(LateralController, UpstreamAngleCutWhenAvoidanceOff) {
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.avoid_lkas_fault_enabled = false;
    const SteeringParams &sp = config.steering_params;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    int frame = 0;
    auto step = [&]() {
        const int f = frame++;
        stamp_can_times(&vehicle, 1.0 + f * 0.01);
        return controller.update(replay_path(), replay_target(), vehicle, 1.0 + f * 0.01, f);
    };
    vehicle.steering_angle_deg = 20.0f;
    for (int i = 0; i < 100; ++i) step();
    vehicle.steering_angle_deg = 100.0f;
    for (int cycle = 0; cycle < 2; ++cycle) {
        for (int since = 1; since <= sp.avoid_lkas_fault_max_frames; ++since) {
            const LateralControlOutput r = step();
            const HyundaiLkas11Values lkas = decode_lkas11(r.frames.front().data);
            ASSERT_TRUE(lkas.steer_req && !lkas.toi_fault) << cycle << " " << since;
            ASSERT_FALSE(r.large_angle_hold);
        }
        for (int cut = 0; cut < sp.avoid_lkas_fault_cut_frames; ++cut) {
            const LateralControlOutput r = step();
            const HyundaiLkas11Values lkas = decode_lkas11(r.frames.front().data);
            ASSERT_TRUE(!lkas.steer_req && lkas.toi_fault) << "2프레임 요청을 끄고 ToiFlt를 켠다 " << cut;
            ASSERT_TRUE(r.active);
            ASSERT_EQ(lkas.steer_torque, r.apply_torque) << "토크는 그대로 보낸다";
            ASSERT_NE(r.apply_torque, 0);
        }
    }
}

// MDPS 일시 고장(MDPS12 ToiFlt·ToiUnavail, 상류 steerFaultTemporary): 결합 중이면 그 프레임부터 조향을 쉬고
// (토크 0, 요청 비트 끔) steerTempUnavailable로 해제 예고를 띄운다. 풀리면 토크를 0부터 다시 올리고, 3초
// 이어지면 해제한다. 결합도 거부한다.
TEST(LateralController, MdpsFaultPausesSteeringAndSoftDisables) {
    LateralControllerConfig config;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle(1.0);
    int frame = 0;
    auto step = [&](double t) {
        stamp_can_times(&vehicle, t);
        vehicle.steering_angle_deg = 5.0f;
        return controller.update(replay_path(), replay_target(), vehicle, t, frame++, panda_allows_control());
    };
    auto press_set = [&](double t) {
        vehicle.clu_button = 2;
        step(t);
        vehicle.clu_button = 0;
        return step(t + 0.01);
    };
    ASSERT_TRUE(press_set(1.0).engaged);
    double t = 1.02;
    LateralControlOutput r;
    for (; t < 3.0; t += 0.01) r = step(t);
    ASSERT_TRUE(r.active);
    ASSERT_NE(r.apply_torque, 0);

    vehicle.steer_fault_temporary = true;
    r = step(t += 0.01);
    EXPECT_TRUE(r.steer_fault && r.engaged && r.active && r.soft_disabling);
    EXPECT_EQ(r.active_block, BlockReason::SteerTempUnavailable);
    EXPECT_EQ(r.apply_torque, 0) << "고장 프레임부터 토크를 내지 않는다(상류 latActive=false)";
    EXPECT_FALSE(decode_lkas11(r.frames.front().data).steer_req) << "요청 비트도 내린다";
    for (int i = 0; i < 20; ++i) r = step(t += 0.01);
    vehicle.steer_fault_temporary = false;
    r = step(t += 0.01);
    EXPECT_TRUE(r.active && !r.soft_disabling && r.active_block == BlockReason::None) << "풀리면 해제 예고도 거둔다";
    EXPECT_TRUE(decode_lkas11(r.frames.front().data).steer_req);
    EXPECT_LE(std::abs(r.apply_torque), kHyundaiSteerDeltaUp) << "토크는 0부터 다시 올린다";

    vehicle.steer_fault_temporary = true;
    for (const double start = t; t < start + 2.9; t += 0.01) {
        r = step(t);
        ASSERT_TRUE(r.engaged && r.soft_disabling) << t;
    }
    for (const double start = t; t < start + 0.2; t += 0.01) r = step(t);
    EXPECT_FALSE(r.engaged) << "3초 이어지면 해제한다";
    const LateralControlOutput again = press_set(t += 0.01);
    EXPECT_TRUE(again.engage_rejected) << "고장 중에는 결합을 거부한다";
    EXPECT_EQ(again.active_block, BlockReason::SteerTempUnavailable);
}

// 상류 car_events: 운전자가 핸들을 잡은 채 고장이 시작했거나, 놓은 지 1.5초가 안 됐거나, 정차면 해제 예고
// 없이 조향만 쉰다(그 고장 동안 유지).
TEST(LateralController, MdpsFaultIsQuietAfterDriverOverride) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    int frame = 0;
    double t = 1.0;
    auto step = [&]() {
        stamp_can_times(&vehicle, t);
        const LateralControlOutput r = controller.update(replay_path(), replay_target(), vehicle, t, frame++);
        t += 0.01;
        return r;
    };
    for (int i = 0; i < 100; ++i) step();
    vehicle.driver_torque = 300;
    for (int i = 0; i < 20; ++i) step();
    vehicle.steer_fault_temporary = true;
    LateralControlOutput r = step();
    ASSERT_TRUE(r.steering_pressed && r.steer_fault);
    EXPECT_EQ(r.active_block, BlockReason::None) << "운전자가 잡은 채 시작한 고장은 경고하지 않는다";
    EXPECT_FALSE(r.soft_disabling);
    EXPECT_EQ(r.apply_torque, 0) << "그래도 조향은 쉰다";
    vehicle.driver_torque = 0;
    for (int i = 0; i < 400; ++i) {
        r = step();
        ASSERT_FALSE(r.soft_disabling) << "놓은 직후에 이어진 고장은 그 고장 동안 조용하다 " << i;
        ASSERT_EQ(r.apply_torque, 0);
    }
    vehicle.steer_fault_temporary = false;
    for (int i = 0; i < 200; ++i) step();
    vehicle.steer_fault_temporary = true;
    r = step();
    EXPECT_TRUE(r.soft_disabling) << "놓은 지 1.5초가 지난 뒤의 새 고장은 해제 예고다";
    vehicle.steer_fault_temporary = false;
    step();
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
    vehicle.steer_fault_temporary = true;
    r = step();
    EXPECT_FALSE(r.soft_disabling) << "정차 중 고장은 조용하다";
}

// 상류 latActive: 최소 조향 속도(0.3 m/s보다 크면 그 속도) 이하나 바퀴가 멈춘 정차에서는 조향하지 않는다.
// 목표 곡률은 실제 곡률을 따라가고 토크는 0이다. 요청 비트는 K7 클러스터 경보 때문에 유지한다.
TEST(LateralController, StandstillTracksActualCurvature) {
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.angle_offset_deg = 0.0f;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 3.0f;  // 0.83 m/s: 최소 조향 속도 1 m/s 아래
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 3.0f;
    vehicle.steering_angle_deg = 30.0f;
    LateralControlOutput r;
    for (int i = 0; i < 300; ++i) {
        stamp_can_times(&vehicle, 1.0 + 0.01 * i);
        r = controller.update(replay_path(), replay_target(), vehicle, 1.0 + 0.01 * i, i);
    }
    ASSERT_TRUE(r.active);
    EXPECT_EQ(r.apply_torque, 0);
    EXPECT_NEAR(r.desired_curvature, r.actual_curvature, 1e-6f) << "정차에서는 목표가 실제 곡률을 따라간다";
    EXPECT_GT(std::fabs(r.actual_curvature), 0.005f) << "실제 곡률은 정차에서도 조향각으로 낸다(상류 self.curvature)";
    EXPECT_TRUE(decode_lkas11(r.frames.front().data).steer_req);
}

// 정차 대기(Stopped)도 steer 요청을 잡고 있으므로 같은 회피를 건다. 예전에는 active일 때만 세어,
// 정차에서 핸들을 85도 넘게 감고 출발하면 1초 뒤 고장이 났다(2026-10-03 2:54). 85도 위에서
// 결합하면 요청을 켜지 않고 85도 아래로 올 때까지 기다린다.
TEST(LateralController, LargeAngleHoldCoversStopAndEngage) {
    LateralControllerConfig config;
    config.force_engaged = true;
    const SteeringParams &sp = config.steering_params;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
    vehicle.cluster_speed_raw = 0.0f;
    LateralPath stopped_path = replay_path();
    stopped_path.usable_for_steering = false;
    int frame = 0;
    auto step = [&]() {
        const int f = frame++;
        stamp_can_times(&vehicle, 1.0 + f * 0.01);
        return controller.update(stopped_path, replay_target(), vehicle, 1.0 + f * 0.01, f);
    };
    auto steer_req = [](const LateralControlOutput &r) {
        EXPECT_FALSE(r.frames.empty()) << "LKAS11을 보낸다";
        return !r.frames.empty() && decode_lkas11(r.frames.front().data).steer_req;
    };

    vehicle.steering_angle_deg = 10.0f;
    LateralControlOutput result;
    for (int i = 0; i < 50; ++i) result = step();
    // 정차 대기는 토크 0으로 steer 요청을 유지한다
    ASSERT_EQ(result.active_block, BlockReason::Stopped);
    ASSERT_FALSE(result.active);
    ASSERT_TRUE(steer_req(result));

    vehicle.steering_angle_deg = 300.0f;
    for (int since = 1; since <= sp.avoid_lkas_fault_max_frames; ++since) {
        result = step();
        ASSERT_EQ(steer_req(result), since < sp.avoid_lkas_fault_max_frames)
            << "정차 대기에서도 85도 위 " << since << "프레임";
    }
    for (int i = 0; i < 100; ++i) ASSERT_FALSE(steer_req(step())) << i;
    vehicle.steering_angle_deg = 30.0f;
    ASSERT_TRUE(steer_req(step())) << "85도 아래로 오면 다시 켠다";

    LateralController late(config);
    VehicleCanState turning = ready_vehicle();
    turning.steering_angle_deg = 120.0f;
    for (int f = 0; f < 150; ++f) {
        stamp_can_times(&turning, 1.0 + f * 0.01);
        result = late.update(replay_path(), replay_target(), turning, 1.0 + f * 0.01, f);
        // 85도 위에서 결합해도 요청과 토크를 내지 않는다
        if (!result.frames.empty()) ASSERT_FALSE(decode_lkas11(result.frames.front().data).steer_req) << f;
        ASSERT_EQ(result.apply_torque, 0) << f;
    }
    turning.steering_angle_deg = 40.0f;
    stamp_can_times(&turning, 2.5);
    result = late.update(replay_path(), replay_target(), turning, 2.5, 150);
    ASSERT_TRUE(steer_req(result)) << "85도 아래로 오면 요청을 켠다";
}

// 정지 부근 path 깜빡임: active 재진입은 0.5s 연속 유효 후에만.
TEST(LateralController, PathFlickerDebounce) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    LateralPath bad = replay_path();
    bad.usable_for_steering = false;
    double t = 1.0;
    auto step = [&](const LateralPath &path) {
        VehicleCanState vehicle = ready_vehicle(t);
        const auto r = controller.update(path, replay_target(), vehicle, t, 0);
        t += 0.01;
        return r;
    };
    ASSERT_TRUE(step(replay_path()).active) << "처음 유효한 경로로 활성이 된다";
    ASSERT_FALSE(step(bad).active) << "경로가 무효가 되면 바로 비활성이 된다";
    int reactivated = 0;
    for (int i = 0; i < 20; ++i) reactivated += step(replay_path()).active ? 1 : 0;
    ASSERT_EQ(reactivated, 0) << "경로가 깜박여도 홀드 전에는 다시 활성이 되지 않는다";
    ASSERT_FALSE(step(bad).active) << "다음 끊김에서도 비활성";
    int active_after = 0;
    for (int i = 0; i < 60; ++i) active_after = step(replay_path()).active ? 1 : 0;
    ASSERT_EQ(active_after, 1) << "경로가 계속 유효하면 홀드 뒤 다시 활성이 된다";
}

// 정차(path 무효)에서도 결합은 받아야 한다 — 조향만 쉰다.
TEST(LateralController, EngageAllowedWithUnavailablePath) {
    LateralControllerConfig config;
    LateralController controller(config);
    LateralPath bad = replay_path();
    bad.usable_for_steering = false;
    VehicleCanState vehicle = ready_vehicle(1.0);
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
    vehicle.clu_button = 2;
    controller.update(bad, replay_target(), vehicle, 1.0, 0, panda_allows_control());
    vehicle.clu_button = 0;
    const auto engaged = controller.update(bad, replay_target(), vehicle, 1.01, 1, panda_allows_control());
    // 정차 중에는 경로가 없어도 결합을 받는다
    ASSERT_TRUE(engaged.engaged);
    ASSERT_FALSE(engaged.engage_rejected);
    // 정지 + path 무효는 오류가 아니라 대기 상태로 보고한다
    {
        // 경로 없는 정차는 오류가 아니라 정차로 알린다
        ASSERT_FALSE(engaged.active);
        ASSERT_EQ(engaged.active_block, BlockReason::Stopped);
    }
    // 대기 중에도 steer_req/스푸프는 유지(토크 0) — 정차 천이 부저 방지
    {
        // 사용 가능해질 때까지 steer_req를 토크 0으로 유지한다
        ASSERT_FALSE(engaged.frames.empty());
        ASSERT_TRUE(decode_lkas11(engaged.frames.front().data).steer_req);
        ASSERT_EQ(decode_lkas11(engaged.frames.front().data).steer_torque, 0);
    }
    // 주행 중 path 무효는 진짜 문제로 보고한다
    vehicle = ready_vehicle(1.02);
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
    const auto rolling = controller.update(bad, replay_target(), vehicle, 1.02, 2, panda_allows_control());
    // 주행 중 쓸 수 없는 경로는 path_invalid로 알린다
    ASSERT_FALSE(rolling.active);
    ASSERT_EQ(rolling.active_block, BlockReason::PathInvalid);
    // 결함은 가용성 대기보다 우선한다 (정차 중 문 열림 -> 즉시 해제)
    vehicle = ready_vehicle(1.03);
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
    vehicle.door_open = true;
    const auto door = controller.update(bad, replay_target(), vehicle, 1.03, 3, panda_allows_control());
    // 고장은 사용 가능 여부보다 우선하고 정차 중에도 바로 해제한다
    ASSERT_EQ(door.active_block, BlockReason::DoorOpen);
    ASSERT_FALSE(door.engaged);
}

// 신호 대기(정차, 경로 무효)에서도 SoftDisable 사유는 결합을 막는다. 앞선 사유가 Stopped(가용성 대기)라 예전에는
// 결합을 받아들인 뒤 3초 해제 예고로 풀었다(상류는 NO_ENTRY로 거부한다).
TEST(LateralController, SoftReasonRejectsEngageWhileStopped) {
    LateralControllerConfig config;
    LateralController controller(config);
    controller.set_calibration_status(CalibrationStatus::Uncalibrated);
    LateralPath stopped_path = replay_path();
    stopped_path.usable_for_steering = false;
    VehicleCanState vehicle = ready_vehicle(1.0);
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph =
        0.0f;
    vehicle.clu_button = 2;
    controller.update(stopped_path, replay_target(), vehicle, 1.0, 0, panda_allows_control());
    vehicle.clu_button = 0;
    const auto stopped = controller.update(stopped_path, replay_target(), vehicle, 1.01, 1, panda_allows_control());
    EXPECT_EQ(stopped.active_block, BlockReason::CalibrationIncomplete) << "거부 알림은 정차가 아니라 보정";
    EXPECT_TRUE(stopped.engage_rejected) << "정차 중에도 보정 미완료면 결합하지 않는다";
    EXPECT_FALSE(stopped.engaged);
    const auto after = controller.update(stopped_path, replay_target(), vehicle, 1.02, 2, panda_allows_control());
    EXPECT_FALSE(after.engage_rejected) << "한 번만 거부한다";

    // Panda 허가를 기다리는 동안(SET 직후)에도 기다리지 않고 보정으로 거부한다. 거부는 한 번이다.
    LateralController waiting(config);
    waiting.set_calibration_status(CalibrationStatus::Uncalibrated);
    VehicleCanState moving = ready_vehicle(1.0);
    moving.clu_button = 2;
    waiting.update(replay_path(), replay_target(), moving, 1.0, 0, panda_without_controls());
    moving.clu_button = 0;
    int rejections = 0;
    for (int i = 1; i <= 150; ++i) {
        const double t = 1.0 + 0.01 * i;
        stamp_can_times(&moving, t);
        const auto r = waiting.update(replay_path(), replay_target(), moving, t, i, panda_without_controls());
        if (i == 1) {
            EXPECT_TRUE(r.engage_rejected);
            EXPECT_EQ(r.active_block, BlockReason::CalibrationIncomplete);
        }
        EXPECT_FALSE(r.engaged);
        rejections += r.engage_rejected ? 1 : 0;
    }
    EXPECT_EQ(rejections, 1);
}

// openpilot의 calibrationIncomplete/Recalibrating/Invalid 이벤트(SOFT_DISABLE + NO_ENTRY).
TEST(LateralController, CalibrationGatesEngageAndSoftDisables) {
    LateralControllerConfig config;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle(1.0);
    auto press_set = [&](double t) {
        vehicle = ready_vehicle(t);
        vehicle.clu_button = 2;
        controller.update(replay_path(), replay_target(), vehicle, t, 0, panda_allows_control());
        vehicle = ready_vehicle(t + 0.01);
        return controller.update(replay_path(), replay_target(), vehicle, t + 0.01, 1, panda_allows_control());
    };

    controller.set_calibration_status(CalibrationStatus::Uncalibrated);
    const auto rejected = press_set(1.0);
    EXPECT_TRUE(rejected.engage_rejected) << "보정 미완료면 결합을 거부한다";
    EXPECT_FALSE(rejected.engaged);
    EXPECT_EQ(rejected.active_block, BlockReason::CalibrationIncomplete);

    controller.set_calibration_status(CalibrationStatus::Calibrated);
    const auto engaged = press_set(2.0);
    ASSERT_TRUE(engaged.engaged && engaged.active);

    // 결합 중 재보정: 경고를 띄운 채 3초 조향하고 해제한다.
    controller.set_calibration_status(CalibrationStatus::Recalibrating);
    LateralControlOutput r;
    double t = 2.02;
    for (; t < 2.02 + 2.9; t += 0.01) {
        vehicle = ready_vehicle(t);
        r = controller.update(replay_path(), replay_target(), vehicle, t, 2, panda_allows_control());
        ASSERT_TRUE(r.engaged && r.active && r.soft_disabling) << "해제 예고 중에는 조향을 유지한다 t=" << t;
        ASSERT_EQ(r.active_block, BlockReason::CalibrationRecalibrating);
    }
    for (; t < 2.02 + 3.2; t += 0.01) {
        vehicle = ready_vehicle(t);
        r = controller.update(replay_path(), replay_target(), vehicle, t, 2, panda_allows_control());
    }
    EXPECT_FALSE(r.engaged) << "3초가 지나면 해제한다";
    EXPECT_FALSE(r.active);
    EXPECT_FALSE(r.soft_disabling);

    // 3초 안에 보정이 돌아오면 해제하지 않고 타이머도 처음부터 다시 센다.
    controller.set_calibration_status(CalibrationStatus::Calibrated);
    ASSERT_TRUE(press_set(10.0).engaged);
    controller.set_calibration_status(CalibrationStatus::Invalid);
    for (t = 10.02; t < 11.5; t += 0.01) {
        vehicle = ready_vehicle(t);
        r = controller.update(replay_path(), replay_target(), vehicle, t, 2, panda_allows_control());
    }
    EXPECT_TRUE(r.soft_disabling);
    EXPECT_EQ(r.active_block, BlockReason::CalibrationInvalid);
    controller.set_calibration_status(CalibrationStatus::Calibrated);
    vehicle = ready_vehicle(t);
    r = controller.update(replay_path(), replay_target(), vehicle, t, 2, panda_allows_control());
    EXPECT_TRUE(r.engaged && r.active && !r.soft_disabling);
}

TEST(LateralController, ClipCurvatureReportsAccelLimit) {
    bool limited = true;
    clip_curvature(20.0f, 0.0f, 0.001f, 0.0f, &limited);
    EXPECT_FALSE(limited);
    // 저크 한계만 물면 limited가 아니다(상류 clip_curvature와 같음).
    clip_curvature(20.0f, 0.0f, 0.005f, 0.0f, &limited);
    EXPECT_FALSE(limited);
    // 횡가속 3.0 m/s² 한계(상류 MAX_LATERAL_ACCEL_NO_ROLL): 20 m/s에서 0.0075 1/m
    EXPECT_NEAR(clip_curvature(20.0f, 0.0074f, 0.0085f, 0.0f, &limited), 0.0075f, 1e-7f);
    EXPECT_TRUE(limited);
}

// 상류 get_curvature_from_plan: 0.3 s(MIN_STABLE_DELAY)보다 짧은 지연은 0.3 s 지점의 yaw를 지연 비율만큼
// 줄여 쓰므로, 결과가 0.3 s로 읽은 것과 같다(짧은 지연에서 2·ψ/(v·t)가 잡음을 키우지 않는다).
TEST(LateralController, ShortDelayReadsPlanAtMinStableDelay) {
    LateralTarget target = replay_target();
    for (int i = 0; i < kLateralControlN; ++i) {  // 곡률이 커지는 plan: ψ = 0.002·t²
        target.psis[i] = 0.002f * model_t_idx(i) * model_t_idx(i);
        target.curvatures[i] = 0.0f;
    }
    const float at_min = lag_adjusted_curvature(target, 20.0f, 0.0f, kMinStableDelayS);
    EXPECT_NEAR(lag_adjusted_curvature(target, 20.0f, 0.0f, 0.15f), at_min, 1e-7f);
    EXPECT_NEAR(lag_adjusted_curvature(target, 20.0f, 0.05f, 0.15f), at_min, 1e-7f)
        << "plan 나이를 더해도 0.3 s 미만이면 같다";
    EXPECT_GT(std::fabs(lag_adjusted_curvature(target, 20.0f, 0.0f, 0.5f) - at_min), 1e-6f)
        << "0.3 s 넘으면 그 지연에서 읽는다";
}

// openpilot steerSaturated: 목표 횡가속이 한계에 잘려 0.4초 넘게 포화이고, 실제가 목표의
// 1/1.2에 못 미치는 커브에서만 경고한다.
TEST(LateralController, SteerSaturatedWarnsWhenTurnExceedsLimit) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralTarget target = replay_target();
    for (int i = 0; i < kLateralControlN; ++i) {  // 20 m/s에서 횡가속 8 m/s² 요구
        target.curvatures[i] = 0.02f;
        target.psis[i] = 0.02f * 17.0f * model_t_idx(i);
    }
    // 실제 곡률은 상류처럼 조향각(차량 모델)에서 나온다.
    auto run = [&](float steering_angle_deg) {
        LateralController controller(config);
        LateralControlOutput r;
        bool warned = false;
        for (int tick = 0; tick < 400; ++tick) {
            const double t = 1.0 + 0.01 * tick;
            VehicleCanState vehicle = ready_vehicle(t);
            vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 72.0f;
            vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 72.0f;
            vehicle.cluster_speed_raw = 75.0f;
            vehicle.yaw_rate_valid = true;
            vehicle.steering_angle_deg = steering_angle_deg;
            r = controller.update(replay_path(), target, vehicle, t, tick);
            warned = warned || r.steer_saturated;
        }
        return warned;
    };
    EXPECT_TRUE(run(0.0f)) << "한계에 잘린 커브를 못 따라가면 경고한다";
    // 상류는 잘리기 전 모델 목표(action.desiredCurvature)와 비교한다. 조향각 30도면 20 m/s에서 횡가속이
    // 한계(3.0 m/s²)를 넘어도 plan 목표(8 m/s²)의 절반이라 경고한다.
    EXPECT_TRUE(run(30.0f)) << "잘린 목표가 아니라 plan 목표와 비교한다";
    // 조향각 60도면 plan 목표 가까이 돈다: 목표/실제 < 1.2 → 경고 없음
    EXPECT_FALSE(run(60.0f)) << "plan 목표를 따라가고 있으면 경고하지 않는다";
    EXPECT_FALSE(run(-60.0f)) << "크기로 비교한다(상류와 같음)";
}

// openpilot처럼 운전자가 핸들을 잡아도 요청 토크를 줄이지 않는다. 2026-09-27 고속도로 램프:
// 같은 방향으로 거들자 예전 1초 페이드가 토크를 0으로 만들었다. 운전자와 반대 방향 토크만
// panda와 같은 운전자 클램프가 줄인다.
TEST(LateralController, DriverTorqueDoesNotFadeRequest) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    LateralTarget target = replay_target();
    for (int i = 0; i < kLateralControlN; ++i) {
        target.curvatures[i] = -0.004f;
        target.psis[i] = -0.004f * 17.0f * model_t_idx(i);
    }
    LateralControlOutput r;
    for (int tick = 0; tick < 300; ++tick) {
        const double t = 1.0 + 0.01 * tick;
        VehicleCanState vehicle = ready_vehicle(t);
        vehicle.driver_torque = 300;  // 2초 넘게 170 위
        r = controller.update(replay_path(), target, vehicle, t, tick);
    }
    ASSERT_TRUE(r.active && r.steering_pressed);
    EXPECT_EQ(r.desired_torque, static_cast<int>(std::lround(r.normalized_output * 384.0f)))
        << "요청 토크는 컨트롤러 출력 그대로다";
    EXPECT_GT(std::abs(r.desired_torque), 20) << "커브 요청이 남아 있다";
}

TEST(LateralController, FixedMaxCurvature) {
    LateralControllerConfig config;
    config.force_engaged = true;
    // openpilot 곡률 한계만 본다. 손을 뗀 상태의 조향각 상한(80도)은 HoldAngleCapsOwnSteeringOnly가 본다.
    config.steering_params.avoid_lkas_fault_hold_angle_deg = 0.0f;
    // 1 m/s(MIN_SPEED)에서 조향하도록 최소 조향 속도를 그 아래로 둔다(그 속도 이하는 상류처럼 정차다)
    config.steering_params.min_steer_speed_mps = 0.5f;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 3.6f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 3.6f;
    vehicle.cluster_speed_raw = 3.6f;

    LateralTarget target = replay_target();
    for (int i = 0; i < kLateralControlN; ++i) {
        target.curvatures[i] = 0.5f;
        target.psis[i] = 0.23f;
    }
    // 틱당 변화율 제한이 걸리므로 한 번에 상한까지 뛰지 않는다. 1 m/s에서
    // 창은 5/(1*1)*0.01 = 0.05 1/m 다.
    const auto first = controller.update(replay_path(), target, vehicle, 1.0, 0);
    // 첫 틱은 한계로 뛰지 않고 횡저크 한 스텝만 움직인다
    ASSERT_TRUE(first.active);
    ASSERT_NEAR(first.desired_curvature, 0.05f, 1e-6f);
    LateralControlOutput result = first;
    for (int tick = 1; tick < 40; ++tick)
        result = controller.update(replay_path(), target, vehicle, 1.0 + 0.01 * tick, tick);
    ASSERT_NEAR(result.desired_curvature, 0.2f, 1e-6f) << "최대 곡률은 openpilot의 0.2 1/m 고정";
}

// 라이브 뱅크: 커브(|yaw*v| >= 0.4)에서는 갱신을 멈추고 직선 값을 유지해야 한다.
TEST(LateralController, BankHoldsDuringCurves) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    double t = 1.0;
    auto step = [&](float yaw_rate_rad_s, float lat_accel_mps2) {
        VehicleCanState vehicle = ready_vehicle(t);
        vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
        vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
        vehicle.cluster_speed_raw = 63.0f;
        vehicle.yaw_rate_valid = true;
        vehicle.yaw_rate_rad_s = yaw_rate_rad_s;
        vehicle.lat_accel_valid = true;
        vehicle.lat_accel_mps2 = lat_accel_mps2;
        controller.update(replay_path(), replay_target(), vehicle, t, 0);
        t += 0.01;
    };
    for (int i = 0; i < 1500; ++i) step(0.0f, -0.117f);  // 직선 크라운
    const float straight_bank = controller.road_bank_lat_accel();
    ASSERT_LT(std::fabs(straight_bank + 0.117f), 5e-3f) << "직선에서 뱅크는 노면 경사(크라운)로 수렴한다";
    // 커브: yaw*v = +1.2, 롤 누설 +0.5 (기구학 성분 상쇄 후 잔여)
    const float v = 60.0f / 3.6f;
    for (int i = 0; i < 500; ++i) step(1.2f / v, -1.2f + 0.5f);
    ASSERT_NEAR(controller.road_bank_lat_accel(), straight_bank, 1e-4f)
        << "커브 중에는 뱅크를 유지하고 롤 누설을 따라가지 않는다";
}

TEST(LateralController, RuntimeParamsApplyImmediately) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.cluster_speed_raw = 72.0f;

    const auto active = controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
    ASSERT_TRUE(active.active) << "런타임 파라미터 검사는 활성 상태에서 시작한다";

    SteeringParams steering = config.steering_params;
    steering.enabled = false;
    controller.update_params(steering);
    const auto disabled = controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
    // 런타임 조향 파라미터는 다음 제어 틱에 적용된다
    ASSERT_FALSE(disabled.active);
    ASSERT_EQ(disabled.active_block, BlockReason::ControllerDisabled);

    steering.enabled = true;
    controller.update_params(steering);
    const auto resumed = controller.update(replay_path(), replay_target(), vehicle, 1.02, 2);
    ASSERT_TRUE(resumed.active) << "런타임 파라미터를 바꿔도 컨트롤러 동작이 이어진다";
}

TEST(LateralController, LkasHudStateStability) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

    const auto active = controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
    // 활성 상태의 HUD 프레임
    ASSERT_TRUE(active.active);
    ASSERT_FALSE(active.frames.empty());
    ASSERT_EQ(decode_lkas11(active.frames.front().data).ldws_sys_state, 3);

    // 클러스터는 sys_state 천이마다 부저를 울리므로, sys_state는 active가
    // 아니라 결합(engaged)만 따른다. enable/disable에서만 천이가 생긴다.
    LateralTarget invalid_target = replay_target();
    invalid_target.mpc_solution_valid = false;
    const auto inactive = controller.update(replay_path(), invalid_target, vehicle, 2.5, 2);
    // 비활성 상태의 HUD 프레임
    ASSERT_FALSE(inactive.active);
    ASSERT_FALSE(inactive.frames.empty());
    ASSERT_EQ(decode_lkas11(inactive.frames.front().data).ldws_sys_state, 3)
        << "결합 중 비활성이어도 부저가 울리지 않게 sys_state를 유지한다";
}

TEST(LateralController, PandaGateAndHandoff) {
    LateralControllerConfig config;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

    vehicle.clu_button = 2;
    const auto set_press = controller.update(replay_path(), replay_target(), vehicle, 1.0, 0, panda_allows_control());
    ASSERT_FALSE(set_press.engaged) << "SET은 떼기 전에 결합하지 않는다";

    vehicle.clu_button = 0;
    const auto panda_blocked =
        controller.update(replay_path(), replay_target(), vehicle, 1.01, 1, panda_without_controls());
    // Panda controls 게이트
    ASSERT_TRUE(panda_blocked.engaged);
    ASSERT_FALSE(panda_blocked.active);
    ASSERT_EQ(panda_blocked.active_block, BlockReason::PandaControlsOff);
    ASSERT_FALSE(panda_blocked.engage_rejected) << "Panda controls 핸드셰이크 대기 중에도 유효한 SET을 거부하지 않는다";
    // Panda 불일치 중에도 토크 0 대체 스트림을 계속 보낸다
    ASSERT_TRUE(panda_blocked.should_send);
    ASSERT_FALSE(panda_blocked.frames.empty());
    const HyundaiLkas11Values zero_lkas = decode_lkas11(panda_blocked.frames.front().data);
    // Panda controls가 꺼져 있으면 LKAS 토크 0
    ASSERT_EQ(zero_lkas.steer_torque, 0);
    ASSERT_FALSE(zero_lkas.steer_req);

    LateralController panda_timeout_controller(config);
    // 이 블록은 t=10 에서 돈다. CAN 타임스탬프를 함께 옮기지 않으면 차량 상태가
    // 9초 낡아 vehicle_state_stale 이 되고, 검증하려던 Panda 유예가 아니라
    // freshness 게이트를 보게 된다.
    VehicleCanState timeout_vehicle = ready_vehicle(10.0);
    timeout_vehicle.clu_button = 2;
    panda_timeout_controller.update(replay_path(), replay_target(), timeout_vehicle, 10.0, 0, panda_allows_control());
    timeout_vehicle.clu_button = 0;
    const auto panda_waiting = panda_timeout_controller.update(replay_path(), replay_target(), timeout_vehicle, 10.01,
                                                               1, panda_without_controls());
    // Panda 핸드셰이크 유예 동안 유효한 요청을 붙잡아 둔다
    ASSERT_TRUE(panda_waiting.engaged);
    ASSERT_FALSE(panda_waiting.engage_rejected);
    // 유예(1초)를 넘기며 CAN 은 계속 신선하게 유지한다.
    VehicleCanState timeout_vehicle_later = ready_vehicle(11.0);
    const auto panda_timeout = panda_timeout_controller.update(replay_path(), replay_target(), timeout_vehicle_later,
                                                               11.02, 102, panda_without_controls());
    // Panda 불일치가 이어지면 결국 결합을 거부한다
    ASSERT_FALSE(panda_timeout.engaged);
    ASSERT_TRUE(panda_timeout.engage_rejected);
    ASSERT_EQ(panda_timeout.active_block, BlockReason::PandaControlsOff);

    LateralController deferred_static_controller(config);
    // t=12 시점 검사이므로 차량 데이터도 신선해야 한다(낡으면 stale이 우선).
    VehicleCanState deferred_vehicle = ready_vehicle(12.0);
    deferred_vehicle.wheel_speed_fl_kph = deferred_vehicle.wheel_speed_fr_kph = 60.0f;
    deferred_vehicle.wheel_speed_rl_kph = deferred_vehicle.wheel_speed_rr_kph = 60.0f;
    deferred_vehicle.clu_button = 2;
    deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle, 12.0, 0,
                                      panda_allows_control());
    deferred_vehicle.clu_button = 0;
    deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle, 12.01, 1,
                                      panda_without_controls());
    LateralTarget deferred_invalid_target = replay_target();
    deferred_invalid_target.mpc_solution_valid = false;
    const auto deferred_static = deferred_static_controller.update(replay_path(), deferred_invalid_target,
                                                                   deferred_vehicle, 12.02, 2, panda_allows_control());
    // Panda가 회복되면 정적 결합 게이트를 다시 본다
    ASSERT_FALSE(deferred_static.engaged);
    ASSERT_TRUE(deferred_static.engage_rejected);
    ASSERT_EQ(deferred_static.active_block, BlockReason::LateralPlanInvalid);

    const auto active = controller.update(replay_path(), replay_target(), vehicle, 1.02, 2, panda_allows_control());
    // Panda controls가 켜지면 활성이 된다
    ASSERT_TRUE(active.engaged);
    ASSERT_TRUE(active.active);

    vehicle.clu_button = 4;
    const auto cancel = controller.update(replay_path(), replay_target(), vehicle, 1.03, 3, panda_without_controls());
    // CANCEL을 누르면 토크 0 프레임 인계를 시작한다
    ASSERT_FALSE(cancel.engaged);
    ASSERT_TRUE(cancel.should_send);
    const auto release_tail =
        controller.update(replay_path(), replay_target(), vehicle, 4.02, 302, panda_without_controls());
    ASSERT_TRUE(release_tail.should_send) << "인계는 3000 ms 이어진다";
    const auto stock_handoff =
        controller.update(replay_path(), replay_target(), vehicle, 4.04, 304, panda_without_controls());
    ASSERT_FALSE(stock_handoff.should_send) << "인계는 3000 ms 뒤 멈춘다";

    vehicle.lkas11_seed[4] = 9U << 4;
    stamp_can_times(&vehicle, 4.05);
    vehicle.clu_button = 2;
    controller.update(replay_path(), replay_target(), vehicle, 4.05, 305, panda_allows_control());
    vehicle.clu_button = 0;
    const auto reengaged =
        controller.update(replay_path(), replay_target(), vehicle, 4.06, 306, panda_allows_control());
    // 인계 뒤 다시 결합
    ASSERT_TRUE(reengaged.active);
    ASSERT_FALSE(reengaged.frames.empty());
    ASSERT_EQ(decode_lkas11(reengaged.frames.front().data).msg_count, 10)
        << "다시 결합하면 순정 카메라의 LKAS 카운터에서 이어간다";

    LateralController rejected_controller(config);
    VehicleCanState rejected_vehicle = ready_vehicle();
    rejected_vehicle.clu_button = 2;
    rejected_controller.update(replay_path(), replay_target(), rejected_vehicle, 1.0, 0, panda_allows_control());
    rejected_vehicle.clu_button = 0;
    LateralTarget invalid_target = replay_target();
    invalid_target.mpc_solution_valid = false;
    const auto rejected =
        rejected_controller.update(replay_path(), invalid_target, rejected_vehicle, 1.01, 1, panda_allows_control());
    // 정적 결합 게이트는 결합 상태를 남기지 않고 거부한다
    ASSERT_FALSE(rejected.engaged);
    ASSERT_TRUE(rejected.engage_rejected);
    ASSERT_EQ(rejected.active_block, BlockReason::LateralPlanInvalid);
}

// 시동 직후 첫 결합: Panda health 가 아직 없고 안전벨트/기어가 막고 있을 때.
// 실차(2026-09-12)에서 결합 톤이 울린 뒤 해제되고, 두 번째 시도부터만
// 거절음이 났다. 하드 결함이 panda_not_ready 뒤로 밀려 가려졌기 때문이다.
TEST(LateralController, ColdStartEngageReportsHardBlock) {
    const auto cold_start_attempt = [](bool panda_ready, bool seatbelt_unlatched, int gear) {
        LateralControllerConfig config;
        LateralController controller(config);
        VehicleCanState vehicle = ready_vehicle();
        vehicle.seatbelt_unlatched = seatbelt_unlatched;
        vehicle.gear = gear;
        PandaGateOutput panda;
        panda.ready = panda.controls_allowed = panda_ready;
        vehicle.clu_button = 2;
        controller.update(replay_path(), replay_target(), vehicle, 1.0, 0, panda);
        vehicle.clu_button = 0;
        return controller.update(replay_path(), replay_target(), vehicle, 1.01, 1, panda);
    };

    const auto belt = cold_start_attempt(false, true, 5);
    // 시동 직후 안전벨트 미착용 SET은 panda를 기다리지 않고 안전벨트를 알린다
    ASSERT_FALSE(belt.engaged);
    ASSERT_TRUE(belt.engage_rejected);
    ASSERT_EQ(belt.active_block, BlockReason::SeatbeltUnlatched);

    const auto gear = cold_start_attempt(false, false, 0);
    // 시동 직후 D가 아닌 SET은 panda를 기다리지 않고 기어를 알린다
    ASSERT_FALSE(gear.engaged);
    ASSERT_TRUE(gear.engage_rejected);
    ASSERT_EQ(gear.active_block, BlockReason::GearNotDrive);

    // 차량이 정상이면 Panda 핸드셰이크 유예는 그대로 살아 있어야 한다.
    const auto handshake = cold_start_attempt(false, false, 5);
    // 시동 직후 차가 정상이면 SET은 panda 핸드셰이크를 기다린다
    ASSERT_TRUE(handshake.engaged);
    ASSERT_FALSE(handshake.engage_rejected);
    ASSERT_EQ(handshake.active_block, BlockReason::PandaNotReady);
}

// 2026-09-24 실차: 663 ms 멈춤 뒤 NaN 속도가 좌측 최대 곡률을 심어 재활성 때 32° 조향했다.
TEST(LateralController, StaleSpeedKeepsCurvature) {
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController controller(config);
    const LateralTarget target = replay_target();
    float before = 0.0f;
    for (int i = 0; i < 20; ++i) {
        const double t = 1.0 + 0.01 * i;
        before = controller.update(replay_path(), target, ready_vehicle(t), t, i).desired_curvature;
    }
    const LateralControlOutput stale = controller.update(replay_path(), target, ready_vehicle(1.19), 1.9, 20);
    ASSERT_EQ(stale.active_block, BlockReason::VehicleStateStale) << "휠 속도가 시한보다 오래됐다";
    ASSERT_EQ(stale.desired_curvature, before) << "속도가 낡으면 마지막 목표 곡률을 유지한다";
    const LateralControlOutput back = controller.update(replay_path(), target, ready_vehicle(1.91), 1.91, 21);
    const float step = kMaxLateralJerk / (60.0f / 3.6f * 60.0f / 3.6f) * kCurvatureRateWindowS;
    ASSERT_NEAR(back.desired_curvature, before, step * 1.001f) << "회복하면 멈추기 전 곡률에서 이어간다";
}

// 보드는 JSON을 읽고 재생·테스트는 기본값을 쓴다. 둘이 갈리면 재생 대조가 보드를 대변하지 못한다.
TEST(LateralController, SteeringJsonMatchesDefaults) {
    SteeringParams json, defaults;
    std::string error;
    ASSERT_TRUE(load_steering_params("params/steering.json", &json, &error)) << "steering.json 읽기";
    // 파일에서 읽는 모든 키가 코드 기본값과 같다
    EXPECT_EQ(first_steering_param_difference(json, defaults), "") << "steering.json과 기본값이 다른 키";
}

// 편경사 추정은 직선에서만 갱신하고 커브에서는 유지하며, 30초 넘게 갱신이 없으면 0으로 줄인다. 호출 간격(dt)이
// 달라도(데이터셋 도구는 ControlState 간격) 같은 시정수와 시간 규칙을 따른다.
TEST(RoadBankEstimator, HoldsInCurvesAndFadesWhenStale) {
    for (const float dt : {0.01f, 0.05f}) {
        RoadBankEstimator bank;
        const int per_second = static_cast<int>(std::lround(1.0f / dt));
        for (int i = 0; i < 20 * per_second; ++i) bank.update(dt, 20.0f, 0.2f, true, 0.0f, true);
        EXPECT_NEAR(bank.lat_accel(), 0.2f, 1e-3f) << dt;
        for (int i = 0; i < 10 * per_second; ++i) bank.update(dt, 20.0f, 1.5f, true, 0.05f, true);  // |yaw·v| = 1
        EXPECT_NEAR(bank.lat_accel(), 0.2f, 1e-3f) << "커브에서는 유지 " << dt;
        // 커브에서 갱신하지 않은 10초도 낡은 시간에 든다
        for (int i = 0; i < 19 * per_second; ++i) bank.update(dt, 20.0f, 0.0f, false, 0.0f, true);
        EXPECT_NEAR(bank.lat_accel(), 0.2f, 1e-3f) << "30초 전에는 유지 " << dt;
        for (int i = 0; i < 12 * per_second; ++i) bank.update(dt, 20.0f, 0.0f, false, 0.0f, true);
        EXPECT_LT(bank.lat_accel(), 0.02f) << "30초 넘게 낡으면 0으로 " << dt;
    }
}

// 스스로 가는 최대 조향각은 고장 각도 아래에서 멈춰야 뜻이 있다: 파일이 더 큰 값을 줘도 그 아래로 자른다.
TEST(LateralController, HoldAngleStaysBelowTheFaultAngle) {
    const std::string path = "/tmp/gtest_lateral_controller_hold_angle_" + std::to_string(getpid()) + ".json";
    std::FILE *f = std::fopen(path.c_str(), "w");
    ASSERT_NE(f, nullptr) << "픽스처 쓰기";
    std::fputs("{\"avoid_lkas_fault_hold_angle_deg\": 120}\n", f);
    std::fclose(f);
    SteeringParams params;
    std::string error;
    const bool loaded = load_steering_params(path, &params, &error);
    std::remove(path.c_str());
    ASSERT_TRUE(loaded) << error;
    EXPECT_LT(params.avoid_lkas_fault_hold_angle_deg, params.avoid_lkas_fault_max_angle_deg);
}

// 상류 controlsd: 비활성 중 목표 곡률은 실제 곡률을 따라가고, 재활성 때 거기서 한계 안으로 출발한다.
// 새 계획(촬영 시각이 바뀐 것)이 오면 목표 곡률은 다섯 틱(50 ms) 동안 직전 계획이 내는 값에서 틱마다 1/5씩 새 계획의
// 값으로 간다. 그 뒤와 같은 계획이 이어지는 동안은 섞지 않는다. 기대값은 같은 입력에 한 계획만 받은 컨트롤러 둘(촬영
// 시각이 바뀌지 않아 섞지 않는다)의 같은 틱 값이다. 계획 사이 차이는 곡률 변화율 한계보다 작다.
TEST(LateralController, NewPlanIsBlendedInOverFiveTicks) {
    const auto plan = [](float scale, uint64_t capture_ns) {
        LateralTarget target = replay_target();
        for (int i = 0; i < kLateralControlN; ++i) {
            target.psis[i] *= scale;
            target.curvatures[i] *= scale;
        }
        target.capture_timestamp_ns = capture_ns;
        return target;
    };
    const LateralTarget a = plan(1.0f, 1'000'000'000ULL);
    const LateralTarget b = plan(1.25f, 1'050'000'000ULL);
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController blended(config), only_a(config), only_b(config);
    double t = 1.0;
    for (int frame = 0; frame < 110; ++frame, t += 0.01) {
        const VehicleCanState vehicle = ready_vehicle(t);
        const bool switched = frame >= 100;
        const LateralControlOutput result = blended.update(replay_path(), switched ? b : a, vehicle, t, frame);
        const float from = only_a.update(replay_path(), a, vehicle, t, frame).desired_curvature;
        const float to = only_b.update(replay_path(), b, vehicle, t, frame).desired_curvature;
        if (switched) {
            ASSERT_TRUE(result.lat_active);
            ASSERT_GT(std::fabs(to - from), 1e-4f) << "두 계획의 목표 곡률이 다르다";
        }
        const float weight = switched ? std::min(1.0f, static_cast<float>(frame - 99) / 5.0f) : 0.0f;
        EXPECT_NEAR(result.desired_curvature, from + weight * (to - from), 1e-6f) << "frame " << frame;
    }
}

// MPC가 무효인 계획은 조향을 막는다(LateralPlanInvalid). 다음 계획으로 조향을 다시 시작할 때 그 계획에서 섞어 오지
// 않는다: 무효 계획의 곡률만 다른 두 컨트롤러의 출력이 같다. 한쪽 무효 계획은 반대쪽으로 굽어, 섞으면 곡률 변화율
// 한계 안에서도 다시 시작하는 방향이 갈린다.
TEST(LateralController, InvalidMpcPlanIsNotBlendedFrom) {
    const auto plan = [](float scale, uint64_t capture_ns, bool mpc_valid) {
        LateralTarget target = replay_target();
        for (int i = 0; i < kLateralControlN; ++i) {
            target.psis[i] *= scale;
            target.curvatures[i] *= scale;
        }
        target.capture_timestamp_ns = capture_ns;
        target.mpc_solution_valid = mpc_valid;
        return target;
    };
    const LateralTarget before = plan(1.0f, 1'000'000'000ULL, true);
    const LateralTarget invalid_a = plan(1.0f, 1'050'000'000ULL, false);
    const LateralTarget invalid_b = plan(-12.5f, 1'050'000'000ULL, false);
    const LateralTarget after = plan(1.25f, 1'100'000'000ULL, true);
    LateralControllerConfig config;
    config.force_engaged = true;
    LateralController a(config), b(config);
    double t = 1.0;
    int active_after = 0;
    for (int frame = 0; frame < 120; ++frame, t += 0.01) {
        const VehicleCanState vehicle = ready_vehicle(t);
        const bool invalid = frame >= 100 && frame < 105;
        const LateralTarget &first = frame < 100 ? before : invalid ? invalid_a : after;
        const LateralTarget &second = frame < 100 ? before : invalid ? invalid_b : after;
        const LateralControlOutput ra = a.update(replay_path(), first, vehicle, t, frame);
        const LateralControlOutput rb = b.update(replay_path(), second, vehicle, t, frame);
        if (invalid) ASSERT_FALSE(ra.lat_active);
        if (frame >= 105 && ra.lat_active) ++active_after;
        ASSERT_NEAR(ra.desired_curvature, rb.desired_curvature, 1e-7f) << "frame " << frame;
    }
    EXPECT_GE(active_after, 10) << "조향을 다시 시작한다";
}

TEST(LateralController, InactiveDesiredTracksActual) {
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.angle_offset_deg = 0.0f;
    LateralController controller(config);
    const float v = 60.0f / 3.6f;
    const float step = kMaxLateralJerk / (v * v) * kCurvatureRateWindowS;
    LateralPath blocked = replay_path();
    blocked.usable_for_steering = false;
    LateralControlOutput r;
    int tick = 0;
    const auto step_once = [&](const LateralPath &path) {
        const double t = 1.0 + 0.01 * tick;
        VehicleCanState vehicle = ready_vehicle(t);
        vehicle.steering_angle_deg = 10.0f;
        r = controller.update(path, replay_target(), vehicle, t, tick++);
    };
    for (int i = 0; i < 100; ++i) step_once(blocked);
    // 비활성 동안 목표 곡률은 실제 곡률에 자리 잡는다
    ASSERT_FALSE(r.active);
    ASSERT_NEAR(r.desired_curvature, r.actual_curvature, 1e-6f);
    float previous = r.desired_curvature;
    while (!r.active && tick < 300) {
        previous = r.desired_curvature;
        step_once(replay_path());
    }
    ASSERT_TRUE(r.active) << "경로 디바운스가 풀린다";
    // 다시 활성이 되면 plan이 아니라 실제 곡률에서 출발한다
    ASSERT_NEAR(r.desired_curvature, previous, step * 1.001f);
    ASSERT_GT(std::fabs(r.desired_curvature - replay_target().curvatures[0]), 10.0f * step);
}

// 운전자가 85도 넘게 돌려 넘겨받은 회전(steering pressed)에서는 요청을 끈 뒤 핸들이 15도 아래로
// 오고 손을 뗄 때까지 끈 채로 둔다(carrotpilot 해제 조건). 빠져나오며 펴는 핸들을 밀지 않는다.
// 운전자가 손대지 않은 체류는 85도 아래로 오면 바로 다시 켠다(LargeAngleFaultAvoidance).
TEST(LateralController, DriverTakeoverHoldsUntilCentered) {
    LateralControllerConfig config;
    config.force_engaged = true;
    const SteeringParams &sp = config.steering_params;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    int frame = 0;
    auto step = [&](float angle, int driver_torque) {
        vehicle.steering_angle_deg = angle;
        vehicle.driver_torque = driver_torque;
        const int f = frame++;
        stamp_can_times(&vehicle, 1.0 + f * 0.01);
        return controller.update(replay_path(), replay_target(), vehicle, 1.0 + f * 0.01, f);
    };
    auto steer_req = [](const LateralControlOutput &r) {
        EXPECT_FALSE(r.frames.empty()) << "LKAS11을 보낸다";
        return !r.frames.empty() && decode_lkas11(r.frames.front().data).steer_req;
    };

    LateralControlOutput result;
    for (int i = 0; i < 100; ++i) result = step(20.0f, 0);
    ASSERT_TRUE(steer_req(result));
    for (int i = 0; i < sp.avoid_lkas_fault_max_frames + 20; ++i) result = step(150.0f, 300);
    // 운전자가 150도까지 감아 85도 위에 머물면 89번째 프레임에 요청을 끈다
    ASSERT_TRUE(result.large_angle_hold);
    ASSERT_TRUE(result.large_angle_hold_by_driver) << "HUD: 15도 아래에서 손을 떼야 다시 조향";
    ASSERT_FALSE(steer_req(result));
    for (int i = 0; i < 50; ++i) ASSERT_FALSE(steer_req(step(60.0f, 300))) << "잡고 펴는 중 " << i;
    for (int i = 0; i < 50; ++i) ASSERT_FALSE(steer_req(step(30.0f, 0))) << "손을 뗐지만 15도 위 " << i;
    for (int i = 0; i < 20; ++i) ASSERT_FALSE(steer_req(step(30.0f, 300))) << "다시 잡음 " << i;
    for (int i = 0; i < 50; ++i) ASSERT_FALSE(steer_req(step(10.0f, 300))) << "15도 아래지만 잡고 있음 " << i;
    int released = -1;
    for (int i = 0; i < 20 && released < 0; ++i) {
        result = step(10.0f, 0);
        if (steer_req(result)) released = i;
    }
    // 15도 아래에서 손을 떼면(조향 감지 디바운스 뒤) 다시 켜고 토크는 0부터 올린다
    ASSERT_GE(released, 0);
    ASSERT_TRUE(result.active);
    ASSERT_FALSE(result.large_angle_hold);
    ASSERT_FALSE(result.large_angle_hold_by_driver);
    ASSERT_LE(std::abs(result.apply_torque), kHyundaiSteerDeltaUp);
}

// 운전자가 85도를 잠깐 넘겨 넘겨받았어도 커브를 붙잡고 있으면(램프): 핸들이 80도 아래이고 계획이 지금 핸들과 같은
// 커브(곡률 ±25%)를 1초 가리키면 다시 조향하고, 토크는 0부터 커브 쪽으로 올린다. 80도 위나 계획이 직진에 가까우면
// (교차로 탈출) 핸들을 잡고 있어도 15도 규칙대로 쉰다.
TEST(LateralController, DriverHoldingThePlannedCurveResumes) {
    LateralControllerConfig config;
    config.force_engaged = true;
    const SteeringParams &sp = config.steering_params;
    constexpr float kSpeedKph = 35.0f;
    // 오른쪽 70도가 내는 곡률을 그대로 이어 가는 계획(곡률 일정: psi = k·v·t)
    const float held_curvature = VehicleModel::make(sp).curvature_at_angle(kSpeedKph / 3.6f, -70.0f);
    ASSERT_GT(held_curvature, 0.0f) << "오른쪽 = 양의 곡률(제어 부호)";
    const auto curve_target = [](float curvature) {
        LateralTarget target;
        target.valid = true;
        target.mpc_solution_valid = true;
        for (int i = 0; i < kLateralControlN; ++i) {
            target.curvatures[i] = curvature;
            target.psis[i] = curvature * (kSpeedKph / 3.6f) * model_t_idx(i);
        }
        return target;
    };
    auto run = [&](float plan_curvature, float held_angle) {
        LateralController controller(config);
        VehicleCanState vehicle = ready_vehicle();
        vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = kSpeedKph;
        vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = kSpeedKph;
        vehicle.cluster_speed_raw = kSpeedKph + 2.0f;
        const LateralTarget target = curve_target(plan_curvature);
        int frame = 0;
        auto step = [&](float angle, int driver_torque) {
            vehicle.steering_angle_deg = angle;
            vehicle.driver_torque = driver_torque;
            const int f = frame++;
            stamp_can_times(&vehicle, 1.0 + f * 0.01);
            return controller.update(replay_path(), target, vehicle, 1.0 + f * 0.01, f);
        };
        LateralControlOutput result;
        for (int i = 0; i < 200; ++i) result = step(-20.0f, 0);
        EXPECT_TRUE(result.lat_active);
        // 운전자가 95도까지 감아 85도 위에 머물면 요청을 끄고 운전자가 넘겨받은 정지가 된다
        for (int i = 0; i < sp.avoid_lkas_fault_max_frames + 20; ++i) result = step(-95.0f, -300);
        EXPECT_TRUE(result.large_angle_hold_by_driver);
        // 82도: 85도 아래지만 다시 켜는 80도 위
        for (int i = 0; i < 100; ++i) {
            result = step(-82.0f, -300);
            EXPECT_TRUE(result.large_angle_hold) << "82도 " << i;
        }
        int resumed = -1;
        LateralControlOutput first;
        for (int i = 0; i < 300 && resumed < 0; ++i) {
            first = step(held_angle, -300);
            if (!first.large_angle_hold) resumed = i;
        }
        for (int i = 0; i < 50; ++i) result = step(held_angle, -300);
        return std::make_pair(resumed, std::make_pair(first, result));
    };

    const auto [resumed, results] = run(held_curvature, -70.0f);
    const auto &[first, later] = results;
    EXPECT_EQ(resumed, 99) << "80도 아래에서 계획이 맞은 지 1초(100프레임)째";
    EXPECT_TRUE(first.lat_active);
    EXPECT_FALSE(first.large_angle_hold_by_driver);
    EXPECT_LE(std::abs(first.apply_torque), kHyundaiSteerDeltaUp) << "토크는 0부터 올린다";
    EXPECT_LT(later.apply_torque, -50) << "0.5초 뒤 커브(오른쪽, 각도와 같은 음의 토크)를 돕는다";
    EXPECT_FALSE(later.large_angle_hold);
    EXPECT_EQ(run(held_curvature * 0.6f, -70.0f).first, -1) << "계획이 커브보다 훨씬 완만하면 쉰다";
    EXPECT_EQ(run(-held_curvature, -70.0f).first, -1) << "계획이 반대쪽이면 쉰다";
    EXPECT_EQ(run(held_curvature, -50.0f).first, -1) << "핸들이 계획보다 많이 풀렸으면(계획이 1.4배) 쉰다";
}

// 운전자가 핸들을 잡지 않으면 컨트롤러 목표를 avoid_lkas_fault_hold_angle_deg(80도) 조향각이 내는
// 곡률 안으로 묶는다. 85도를 넘겨 토크가 끊겼다 다시 잡는 반복 대신 그 각도에서 버틴다. 운전자가
// 조향 중이거나 0으로 끄면 묶지 않는다.
TEST(LateralController, HoldAngleCapsOwnSteeringOnly) {
    LateralControllerConfig config;
    config.force_engaged = true;
    const float v = 20.0f / 3.6f;
    auto run = [&](const LateralControllerConfig &cfg, float plan_curvature, int driver_torque) {
        LateralTarget tight = replay_target();  // 20 km/h 교차로 회전: plan은 반경 10 m를 원한다
        for (int i = 0; i < kLateralControlN; ++i) {
            tight.curvatures[i] = plan_curvature;
            tight.psis[i] = plan_curvature * v * model_t_idx(i);
        }
        LateralController controller(cfg);
        VehicleCanState vehicle = ready_vehicle();
        vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 20.0f;
        vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 20.0f;
        vehicle.cluster_speed_raw = 21.0f;
        vehicle.steering_angle_deg = 30.0f;
        vehicle.driver_torque = driver_torque;
        LateralControlOutput r;
        for (int f = 0; f < 300; ++f) {
            stamp_can_times(&vehicle, 1.0 + f * 0.01);
            r = controller.update(replay_path(), tight, vehicle, 1.0 + f * 0.01, f);
        }
        EXPECT_TRUE(r.active);
        return r.desired_curvature;
    };
    const VehicleModel model = VehicleModel::make(config.steering_params);
    const float right_cap = model.curvature_at_angle(v, -80.0f);
    const float left_cap = model.curvature_at_angle(v, 80.0f);
    // 80도가 내는 곡률은 반경 30 m 남짓이다(오른쪽 +, 왼쪽 −)
    ASSERT_GT(right_cap, 0.02f);
    ASSERT_LT(right_cap, 0.05f);
    ASSERT_LT(left_cap, -0.02f);
    EXPECT_NEAR(run(config, 0.1f, 0), right_cap, 1e-3f) << "혼자서는 80도까지만 요청한다(오른쪽)";
    EXPECT_NEAR(run(config, -0.1f, 0), left_cap, 1e-3f) << "왼쪽도 같다";
    EXPECT_GT(run(config, 0.1f, 300), 2.0f * right_cap) << "운전자가 조향 중이면 plan을 따른다";
    LateralControllerConfig off = config;
    off.steering_params.avoid_lkas_fault_hold_angle_deg = 0.0f;
    EXPECT_GT(run(off, 0.1f, 0), 2.0f * right_cap) << "0이면 끈다";
    EXPECT_NEAR(run(config, 0.01f, 0), 0.01f, 1e-3f) << "상한 안의 요청은 그대로다";
}

// ---------------------------------------------------------------- paramsd·torqued·lagd 소비

LiveLateralParams odd_live_params() {
    LiveLateralParams live;
    live.use_vehicle = true;
    live.steer_ratio = 13.1f;
    live.stiffness_factor = 0.7f;
    live.angle_offset_deg = 2.5f;
    live.roll_rad = 0.04f;
    live.use_torque = true;
    live.lat_accel_factor = 3.0f;
    live.lat_accel_offset = 0.2f;
    live.friction = 0.05f;
    return live;
}

// 스위치를 끄면 학습값을 넣어도 모든 틱이 비트 단위로 같아야 한다. 무효·캘리브 완료도
// 끈 쪽에서는 차단하지 않는다.
TEST(LateralController, LiveParamsSwitchOffIsIdentical) {
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.use_live_vehicle_params = false;
    config.steering_params.use_live_torque_params = false;
    LateralController plain(config), fed(config);
    double t = 1.0;
    for (int tick = 0; tick < 600; ++tick, t += 0.01) {
        VehicleCanState vehicle = ready_vehicle(t);
        vehicle.steering_angle_deg = 12.0f * std::sin(0.02f * tick);
        vehicle.yaw_rate_valid = true;
        vehicle.yaw_rate_rad_s = 0.05f * std::sin(0.02f * tick);
        vehicle.lat_accel_valid = true;
        vehicle.lat_accel_mps2 = -0.3f;
        vehicle.driver_torque = tick % 97 == 0 ? 200 : 10;
        LateralTarget target = replay_target();
        for (int i = 0; i < kLateralControlN; ++i) target.curvatures[i] = 0.002f * std::cos(0.01f * tick);
        fed.set_live_params(odd_live_params(), false);
        const LateralControlOutput a = plain.update(replay_path(), target, vehicle, t, tick);
        const LateralControlOutput b = fed.update(replay_path(), target, vehicle, t, tick);
        // 학습기를 끄면 막는 일이 없다
        ASSERT_EQ(a.active_block, b.active_block);
        ASSERT_EQ(a.active, b.active);
        // 학습기를 끄면 모든 틱이 비트까지 같다
        ASSERT_EQ(std::memcmp(&a.desired_curvature, &b.desired_curvature, sizeof(float)), 0);
        ASSERT_EQ(std::memcmp(&a.actual_curvature, &b.actual_curvature, sizeof(float)), 0);
        ASSERT_EQ(std::memcmp(&a.normalized_output, &b.normalized_output, sizeof(float)), 0);
        ASSERT_EQ(std::memcmp(&a.feedforward, &b.feedforward, sizeof(float)), 0);
        ASSERT_EQ(a.desired_torque, b.desired_torque);
        ASSERT_EQ(a.apply_torque, b.apply_torque);
    }
}

TEST(LateralController, ParamsdInvalidBlocksOnlyWhenUsed) {
    LateralControllerConfig config;
    config.force_engaged = true;
    config.steering_params.use_live_vehicle_params = true;
    LateralController controller(config);
    const VehicleCanState vehicle = ready_vehicle(1.0);
    controller.set_live_params(odd_live_params(), true);
    ASSERT_TRUE(controller.update(replay_path(), replay_target(), vehicle, 1.0, 0).active)
        << "유효한 학습값은 제어를 활성으로 둔다";
    controller.set_live_params(odd_live_params(), false);
    controller.set_calibration_status(CalibrationStatus::Uncalibrated);
    ASSERT_EQ(controller.update(replay_path(), replay_target(), vehicle, 1.01, 1).active_block,
              BlockReason::CalibrationIncomplete)
        << "보정 전에는 보정 사유가 먼저다(상류 cal_status 검사): 무효 학습값으로 막지 않는다";
    controller.set_calibration_status(CalibrationStatus::Calibrated);
    const LateralControlOutput invalid = controller.update(replay_path(), replay_target(), vehicle, 1.02, 2);
    ASSERT_EQ(invalid.active_block, BlockReason::ParamsdInvalid) << "보정 뒤 무효 학습값은 막는다";
    // 상류 paramsdTemporaryError처럼 SOFT_DISABLE: 결합 중이면 경고하며 3초 더 조향한 뒤 해제한다
    EXPECT_EQ(block_kind(BlockReason::ParamsdInvalid), BlockKind::SoftDisable);
    EXPECT_TRUE(invalid.soft_disabling);
    EXPECT_TRUE(invalid.active);
    LiveLateralParams unseen = odd_live_params();
    unseen.use_vehicle = false;
    controller.set_live_params(unseen, false);
    ASSERT_NE(controller.update(replay_path(), replay_target(), vehicle, 1.03, 3).active_block,
              BlockReason::ParamsdInvalid)
        << "paramsd가 발행하기 전에는 막지 않는다(상류 sm.seen)";
}

TEST(LateralController, CurvatureLimitFollowsRoll) {
    const float v = 20.0f, roll = 0.03f;
    const float up = clip_curvature(v, 0.05f, 0.05f, roll);
    ASSERT_NEAR(up, (kMaxLateralAccel + roll * 9.81f) / (v * v), 1e-7f) << "롤만큼 횡가속도 상한이 roll*g 움직인다";
    const float down = clip_curvature(v, -0.05f, -0.05f, roll);
    ASSERT_NEAR(down, (-kMaxLateralAccel + roll * 9.81f) / (v * v), 1e-7f) << "하한도 같은 roll*g만큼 움직인다";
}

// lagd 지연: 스위치를 켜고 확정된 값이 있을 때만 경로 지연을 바꾼다(범위 0.15~0.65 s).
TEST(LateralController, LiveDelaySwitch) {
    LateralControllerConfig config;
    config.steering_params.steer_actuator_delay = 0.34f;
    LateralController controller(config);
    controller.set_live_delay(0.44f, true);
    EXPECT_FALSE(controller.live_delay_in_use()) << "스위치가 꺼져 있으면 쓰지 않는다";
    EXPECT_FLOAT_EQ(controller.plan_delay_s(), 0.34f);

    config.steering_params.use_live_delay = true;
    LateralController on(config);
    EXPECT_FLOAT_EQ(on.plan_delay_s(), 0.34f) << "추정이 오기 전에는 수동값";
    on.set_live_delay(0.44f, true);
    EXPECT_TRUE(on.live_delay_in_use());
    EXPECT_FLOAT_EQ(on.plan_delay_s(), 0.44f);
    on.set_live_delay(0.44f, false);
    EXPECT_FLOAT_EQ(on.plan_delay_s(), 0.34f) << "확정 전·오래된 값은 쓰지 않는다";
}

// 결합한 컨트롤러와 그 차를 함께 들고 틱을 진행하는 작은 도구(입력 끊김·해제 예고 시험용)
struct EngagedDrive {
    LateralControllerConfig config;
    LateralController controller;
    VehicleCanState vehicle = ready_vehicle(1.0);
    double t = 1.0;
    int frame = 0;

    explicit EngagedDrive(LateralControllerConfig c = {}) : config(c), controller(config) {}
    LateralControlOutput tick(const LateralPath &path, const LateralTarget &target, bool panda_allowed = true,
                              bool fresh_can = true) {
        t += 0.01;
        if (fresh_can) {
            const int button = vehicle.clu_button;
            const float wheel = vehicle.wheel_speed_fl_kph;
            vehicle = ready_vehicle(t);
            vehicle.clu_button = button;
            vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = wheel;
            vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = wheel;
        }
        PandaGateOutput panda = panda_allows_control();
        panda.controls_allowed = panda_allowed;
        return controller.update(path, target, vehicle, t, frame++, panda);
    }
    LateralControlOutput engage() {
        vehicle.clu_button = 2;
        tick(replay_path(), replay_target());
        vehicle.clu_button = 0;
        return tick(replay_path(), replay_target());
    }
};

LateralPath lost_model_path() {
    LateralPath path;
    path.model_lost = true;
    return path;
}

// 결합 중 모델이 끊기면 바로 조향을 쉬고(토크 0) 해제 예고를 띄운다. 3초 안에 돌아오면 이어서 조향하고, 넘으면
// 해제한다. 예전에는 경고 없이 쉬다가 모델이 돌아오면 운전자 확인 없이 다시 조향했다(상류는 commIssue로 해제).
TEST(LateralController, LostModelWarnsThenDisengages) {
    EngagedDrive drive;
    ASSERT_TRUE(drive.engage().engaged);
    for (int i = 0; i < 100; ++i) drive.tick(replay_path(), replay_target());
    LateralControlOutput r = drive.tick(lost_model_path(), replay_target());
    EXPECT_EQ(r.active_block, BlockReason::ModelLost);
    EXPECT_TRUE(r.engaged);
    EXPECT_FALSE(r.active) << "낡은 모델로는 조향하지 않는다";
    EXPECT_EQ(r.apply_torque, 0);
    EXPECT_FALSE(r.soft_disabling) << "짧은 끊김에는 경고하지 않는다";
    for (int i = 0; i < 40; ++i) r = drive.tick(lost_model_path(), replay_target());
    EXPECT_FALSE(r.soft_disabling) << "0.4초";
    for (int i = 0; i < 15; ++i) r = drive.tick(lost_model_path(), replay_target());
    EXPECT_TRUE(r.soft_disabling) << "0.5초가 넘으면 해제 예고를 띄운다";
    // 2초 안에 돌아오면 해제하지 않고, 경로 복귀 디바운스(0.5초) 뒤 다시 조향한다
    for (int i = 0; i < 150; ++i) r = drive.tick(lost_model_path(), replay_target());
    ASSERT_TRUE(r.engaged && r.soft_disabling);
    for (int i = 0; i < 60; ++i) r = drive.tick(replay_path(), replay_target());
    EXPECT_TRUE(r.engaged && r.active);
    EXPECT_FALSE(r.soft_disabling);
    // 3초를 넘기면 해제한다
    const double lost_at = drive.t;
    for (const double end = drive.t + kInterruptDisengageS - 0.05; drive.t < end;) {
        r = drive.tick(lost_model_path(), replay_target());
        const double lost = drive.t - lost_at - 0.01;  // 첫 끊김 틱부터
        ASSERT_TRUE(r.engaged) << lost;
        if (std::fabs(lost - kInterruptAlertS) > 0.015) ASSERT_EQ(r.soft_disabling, lost > kInterruptAlertS) << lost;
    }
    for (int i = 0; i < 10; ++i) r = drive.tick(lost_model_path(), replay_target());
    EXPECT_FALSE(r.engaged) << "3초가 지나면 해제한다";
    EXPECT_FALSE(r.soft_disabling);
    // 모델이 돌아와도 다시 결합하지 않는다
    for (int i = 0; i < 100; ++i) r = drive.tick(replay_path(), replay_target());
    EXPECT_FALSE(r.engaged || r.active);
}

// 차량 CAN이 끊겨도(차량 상태가 낡음) 같다. 정차 중 모델이 끊겨도 정차 대기가 아니라 모델 끊김이다.
TEST(LateralController, StaleCarStateWarnsThenDisengages) {
    EngagedDrive drive;
    ASSERT_TRUE(drive.engage().engaged);
    // 마지막 CAN을 0.3초 전 것으로 돌려 0.2초 뒤에 kVehicleStateTimeoutS(0.5초)를 넘긴다
    stamp_can_times(&drive.vehicle, drive.t - 0.3);
    LateralControlOutput r;
    for (int i = 0; i < 80; ++i) r = drive.tick(replay_path(), replay_target(), true, false);
    EXPECT_EQ(r.active_block, BlockReason::VehicleStateStale);
    EXPECT_TRUE(r.engaged && r.soft_disabling && !r.active);
    for (const double end = drive.t + kInterruptDisengageS; drive.t < end;)
        r = drive.tick(replay_path(), replay_target(), true, false);
    EXPECT_FALSE(r.engaged);

    EngagedDrive stopped;
    ASSERT_TRUE(stopped.engage().engaged);
    stopped.vehicle.wheel_speed_fl_kph = 0.0f;
    r = stopped.tick(lost_model_path(), replay_target());
    EXPECT_EQ(r.active_block, BlockReason::ModelLost);
}

// 결합 중 Panda가 허가를 거두면(재연결·safety 재설정) 해제 예고 뒤 해제한다. 상류는 불일치 2초 뒤 해제한다.
// 예전에는 계기판에 작동 표시를 낸 채 무기한 쉬다가 RES/SET에 말없이 다시 조향했다.
TEST(LateralController, PandaRevokeWhileEngagedDisengages) {
    EngagedDrive drive;
    ASSERT_TRUE(drive.engage().engaged);
    for (int i = 0; i < 50; ++i) drive.tick(replay_path(), replay_target());
    LateralControlOutput r = drive.tick(replay_path(), replay_target(), false);
    EXPECT_EQ(r.active_block, BlockReason::PandaControlsOff);
    EXPECT_TRUE(r.engaged && !r.active);
    // 잠깐(0.5초 안) 끊겼다 돌아오면 경고 없이 그대로 간다
    for (int i = 0; i < 40; ++i) {
        r = drive.tick(replay_path(), replay_target(), false);
        ASSERT_FALSE(r.soft_disabling);
    }
    r = drive.tick(replay_path(), replay_target());
    EXPECT_TRUE(r.engaged && r.active && !r.soft_disabling);
    for (const double end = drive.t + kInterruptDisengageS + 0.02; drive.t < end;)
        r = drive.tick(replay_path(), replay_target(), false);
    EXPECT_FALSE(r.engaged) << "허가가 3초 넘게 없으면 해제한다";
}

// 해제 예고 타이머는 SoftDisable 사유가 이어지는 동안 앞선 다른 사유(경로 무효, 정차)가 끼어도 이어 센다
// (상류 softDisabling). 예전에는 다른 사유가 한 틱만 끼어도 처음부터 다시 세어 해제가 끝나지 않았다.
TEST(LateralController, SoftDisableTimerKeepsCountingThroughOtherReasons) {
    EngagedDrive drive;
    ASSERT_TRUE(drive.engage().engaged);
    drive.controller.set_calibration_status(CalibrationStatus::Invalid);
    LateralPath short_path = replay_path();
    short_path.usable_for_steering = false;
    LateralControlOutput r;
    const double start = drive.t;
    while (drive.t < start + kSoftDisableS + 0.1) {
        // 0.5초마다 한 틱 경로가 무효다
        const bool blip = static_cast<int>((drive.t - start) * 100.0 + 0.5) % 50 == 25;
        r = drive.tick(blip ? short_path : replay_path(), replay_target());
        if (!r.engaged) break;
        ASSERT_TRUE(r.soft_disabling) << drive.t - start;
    }
    EXPECT_FALSE(r.engaged) << "끼어든 경로 무효와 상관없이 3초 뒤 해제한다";
    EXPECT_NEAR(drive.t - start, kSoftDisableS, 0.05);
}

}  // namespace
