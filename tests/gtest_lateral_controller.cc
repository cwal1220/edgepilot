/* 횡제어기(LateralController): engage 게이트와 홀드, 토크 한계와 MDPS 고장 회피, 곡률 제한, Panda
 * 게이트와 넘겨받기, LKAS HUD, 그리고 학습값(paramsd·torqued)과 lagd 지연 소비. CanFixture는 녹화 CAN
 * 픽스처를 컨트롤러에 흘려 openpilot 참조식과 대조하며, 픽스처를 인자로 줄 때만 돈다. CAN 신호는
 * gtest_car, 토크 제어기는 gtest_lateral_torque, 홀드는 gtest_control_holds, 플래너는
 * gtest_lateral_planner가 본다. */
#include "controls/control_params.h"
#include "car/hyundai_can.h"
#include "controls/lateral_controller.h"
#include "controls/lateral_path.h"
#include "controls/lateral_torque.h"
#include "common/model_output.h"
#include "car/vehicle_can.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace {

/* 컨트롤러가 보는 수신 메시지 시각을 한 번에 t로 맞춘다. */
void stamp_can_times(VehicleCanState *vehicle, double t) {
  vehicle->lkas11_time_s = t;
  vehicle->clu11_time_s = t;
  vehicle->sas11_time_s = t;
  vehicle->esp12_time_s = t;
  vehicle->mdps12_time_s = t;
  vehicle->tcs13_time_s = t;
  vehicle->tcs15_time_s = t;
  vehicle->e_ems11_time_s = t;
  vehicle->elect_gear_time_s = t;
  vehicle->whl_spd11_time_s = t;
  vehicle->cgw1_time_s = t;
  vehicle->cgw2_time_s = t;
}

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
  path.left_valid = true;
  path.right_valid = true;
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

/* lag 보상 곡률과 clip_curvature의 독립 전사본. 한계값은 lateral_controller.h에서 그대로
 * 가져온다 — 숫자를 복제하면 구현이 바뀔 때 이 검증이 조용히 썩는다.
 * 픽스처 target은 capture_timestamp_ns=0이라 plan 나이 보정은 0이다. */
float reference_plan_curvature(const LateralTarget &target, float speed_mps,
                               float actuator_delay) {
  const float delay = std::max(0.01f, actuator_delay);
  float psi = target.psis[kLateralControlN - 1];
  if (delay <= 0.0f) {
    psi = target.psis[0];
  } else {
    for (int i = 1; i < kLateralControlN; ++i) {
      if (delay <= model_t_idx(i)) {
        const float p = (delay - model_t_idx(i - 1)) /
                        (model_t_idx(i) - model_t_idx(i - 1));
        psi = target.psis[i - 1] + p * (target.psis[i] - target.psis[i - 1]);
        break;
      }
    }
  }
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  const float current = target.curvatures[0];
  return current + 2.0f * (psi / (speed * delay) - current);
}

float reference_clip_curvature(float speed_mps, float prev_curvature, float desired) {
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  const float rate_limit = kMaxLateralJerk / (speed * speed);
  desired = std::clamp(desired,
                       prev_curvature - rate_limit * kCurvatureRateWindowS,
                       prev_curvature + rate_limit * kCurvatureRateWindowS);
  const float accel_speed = std::max(speed, 1.0f);
  desired = std::clamp(desired,
                       -kMaxLateralAccel / (accel_speed * accel_speed),
                       kMaxLateralAccel / (accel_speed * accel_speed));
  return std::clamp(desired, -kMaxCurvature, kMaxCurvature);
}

TEST(LateralController, BrakingDoesNotDisengage) {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음
  vehicle.brake_light = true;

  const auto brake_light_result =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  ASSERT_TRUE(brake_light_result.active) << "DriverBraking 없는 브레이크등만으로는 활성을 유지한다";

  vehicle.brake_pressed = true;
  const auto brake_pressed_result =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  ASSERT_TRUE(brake_pressed_result.active) << "DriverBraking도 횡제어를 disengage하지 않는다";
}

/* K7 MDPS는 steer 요청이 켜진 채 85도 위에 0.98초 넘게 머물면 토크와 무관하게 fault를 낸다
 * (2026-09-18 실측). 85도 위에서도 컨트롤러 토크는 그대로 내되, 크기를 steer_delta_down × 남은
 * 프레임으로 묶어 89번째 프레임에 0에 닿게 하고, 그때부터 85도 아래로 돌아올 때까지 steer 요청을
 * ToiFlt 없이 끈다. 2프레임 컷처럼 85도 위에서 요청을 다시 켜지 않는다(2026-10-03: 다시 켜는
 * 순간 고장). 85도 아래로 오면 요청을 켜고 토크를 0부터 올린다. */
TEST(LateralController, LargeAngleFaultAvoidance) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
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
  auto lkas = [](const LateralControlResult &r) {
    EXPECT_FALSE(r.frames.empty()) << "LKAS11을 보낸다";
    return r.frames.empty() ? HyundaiLkas11Values{} : decode_lkas11(r.frames.front().data);
  };

  vehicle.steering_angle_deg = 20.0f;
  LateralControlResult result;
  for (int i = 0; i < 100; ++i) result = step();
  // MDPS 고장 각도 아래에서는 토크가 나간다
  ASSERT_TRUE(result.active);
  ASSERT_NE(result.apply_torque, 0);

  vehicle.steering_angle_deg = 100.0f;
  int full_until = 0;
  for (int since = 1; since < sp.avoid_lkas_fault_max_frames; ++since) {
    result = step();
    const int cap = sp.steer_delta_down * (sp.avoid_lkas_fault_max_frames - since);
    // 요청을 끄는 프레임 전까지는 steer 요청을 유지하고, 토크는 그때 0에 닿을 만큼만 남긴다
    ASSERT_TRUE(result.active);
    ASSERT_FALSE(result.large_angle_hold) << since;
    ASSERT_TRUE(lkas(result).steer_req) << since;
    ASSERT_LE(std::abs(result.desired_torque), cap) << since;
    ASSERT_LE(std::abs(result.apply_torque), cap) << since;
    if (std::abs(result.desired_torque) == sp.steer_max) full_until = since;
  }
  ASSERT_GE(full_until, sp.avoid_lkas_fault_max_frames - sp.steer_max / sp.steer_delta_down - 1)
      << "최대 요청은 하강에 필요한 프레임 전까지 그대로 둔다(예전 램프는 첫 프레임부터 줄였다)";

  result = step();
  // 89번째 프레임: 토크 0에서 요청을 끈다. disengage가 아니고 ToiFlt도 세우지 않는다
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
  ASSERT_LE(std::abs(result.apply_torque), sp.steer_delta_up);
  for (int i = 0; i < 30; ++i) result = step();
  ASSERT_TRUE(result.active);
  ASSERT_NE(result.apply_torque, 0);
}

/* 정차 대기(Stopped)도 steer 요청을 잡고 있으므로 같은 회피를 건다. 예전에는 active일 때만 세어,
 * 정차에서 핸들을 85도 넘게 감고 출발하면 1초 뒤 고장이 났다(2026-10-03 2:54). 85도 위에서
 * 결합하면 요청을 켜지 않고 85도 아래로 올 때까지 기다린다. */
TEST(LateralController, LargeAngleHoldCoversStopAndEngage) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
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
  auto steer_req = [](const LateralControlResult &r) {
    EXPECT_FALSE(r.frames.empty()) << "LKAS11을 보낸다";
    return !r.frames.empty() && decode_lkas11(r.frames.front().data).steer_req;
  };

  vehicle.steering_angle_deg = 10.0f;
  LateralControlResult result;
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
  config.driving_params.vehicle_state_timeout_ms = 2000;
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

// 정차(path 무효)에서도 engage는 받아야 한다 — 조향만 쉰다.
TEST(LateralController, EngageAllowedWithUnavailablePath) {
  LateralControllerConfig config;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  LateralPath bad = replay_path();
  bad.usable_for_steering = false;
  VehicleCanState vehicle = ready_vehicle(1.0);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
  vehicle.clu_button = 2;
  controller.update(bad, replay_target(), vehicle, 1.0, 0, true, true);
  vehicle.clu_button = 0;
  const auto engaged =
      controller.update(bad, replay_target(), vehicle, 1.01, 1, true, true);
  // 정차 중에는 경로가 없어도 engage를 받는다
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
  const auto rolling =
      controller.update(bad, replay_target(), vehicle, 1.02, 2, true, true);
  // 주행 중 쓸 수 없는 경로는 path_invalid로 알린다
  ASSERT_FALSE(rolling.active);
  ASSERT_EQ(rolling.active_block, BlockReason::PathInvalid);
  // 결함은 가용성 대기보다 우선한다 (정차 중 문 열림 -> hard disengage)
  vehicle = ready_vehicle(1.03);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
  vehicle.door_open = true;
  const auto door =
      controller.update(bad, replay_target(), vehicle, 1.03, 3, true, true);
  // 고장은 사용 가능 여부보다 우선하고 정차 중에도 바로 disengage한다
  ASSERT_EQ(door.active_block, BlockReason::DoorOpen);
  ASSERT_FALSE(door.engaged);
}

/* openpilot calibrationIncomplete/Recalibrating/Invalid(SOFT_DISABLE + NO_ENTRY). */
TEST(LateralController, CalibrationGatesEngageAndSoftDisables) {
  LateralControllerConfig config;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle(1.0);
  auto press_set = [&](double t) {
    vehicle = ready_vehicle(t);
    vehicle.clu_button = 2;
    controller.update(replay_path(), replay_target(), vehicle, t, 0, true, true);
    vehicle = ready_vehicle(t + 0.01);
    return controller.update(replay_path(), replay_target(), vehicle, t + 0.01, 1, true, true);
  };

  controller.set_calibration_status(0);
  const auto rejected = press_set(1.0);
  EXPECT_TRUE(rejected.engage_rejected) << "캘리브레이션 미완료면 engage를 거부한다";
  EXPECT_FALSE(rejected.engaged);
  EXPECT_EQ(rejected.active_block, BlockReason::CalibrationIncomplete);

  controller.set_calibration_status(1);
  const auto engaged = press_set(2.0);
  ASSERT_TRUE(engaged.engaged && engaged.active);

  // engage 중 재보정: 경고를 띄운 채 3초 조향하고 해제한다.
  controller.set_calibration_status(3);
  LateralControlResult r;
  double t = 2.02;
  for (; t < 2.02 + 2.9; t += 0.01) {
    vehicle = ready_vehicle(t);
    r = controller.update(replay_path(), replay_target(), vehicle, t, 2, true, true);
    ASSERT_TRUE(r.engaged && r.active && r.soft_disabling) << "해제 예고 중에는 조향을 유지한다 t=" << t;
    ASSERT_EQ(r.active_block, BlockReason::CalibrationRecalibrating);
  }
  for (; t < 2.02 + 3.2; t += 0.01) {
    vehicle = ready_vehicle(t);
    r = controller.update(replay_path(), replay_target(), vehicle, t, 2, true, true);
  }
  EXPECT_FALSE(r.engaged) << "3초가 지나면 해제한다";
  EXPECT_FALSE(r.active);
  EXPECT_FALSE(r.soft_disabling);

  // 3초 안에 보정이 돌아오면 해제하지 않고 타이머도 처음부터 다시 센다.
  controller.set_calibration_status(1);
  ASSERT_TRUE(press_set(10.0).engaged);
  controller.set_calibration_status(2);
  for (t = 10.02; t < 11.5; t += 0.01) {
    vehicle = ready_vehicle(t);
    r = controller.update(replay_path(), replay_target(), vehicle, t, 2, true, true);
  }
  EXPECT_TRUE(r.soft_disabling);
  EXPECT_EQ(r.active_block, BlockReason::CalibrationInvalid);
  controller.set_calibration_status(1);
  vehicle = ready_vehicle(t);
  r = controller.update(replay_path(), replay_target(), vehicle, t, 2, true, true);
  EXPECT_TRUE(r.engaged && r.active && !r.soft_disabling);
}

TEST(LateralController, ClipCurvatureReportsAccelLimit) {
  bool limited = true;
  clip_curvature(20.0f, 0.0f, 0.001f, 0.0f, &limited);
  EXPECT_FALSE(limited);
  // 저크 한계만 물면 limited가 아니다(상류 clip_curvature와 같음).
  clip_curvature(20.0f, 0.0f, 0.005f, 0.0f, &limited);
  EXPECT_FALSE(limited);
  // 횡가속 3.3 m/s² 한계: 20 m/s에서 0.00825 1/m
  clip_curvature(20.0f, 0.0082f, 0.0095f, 0.0f, &limited);
  EXPECT_TRUE(limited);
}

/* openpilot steerSaturated: 목표 횡가속이 한계에 잘려 0.4초 넘게 포화이고, 실제가 목표의
 * 1/1.2에 못 미치는 커브에서만 경고한다. */
TEST(LateralController, SteerSaturatedWarnsWhenTurnExceedsLimit) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralTarget target = replay_target();
  for (int i = 0; i < kLateralControlN; ++i) {  // 20 m/s에서 횡가속 8 m/s² 요구
    target.curvatures[i] = 0.02f;
    target.psis[i] = 0.02f * 17.0f * model_t_idx(i);
  }
  // 실제 곡률은 상류처럼 조향각(차량 모델)에서 나온다.
  auto run = [&](float steering_angle_deg) {
    LateralController controller(config);
    LateralControlResult r;
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
  // 조향각 60도면 20 m/s에서 횡가속이 한계(3.3 m/s²)를 넘는다: 목표/실제 < 1.2 → 경고 없음
  EXPECT_FALSE(run(60.0f)) << "잘린 목표를 따라가고 있으면 경고하지 않는다";
  EXPECT_FALSE(run(-60.0f)) << "크기로 비교한다(상류와 같음)";
}

/* openpilot처럼 운전자가 핸들을 잡아도 요청 토크를 줄이지 않는다. 2026-09-27 고속도로 램프:
 * 같은 방향으로 거들자 예전 1초 페이드가 토크를 0으로 만들었다. 운전자와 반대 방향 토크만
 * panda와 같은 운전자 클램프가 줄인다. */
TEST(LateralController, DriverTorqueDoesNotFadeRequest) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  LateralTarget target = replay_target();
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = -0.004f;
    target.psis[i] = -0.004f * 17.0f * model_t_idx(i);
  }
  LateralControlResult r;
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
  config.driving_params.vehicle_state_timeout_ms = 2000;
  // openpilot 곡률 한계만 본다. 손을 뗀 상태의 조향각 상한(80도)은 HoldAngleCapsOwnSteeringOnly가 본다.
  config.steering_params.avoid_lkas_fault_hold_angle_deg = 0.0f;
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
  /* 틱당 변화율 제한이 걸리므로 한 번에 상한까지 뛰지 않는다. 1 m/s에서
   * 창은 5/(1*1)*0.01 = 0.05 1/m 다. */
  const auto first = controller.update(replay_path(), target, vehicle, 1.0, 0);
  // 첫 틱은 한계로 뛰지 않고 횡저크 한 스텝만 움직인다
  ASSERT_TRUE(first.active);
  ASSERT_NEAR(first.desired_curvature, 0.05f, 1e-6f);
  LateralControlResult result = first;
  for (int tick = 1; tick < 40; ++tick)
    result = controller.update(replay_path(), target, vehicle,
                               1.0 + 0.01 * tick, tick);
  ASSERT_NEAR(result.desired_curvature, 0.2f, 1e-6f)
      << "최대 곡률은 openpilot의 0.2 1/m 고정";
}

// 라이브 뱅크: 커브(|yaw*v| >= 0.4)에서는 갱신을 멈추고 직선 값을 유지해야 한다.
TEST(LateralController, BankHoldsDuringCurves) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
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

  const auto active =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  ASSERT_TRUE(active.active) << "런타임 파라미터 검사는 활성 상태에서 시작한다";

  SteeringParams steering = config.steering_params;
  steering.enabled = false;
  controller.update_params(steering, config.driving_params);
  const auto disabled =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  // 런타임 조향 파라미터는 다음 제어 틱에 적용된다
  ASSERT_FALSE(disabled.active);
  ASSERT_EQ(disabled.active_block, BlockReason::ControllerDisabled);

  steering.enabled = true;
  controller.update_params(steering, config.driving_params);
  const auto resumed =
      controller.update(replay_path(), replay_target(), vehicle, 1.02, 2);
  ASSERT_TRUE(resumed.active) << "런타임 파라미터를 바꿔도 컨트롤러 동작이 이어진다";
}

TEST(LateralController, LkasHudStateStability) {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

  LateralPath no_lane_path = replay_path();
  no_lane_path.left_valid = false;
  no_lane_path.right_valid = false;
  const auto active =
      controller.update(no_lane_path, replay_target(), vehicle, 1.0, 0);
  // 활성 상태의 HUD 프레임
  ASSERT_TRUE(active.active);
  ASSERT_FALSE(active.frames.empty());
  ASSERT_EQ(decode_lkas11(active.frames.front().data).ldws_sys_state, 3)
      << "차선 확률이 흔들려도 HUD는 활성으로 남는다";

  /* 클러스터는 sys_state 천이마다 부저를 울리므로, sys_state는 active가
   * 아니라 engaged만 따른다. enable/disable에서만 천이가 생긴다. */
  LateralTarget invalid_target = replay_target();
  invalid_target.mpc_solution_valid = false;
  const auto inactive =
      controller.update(no_lane_path, invalid_target, vehicle, 2.5, 2);
  // 비활성 상태의 HUD 프레임
  ASSERT_FALSE(inactive.active);
  ASSERT_FALSE(inactive.frames.empty());
  ASSERT_EQ(decode_lkas11(inactive.frames.front().data).ldws_sys_state, 3)
      << "engage 중 비활성이어도 부저가 울리지 않게 sys_state를 유지한다";
}

TEST(LateralController, PandaGateAndHandoff) {
  LateralControllerConfig config;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

  vehicle.clu_button = 2;
  const auto set_press =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0, true, true);
  ASSERT_FALSE(set_press.engaged) << "SET은 떼기 전에 engage하지 않는다";

  vehicle.clu_button = 0;
  const auto panda_blocked =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1, true, false);
  // Panda controls 게이트
  ASSERT_TRUE(panda_blocked.engaged);
  ASSERT_FALSE(panda_blocked.active);
  ASSERT_EQ(panda_blocked.active_block, BlockReason::PandaControlsOff);
  ASSERT_FALSE(panda_blocked.engage_rejected)
      << "Panda controls 핸드셰이크 대기 중에도 유효한 SET을 거부하지 않는다";
  // Panda 불일치 중에도 토크 0 대체 스트림을 계속 보낸다
  ASSERT_TRUE(panda_blocked.should_send);
  ASSERT_FALSE(panda_blocked.frames.empty());
  const HyundaiLkas11Values zero_lkas =
      decode_lkas11(panda_blocked.frames.front().data);
  // Panda controls가 꺼져 있으면 LKAS 토크 0
  ASSERT_EQ(zero_lkas.steer_torque, 0);
  ASSERT_FALSE(zero_lkas.steer_req);

  LateralController panda_timeout_controller(config);
  /* 이 블록은 t=10 에서 돈다. CAN 타임스탬프를 함께 옮기지 않으면 차량 상태가
   * 9초 낡아 vehicle_state_stale 이 되고, 검증하려던 Panda 유예가 아니라
   * freshness 게이트를 보게 된다. */
  VehicleCanState timeout_vehicle = ready_vehicle(10.0);
  timeout_vehicle.clu_button = 2;
  panda_timeout_controller.update(replay_path(), replay_target(), timeout_vehicle,
                                  10.0, 0, true, true);
  timeout_vehicle.clu_button = 0;
  const auto panda_waiting = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle, 10.01, 1, true, false);
  // Panda 핸드셰이크 유예 동안 유효한 요청을 붙잡아 둔다
  ASSERT_TRUE(panda_waiting.engaged);
  ASSERT_FALSE(panda_waiting.engage_rejected);
  // 유예(1초)를 넘기며 CAN 은 계속 신선하게 유지한다.
  VehicleCanState timeout_vehicle_later = ready_vehicle(11.0);
  const auto panda_timeout = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle_later, 11.02, 102, true, false);
  // Panda 불일치가 이어지면 결국 engage를 거부한다
  ASSERT_FALSE(panda_timeout.engaged);
  ASSERT_TRUE(panda_timeout.engage_rejected);
  ASSERT_EQ(panda_timeout.active_block, BlockReason::PandaControlsOff);

  LateralController deferred_static_controller(config);
  // t=12 시점 검사이므로 차량 데이터도 신선해야 한다(낡으면 stale이 우선).
  VehicleCanState deferred_vehicle = ready_vehicle(12.0);
  deferred_vehicle.wheel_speed_fl_kph = deferred_vehicle.wheel_speed_fr_kph = 60.0f;
  deferred_vehicle.wheel_speed_rl_kph = deferred_vehicle.wheel_speed_rr_kph = 60.0f;
  deferred_vehicle.clu_button = 2;
  deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle,
                                    12.0, 0, true, true);
  deferred_vehicle.clu_button = 0;
  deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle,
                                    12.01, 1, true, false);
  LateralTarget deferred_invalid_target = replay_target();
  deferred_invalid_target.mpc_solution_valid = false;
  const auto deferred_static = deferred_static_controller.update(
      replay_path(), deferred_invalid_target, deferred_vehicle, 12.02, 2, true, true);
  // Panda가 회복되면 정적 engage 게이트를 다시 본다
  ASSERT_FALSE(deferred_static.engaged);
  ASSERT_TRUE(deferred_static.engage_rejected);
  ASSERT_EQ(deferred_static.active_block, BlockReason::LateralPlanInvalid);

  const auto active =
      controller.update(replay_path(), replay_target(), vehicle, 1.02, 2, true, true);
  // Panda controls가 켜지면 활성이 된다
  ASSERT_TRUE(active.engaged);
  ASSERT_TRUE(active.active);

  vehicle.clu_button = 4;
  const auto cancel =
      controller.update(replay_path(), replay_target(), vehicle, 1.03, 3, true, false);
  // CANCEL을 누르면 토크 0 프레임 인계를 시작한다
  ASSERT_FALSE(cancel.engaged);
  ASSERT_TRUE(cancel.should_send);
  const auto release_tail =
      controller.update(replay_path(), replay_target(), vehicle, 4.02, 302, true, false);
  ASSERT_TRUE(release_tail.should_send) << "인계는 3000 ms 이어진다";
  const auto stock_handoff =
      controller.update(replay_path(), replay_target(), vehicle, 4.04, 304, true, false);
  ASSERT_FALSE(stock_handoff.should_send) << "인계는 3000 ms 뒤 멈춘다";

  vehicle.lkas11_seed[4] = 9U << 4;
  stamp_can_times(&vehicle, 4.05);
  vehicle.clu_button = 2;
  controller.update(replay_path(), replay_target(), vehicle, 4.05, 305, true, true);
  vehicle.clu_button = 0;
  const auto reengaged =
      controller.update(replay_path(), replay_target(), vehicle, 4.06, 306, true, true);
  // 인계 뒤 다시 engage
  ASSERT_TRUE(reengaged.active);
  ASSERT_FALSE(reengaged.frames.empty());
  ASSERT_EQ(decode_lkas11(reengaged.frames.front().data).msg_count, 10)
      << "다시 engage하면 순정 카메라의 LKAS 카운터에서 이어간다";

  LateralController rejected_controller(config);
  VehicleCanState rejected_vehicle = ready_vehicle();
  rejected_vehicle.clu_button = 2;
  rejected_controller.update(replay_path(), replay_target(), rejected_vehicle,
                              1.0, 0, true, true);
  rejected_vehicle.clu_button = 0;
  LateralTarget invalid_target = replay_target();
  invalid_target.mpc_solution_valid = false;
  const auto rejected = rejected_controller.update(
      replay_path(), invalid_target, rejected_vehicle, 1.01, 1, true, true);
  // 정적 engage 게이트는 engaged 상태를 남기지 않고 거부한다
  ASSERT_FALSE(rejected.engaged);
  ASSERT_TRUE(rejected.engage_rejected);
  ASSERT_EQ(rejected.active_block, BlockReason::LateralPlanInvalid);
}

/* 시동 직후 첫 engage: Panda health 가 아직 없고 안전벨트/기어가 막고 있을 때.
 * 실차(2026-09-12)에서 engage 톤이 울린 뒤 해제되고, 두 번째 시도부터만
 * 거절음이 났다. 하드 결함이 panda_not_ready 뒤로 밀려 가려졌기 때문이다. */
TEST(LateralController, ColdStartEngageReportsHardBlock) {
  const auto cold_start_attempt = [](bool panda_ready, bool seatbelt_unlatched,
                                     int gear) {
    LateralControllerConfig config;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.seatbelt_unlatched = seatbelt_unlatched;
    vehicle.gear = gear;
    vehicle.clu_button = 2;
    controller.update(replay_path(), replay_target(), vehicle, 1.0, 0,
                      panda_ready, panda_ready);
    vehicle.clu_button = 0;
    return controller.update(replay_path(), replay_target(), vehicle, 1.01, 1,
                             panda_ready, panda_ready);
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
  const LateralControlResult stale =
      controller.update(replay_path(), target, ready_vehicle(1.19), 1.9, 20);
  ASSERT_EQ(stale.active_block, BlockReason::VehicleStateStale)
      << "휠 속도가 시한보다 오래됐다";
  ASSERT_EQ(stale.desired_curvature, before) << "속도가 낡으면 마지막 목표 곡률을 유지한다";
  const LateralControlResult back =
      controller.update(replay_path(), target, ready_vehicle(1.91), 1.91, 21);
  const float step = kMaxLateralJerk / (60.0f / 3.6f * 60.0f / 3.6f) * kCurvatureRateWindowS;
  ASSERT_NEAR(back.desired_curvature, before, step * 1.001f)
      << "회복하면 멈추기 전 곡률에서 이어간다";
}

// 보드는 JSON을 읽고 재생·테스트는 기본값을 쓴다. 둘이 갈리면 재생 대조가 보드를 대변하지 못한다.
TEST(LateralController, SteeringJsonMatchesDefaults) {
  SteeringParams json, defaults;
  std::string error;
  ASSERT_TRUE(load_steering_params_json("params/steering.json", &json, &error))
      << "steering.json 읽기";
  // steering.json 토크 이득이 코드 기본값과 같다
  ASSERT_EQ(json.torque_lat_accel_factor, defaults.torque_lat_accel_factor);
  ASSERT_EQ(json.torque_kp, defaults.torque_kp);
  ASSERT_EQ(json.torque_ki, defaults.torque_ki);
  ASSERT_EQ(json.torque_friction, defaults.torque_friction);
  const std::string path = "/tmp/gtest_lateral_controller_raw_keys.json";
  std::FILE *f = std::fopen(path.c_str(), "w");
  ASSERT_NE(f, nullptr) << "raw 키 픽스처 쓰기";
  std::fputs("{\"torque_kf_raw\": 20}\n", f);
  std::fclose(f);
  SteeringParams rejected;
  const bool loaded = load_steering_params_json(path, &rejected, &error);
  std::remove(path.c_str());
  // 2026-09-24 이전 raw 이득 키는 기본값으로 넘어가지 않고 거부된다
  ASSERT_FALSE(loaded);
  ASSERT_NE(error.find("torque_kf_raw"), std::string::npos);
}

/* 상류 controlsd: 비활성 중 목표 곡률은 실제 곡률을 따라가고, 재활성 때 거기서 한계 안으로 출발한다. */
TEST(LateralController, InactiveDesiredTracksActual) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.steering_params.angle_offset_deg = 0.0f;
  LateralController controller(config);
  const float v = 60.0f / 3.6f;
  const float step = kMaxLateralJerk / (v * v) * kCurvatureRateWindowS;
  LateralPath blocked = replay_path();
  blocked.usable_for_steering = false;
  LateralControlResult r;
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

/* 운전자가 85도 넘게 돌려 넘겨받은 회전(steering pressed)에서는 요청을 끈 뒤 핸들이 15도 아래로
 * 오고 손을 뗄 때까지 끈 채로 둔다(carrotpilot 해제 조건). 빠져나오며 펴는 핸들을 밀지 않는다.
 * 운전자가 손대지 않은 체류는 85도 아래로 오면 바로 다시 켠다(LargeAngleFaultAvoidance). */
TEST(LateralController, DriverTakeoverHoldsUntilCentered) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
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
  auto steer_req = [](const LateralControlResult &r) {
    EXPECT_FALSE(r.frames.empty()) << "LKAS11을 보낸다";
    return !r.frames.empty() && decode_lkas11(r.frames.front().data).steer_req;
  };

  LateralControlResult result;
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
  ASSERT_LE(std::abs(result.apply_torque), sp.steer_delta_up);
}

/* 운전자가 핸들을 잡지 않으면 컨트롤러 목표를 avoid_lkas_fault_hold_angle_deg(80도) 조향각이 내는
 * 곡률 안으로 묶는다. 85도를 넘겨 토크가 끊겼다 다시 잡는 반복 대신 그 각도에서 버틴다. 운전자가
 * 조향 중이거나 0으로 끄면 묶지 않는다. */
TEST(LateralController, HoldAngleCapsOwnSteeringOnly) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
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
    LateralControlResult r;
    for (int f = 0; f < 300; ++f) {
      stamp_can_times(&vehicle, 1.0 + f * 0.01);
      r = controller.update(replay_path(), tight, vehicle, 1.0 + f * 0.01, f);
    }
    EXPECT_TRUE(r.active);
    return r.desired_curvature;
  };
  TorqueController model;
  const float right_cap = model.curvature_at_angle(v, -80.0f, config.steering_params);
  const float left_cap = model.curvature_at_angle(v, 80.0f, config.steering_params);
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

/* 회전 desire 중(계획의 turn_desire) 운전자가 깜빡이 방향으로 돌리고 있으면 반대 방향 토크는 내지
 * 않는다. 회전 desire가 없거나, 운전자가 반대로 돌리거나, 손만 얹은 정도면 평소대로다. */
TEST(LateralController, TurnDesireDoesNotPushAgainstDriver) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  // 20 km/h, 왼쪽 깜빡이. 계획은 오른쪽(+곡률)을 원한다(걷는 속도에서 회전을 놓친 계획)
  auto run = [&](int turn_desire, int driver_torque) {
    LateralTarget plan = replay_target();
    for (int i = 0; i < kLateralControlN; ++i) {
      plan.curvatures[i] = 0.02f;
      plan.psis[i] = 0.02f * (20.0f / 3.6f) * model_t_idx(i);
    }
    plan.turn_desire = turn_desire;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 20.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 20.0f;
    vehicle.cluster_speed_raw = 21.0f;
    vehicle.left_blinker = true;
    vehicle.steering_angle_deg = 10.0f;
    vehicle.driver_torque = driver_torque;
    LateralControlResult r;
    for (int f = 0; f < 200; ++f) {
      stamp_can_times(&vehicle, 1.0 + f * 0.01);
      r = controller.update(replay_path(), plan, vehicle, 1.0 + f * 0.01, f);
    }
    EXPECT_TRUE(r.active);
    return r;
  };
  ASSERT_LT(run(0, 120).desired_torque, 0) << "회전 desire가 없으면 계획대로 오른쪽(운전자 반대) 토크다";
  const LateralControlResult turning = run(1, 120);
  // 왼쪽 회전 desire 중 왼쪽으로 돌리는 운전자를 밀지 않는다
  EXPECT_GE(turning.desired_torque, 0);
  EXPECT_GE(turning.apply_torque, 0);
  EXPECT_LT(run(1, -120).desired_torque, 0) << "운전자가 깜빡이 반대로 돌리면 평소대로다";
  EXPECT_LT(run(1, 30).desired_torque, 0) << "손만 얹은 정도(허용치 50 이하)는 평소대로다";
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

/* 스위치를 끄면 학습값을 넣어도 모든 틱이 비트 단위로 같아야 한다. 무효·캘리브 완료도
 * 끈 쪽에서는 차단하지 않는다. */
TEST(LateralController, LiveParamsSwitchOffIsIdentical) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
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
    fed.set_live_params(odd_live_params(), false, true);
    const LateralControlResult a = plain.update(replay_path(), target, vehicle, t, tick);
    const LateralControlResult b = fed.update(replay_path(), target, vehicle, t, tick);
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
  config.driving_params.vehicle_state_timeout_ms = 2000;
  config.steering_params.use_live_vehicle_params = true;
  LateralController controller(config);
  const VehicleCanState vehicle = ready_vehicle(1.0);
  controller.set_live_params(odd_live_params(), true, true);
  ASSERT_TRUE(controller.update(replay_path(), replay_target(), vehicle, 1.0, 0).active)
      << "유효한 학습값은 제어를 활성으로 둔다";
  controller.set_live_params(odd_live_params(), false, false);
  ASSERT_TRUE(controller.update(replay_path(), replay_target(), vehicle, 1.01, 1).active)
      << "보정 전 무효 학습값은 막지 않는다(상류 cal_status 검사)";
  controller.set_live_params(odd_live_params(), false, true);
  ASSERT_EQ(controller.update(replay_path(), replay_target(), vehicle, 1.02, 2).active_block,
            BlockReason::ParamsdInvalid)
      << "보정 뒤 무효 학습값은 막는다";
  LiveLateralParams unseen = odd_live_params();
  unseen.use_vehicle = false;
  controller.set_live_params(unseen, false, true);
  ASSERT_NE(controller.update(replay_path(), replay_target(), vehicle, 1.03, 3).active_block,
            BlockReason::ParamsdInvalid)
      << "paramsd가 발행하기 전에는 막지 않는다(상류 sm.seen)";
}

TEST(LateralController, CurvatureLimitFollowsRoll) {
  LateralTarget target = replay_target();
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = 0.05f;
    target.psis[i] = 0.05f * 20.0f * model_t_idx(i);
  }
  const float v = 20.0f, roll = 0.03f;
  const float up = lag_adjusted_desired_curvature(target, v, 0.0f, 0.34f, 0.05f, roll);
  ASSERT_NEAR(up, (kMaxLateralAccel + roll * 9.81f) / (v * v), 1e-7f)
      << "롤만큼 횡가속도 상한이 roll*g 움직인다";
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = -0.05f;
    target.psis[i] = -target.psis[i];
  }
  const float down = lag_adjusted_desired_curvature(target, v, 0.0f, 0.34f, -0.05f, roll);
  ASSERT_NEAR(down, (-kMaxLateralAccel + roll * 9.81f) / (v * v), 1e-7f)
      << "하한도 같은 roll*g만큼 움직인다";
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
  on.set_live_delay(0.9f, true);
  EXPECT_FLOAT_EQ(on.plan_delay_s(), 0.65f);
  on.set_live_delay(std::nanf(""), true);
  EXPECT_FALSE(on.live_delay_in_use());
}

// 명령줄로 받은 CAN 픽스처(없으면 CanFixture를 건너뛴다)
const char *g_fixture_path = nullptr;

struct TimedCanFrame {
  uint64_t timestamp_us = 0;
  CanFrame frame;
};

uint32_t read_u32_le(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8u) |
         (static_cast<uint32_t>(data[2]) << 16u) | (static_cast<uint32_t>(data[3]) << 24u);
}

uint64_t read_u64_le(const uint8_t *data) {
  return static_cast<uint64_t>(read_u32_le(data)) |
         (static_cast<uint64_t>(read_u32_le(data + 4)) << 32u);
}

/* EDGECAN1: "EDGECAN1", u32 version 1, u32 record_size 24, u64 count, 이어서 레코드
 * (u64 timestamp_us, u32 address, u8 bus, u8 length, data[8], 2 B 패딩). 시간순이어야 한다.
 * 형식이 틀리면 실패를 남기고 빈 목록을 돌려준다. */
std::vector<TimedCanFrame> read_can_fixture(const std::string &path) {
  constexpr size_t kHeaderSize = 24;
  constexpr uint32_t kRecordSize = 24;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    ADD_FAILURE() << "열 수 없음 " << path;
    return {};
  }
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                                   std::istreambuf_iterator<char>());
  if (bytes.size() < kHeaderSize || std::memcmp(bytes.data(), "EDGECAN1", 8) != 0) {
    ADD_FAILURE() << "CAN 픽스처 magic이 틀림";
    return {};
  }
  if (read_u32_le(bytes.data() + 8) != 1 || read_u32_le(bytes.data() + 12) != kRecordSize) {
    ADD_FAILURE() << "지원하지 않는 CAN 픽스처 버전";
    return {};
  }
  const uint64_t count = read_u64_le(bytes.data() + 16);
  if (count > (std::numeric_limits<size_t>::max() - kHeaderSize) / kRecordSize ||
      bytes.size() != kHeaderSize + static_cast<size_t>(count) * kRecordSize) {
    ADD_FAILURE() << "CAN 픽스처 크기가 틀림";
    return {};
  }
  std::vector<TimedCanFrame> records;
  records.reserve(static_cast<size_t>(count));
  for (size_t index = 0; index < static_cast<size_t>(count); ++index) {
    const uint8_t *record = bytes.data() + kHeaderSize + index * kRecordSize;
    TimedCanFrame timed;
    timed.timestamp_us = read_u64_le(record);
    timed.frame.address = read_u32_le(record + 8);
    timed.frame.bus = record[12];
    timed.frame.length = record[13];
    std::copy_n(record + 14, timed.frame.data.size(), timed.frame.data.begin());
    if ((index > 0 && timed.timestamp_us < records.back().timestamp_us) ||
        timed.frame.address > 0x1fffffffU || timed.frame.bus > 7u ||
        timed.frame.length > timed.frame.data.size()) {
      ADD_FAILURE() << "CAN 픽스처 레코드가 틀림, 인덱스 " << index;
      return {};
    }
    records.push_back(timed);
  }
  return records;
}

/* 픽스처는 60초 연속 주행 구간이어야 한다(active > 5900틱, 토크 > 0). 정차
 * 구간은 이 전제에 걸려 실패한다. tools/control/export_can_fixture.py가 녹화
 * events/NNN.bin 하나를 이 형식으로 내보낸다. */
TEST(LateralController, CanFixture) {
  if (g_fixture_path == nullptr)
    GTEST_SKIP() << "gtest_lateral_controller <fixture.can>으로 준 경우만 돈다";
  const std::vector<TimedCanFrame> records = read_can_fixture(g_fixture_path);
  ASSERT_FALSE(records.empty()) << "CAN 픽스처에 프레임이 없다";
  LateralControllerConfig config;
  config.force_engaged = true;
  std::string error;
  ASSERT_TRUE(load_steering_params_json("params/steering.json", &config.steering_params, &error))
      << "조향 파라미터 읽기";
  ASSERT_TRUE(load_driving_params_json("params/driving.json", &config.driving_params, &error))
      << "주행 파라미터 읽기";
  LateralController controller(config);
  VehicleCanState vehicle;
  const LateralPath path = replay_path();
  const LateralTarget target = replay_target();
  size_t next_record = 0;
  size_t active_ticks = 0;
  size_t invalid_frames = 0;
  size_t lkas0 = 0;
  size_t lkas1 = 0;
  size_t clu1 = 0;
  size_t mdps2 = 0;
  int max_torque = 0;
  float max_curvature_error = 0.0f;
  float ref_prev_curvature = 0.0f;

  const double duration_s = records.back().timestamp_us / 1000000.0;
  const int ticks = static_cast<int>(std::ceil(duration_s * 100.0)) + 2;
  for (int tick = 0; tick < ticks; ++tick) {
    const double now_s = static_cast<double>(tick) * 0.01;
    const uint64_t now_us = static_cast<uint64_t>(now_s * 1000000.0);
    // 이 틱까지 도착한 프레임을 먹인다.
    for (; next_record < records.size() && records[next_record].timestamp_us <= now_us;
         ++next_record) {
      const CanFrame &frame = records[next_record].frame;
      update_vehicle_can_state(&vehicle, frame.address, frame.data,
                               frame.length, frame.bus, now_s);
    }
    const auto result = controller.update(path, target, vehicle, now_s, tick);
    /* 컨트롤러는 휠 속도 평균으로 곡률을 낸다. 클러스터 속도를 먹이면
     * 참조식이 다른 입력을 보게 되어 비교가 성립하지 않는다. 첫 WHL_SPD11
     * 전에는 속도가 NaN이고 컨트롤러가 어차피 비활성이라 비교하지 않는다. */
    if (std::isfinite(result.control_speed_kph)) {
      // 컨트롤러처럼 비활성이면 plan 대신 실제 곡률을 클립한다.
      const float speed = std::max(0.0f, result.control_speed_kph / 3.6f);
      const float requested =
          result.active
              ? reference_plan_curvature(target, speed, config.steering_params.steer_actuator_delay)
              : result.actual_curvature;
      const float expected_curvature =
          reference_clip_curvature(speed, ref_prev_curvature, requested);
      ref_prev_curvature = expected_curvature;
      max_curvature_error = std::max(
          max_curvature_error, std::fabs(result.desired_curvature - expected_curvature));
    }
    if (result.active) ++active_ticks;
    max_torque = std::max(max_torque, std::abs(result.apply_torque));
    for (const CanFrame &frame : result.frames) {
      const bool valid_length =
          (frame.address == kHyundaiClu11Address && frame.length == 4) ||
          (frame.address != kHyundaiClu11Address && frame.length == 8);
      if (!valid_length) ++invalid_frames;
      if (frame.address == kHyundaiLkas11Address && frame.bus == 0) ++lkas0;
      if (frame.address == kHyundaiLkas11Address && frame.bus == 1) ++lkas1;
      if (frame.address == kHyundaiClu11Address && frame.bus == 1) ++clu1;
      if (frame.address == kHyundaiMdps12Address && frame.bus == 2) ++mdps2;
    }
  }
  ASSERT_EQ(next_record, records.size()) << "픽스처 프레임을 모두 먹였다";
  ASSERT_GT(active_ticks, 5900) << "주행 구간 내내 컨트롤러가 활성이다";
  ASSERT_EQ(invalid_frames, 0) << "생성한 CAN 프레임의 길이가 맞다";
  // LKAS/MDPS는 100 Hz로 나간다
  ASSERT_GT(lkas0, 5900);
  ASSERT_EQ(lkas0, lkas1);
  ASSERT_EQ(lkas0, mdps2);
  // CLU11은 50 Hz로 나간다
  ASSERT_GT(clu1, 2900);
  ASSERT_GE(clu1 * 2, lkas0 - 1);
  ASSERT_LE(clu1 * 2, lkas0 + 1);
  // 조향 토크 범위
  ASSERT_GT(max_torque, 0);
  ASSERT_LE(max_torque, 384);
  ASSERT_LT(max_curvature_error, 1e-6f)
      << "지연 보정 곡률이 openpilot 참조식과 같다";
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 2) {
    std::fprintf(stderr, "usage: %s [fixture.can]\n", argv[0]);
    return 2;
  }
  if (argc == 2) g_fixture_path = argv[1];
  return RUN_ALL_TESTS();
}
