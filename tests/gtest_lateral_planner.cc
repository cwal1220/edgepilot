/* 횡 플래너(LateralPlanner): laneless는 openpilot 메인의 get_curvature_from_plan과, 차선 변경은
 * 상류 desire_helper와 대조한다. path_offset_m의 적용 범위도 본다. MPC 자체의 최적성은
 * gtest_lateral_mpc가 본다. */
#include "controls/control_params.h"
#include "common/ipc_messages.h"
#include "controls/lateral_controller.h"
#include "planning/lateral_planner.h"
#include "common/model_output.h"
#include "car/vehicle_can.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

/* laneless 모드는 openpilot 메인의 get_curvature_from_plan이다: 모델 plan의 yaw·yaw rate만 쓰고
 * plan 위치·경로 오프셋·MPC를 쓰지 않는다. */
TEST(LateralPlanner, LanelessUsesPlanYawLikeUpstream) {
  SteeringParams steering;
  steering.path_offset_m = -0.3f;
  steering.laneless_mode = true;
  LateralPlanner planner(steering);
  const float v = 20.0f;
  auto model_for = [&](float kappa, float lateral_offset) {
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
      const float t = model_t_idx(i);
      ms.model_t[i] = t;
      ms.plan[i] = {v * t, lateral_offset, 0.0f};
      ms.plan_yaw[i] = kappa * v * t;
      ms.plan_yaw_rate[i] = kappa * v;
    }
    return ms;
  };
  VehicleCanState vehicle{};
  // 일정 곡률 plan: 2·ψ(t)/(v·t) − ψ̇/v = 2κ − κ = κ
  LateralTarget turn = planner.update(model_for(0.002f, 0.0f), vehicle, v, 0.0f, true);
  ASSERT_TRUE(turn.valid);
  ASSERT_TRUE(turn.laneless_mode);
  ASSERT_TRUE(turn.mpc_solution_valid);
  EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.0f, 0.34f), 0.002f, 1e-5f) << "일정 곡률 plan은 그 곡률";
  EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.1f, 0.34f), 0.002f, 1e-5f) << "plan 나이와 무관";
  // yaw가 0이면 plan이 옆으로 0.5 m 떨어져 있고 오프셋이 −0.3이어도 목표는 직진이다
  LateralTarget straight = planner.update(model_for(0.0f, 0.5f), vehicle, v, 0.0f, true);
  EXPECT_NEAR(lag_adjusted_curvature(straight, v, 0.05f, 0.34f), 0.0f, 1e-7f)
      << "위치와 경로 오프셋은 laneless 곡률에 들어가지 않는다";
}

/* 차선 변경은 openpilot desire_helper와 같다: 깜빡이 + 그 방향 핸들 토크로 시작하고, 모델이
 * 끝났다고 할 때(lane_change_prob < 0.02)나 10초·비활성으로만 끝난다. 차선선은 0.5초에 뺀다. */
TEST(LateralPlanner, LaneChangeFollowsUpstreamDesireHelper) {
  SteeringParams steering;
  LateralPlanner planner(steering);
  const float v = 20.0f;
  ModelState ms{};
  ms.valid = 1;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float t = model_t_idx(i);
    ms.model_t[i] = t;
    ms.lane_t[i] = t;
    ms.plan[i] = {v * t, 0.0f, 0.0f};
  }
  ms.desire_state[0] = 1.0f;
  VehicleCanState vehicle{};
  LateralTarget r = planner.update(ms, vehicle, v, 0.0f, true);
  ASSERT_EQ(r.desire, 0);
  vehicle.left_blinker = true;
  r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 0) << "깜빡이만으로는 시작하지 않는다";
  EXPECT_EQ(r.lane_change_state, 1) << "HUD: 핸들을 밀기를 기다린다";
  EXPECT_EQ(r.lane_change_direction, -1);
  vehicle.driver_torque = 300;  // 왼쪽으로 민다
  r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 3) << "깜빡이 방향으로 밀면 laneChangeLeft";
  EXPECT_EQ(r.lane_change_state, 2) << "HUD: 변경 중";
  // 모델이 차선 변경 중이라고 하는 동안(prob 높음)은 핸들을 놓아도 3초 넘게 이어진다.
  vehicle.driver_torque = 0;
  ms.desire_state[0] = 0.1f;
  ms.desire_state[3] = 0.9f;
  for (int i = 0; i < 60; ++i) r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 3) << "출력이 커져도 취소하지 않는다(예전 출력 0.8 취소 제거)";
  // 모델이 끝났다고 하면 마무리(차선선을 0.5초에 되살림) 동안은 openpilot 0.9.4 DESIRES처럼
  // laneChangeLeft를 유지하고, 끝나면 내린다.
  ms.desire_state[0] = 1.0f;
  ms.desire_state[3] = 0.0f;
  vehicle.left_blinker = false;
  r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 3) << "마무리 단계";
  for (int i = 0; i < 12; ++i) r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 0) << "차선선을 되살리면 끝난다";
  EXPECT_EQ(r.lane_change_state, 0);
}

/* 차선 변경 속도 아래에서 켠 깜빡이는 desire를 주지 않는다(상류 desire_helper). 빠를 때 시작한 차선 변경은
 * 그 속도 아래로 감속해도 이어진다. */
TEST(LateralPlanner, SlowBlinkerGivesNoDesire) {
  SteeringParams steering;
  LateralPlanner planner(steering);
  ModelState ms{};
  ms.valid = 1;
  const float v = 5.0f;  // 18 km/h < 30
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float t = model_t_idx(i);
    ms.model_t[i] = t;
    ms.lane_t[i] = t;
    ms.plan[i] = {v * t, 0.0f, 0.0f};
  }
  for (int l = 0; l < 4; ++l) ms.lane_probabilities[l] = 0.9f;
  ms.desire_state[0] = 1.0f;
  VehicleCanState vehicle{};
  vehicle.left_blinker = true;
  for (int i = 0; i < 60; ++i) {  // 3초
    const LateralTarget r = planner.update(ms, vehicle, v, 0.0f, true);
    ASSERT_EQ(r.desire, 0) << i;
    ASSERT_EQ(r.lane_change_state, 0) << i;
  }

  LateralPlanner changing(steering);
  vehicle = VehicleCanState{};
  vehicle.left_blinker = true;
  changing.update(ms, vehicle, 35.0f / 3.6f, 0.0f, true);
  vehicle.driver_torque = 300;  // 왼쪽 넛지
  EXPECT_EQ(changing.update(ms, vehicle, 35.0f / 3.6f, 0.0f, true).desire, 3) << "차선 변경 시작";
  vehicle.driver_torque = 0;
  EXPECT_EQ(changing.update(ms, vehicle, 25.0f / 3.6f, 0.0f, true).desire, 3)
      << "시작한 변경은 감속해도 이어진다";
}

/* path_offset_m은 차선 중심에만 적용된다. 차선이 없어 모델 경로로 넘어가면(교차로) 적용하지
 * 않는다: 차 기준인 모델 경로에 더하면 위치 고정점 없이 차가 오프셋 쪽으로 계속 밀린다. */
TEST(LateralPlanner, PathOffsetOnlyShiftsLanePath) {
  SteeringParams steering;
  steering.path_offset_m = -0.3f;
  LateralPlanner planner(steering);
  const float v = 15.0f;
  auto model_for = [&](float lane_prob) {
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
      const float t = model_t_idx(i);
      ms.model_t[i] = t;
      ms.lane_t[i] = t;
      ms.plan[i] = {v * t, 0.0f, 0.0f};
      ms.lanes[1][i] = {v * t, -1.75f, 0.0f};
      ms.lanes[2][i] = {v * t, 1.75f, 0.0f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = lane_prob;
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    ms.desire_state[0] = 1.0f;
    return ms;
  };
  VehicleCanState vehicle{};
  LateralTarget r;
  for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.99f), vehicle, v, 0.0f, true);
  ASSERT_FALSE(r.laneless_mode);
  EXPECT_NEAR(r.target_y_m, -0.3f, 0.03f) << "차선이 보이면 차선 중심에서 오프셋만큼";
  for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.0f), vehicle, v, 0.0f, true);
  ASSERT_TRUE(r.laneless_mode);
  EXPECT_NEAR(r.target_y_m, 0.0f, 0.01f) << "모델 경로로 넘어가면 오프셋을 더하지 않는다";
}

/* Lane 모드에서 차선이 안 보이면(교차로) laneless 모드와 같은 openpilot 메인 계산으로 목표를 낸다:
 * plan 위치가 직진이어도 plan yaw가 곡률 κ를 말하면 목표는 κ다. 차선을 잃는 순간에는 한 프레임에
 * 바꾸지 않고 0.5초에 걸쳐 섞는다. */
TEST(LateralPlanner, LaneModeFallbackMatchesLaneless) {
  const float v = 6.0f;
  const float kappa = 0.02f;
  auto model_for = [&](float lane_prob) {
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
      const float t = model_t_idx(i);
      ms.model_t[i] = t;
      ms.lane_t[i] = t;
      ms.plan[i] = {v * t, 0.0f, 0.0f};  // 위치는 직진
      ms.plan_yaw[i] = kappa * v * t;   // yaw는 κ로 회전
      ms.plan_yaw_rate[i] = kappa * v;
      ms.lanes[1][i] = {v * t, -1.75f, 0.0f};
      ms.lanes[2][i] = {v * t, 1.75f, 0.0f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = lane_prob;
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    ms.desire_state[0] = 1.0f;
    return ms;
  };
  SteeringParams steering;
  LateralPlanner lane_mode(steering);
  steering.laneless_mode = true;
  LateralPlanner laneless(steering);
  VehicleCanState vehicle{};
  LateralTarget r;
  for (int i = 0; i < 60; ++i) r = lane_mode.update(model_for(0.99f), vehicle, v, 0.0f, true);
  ASSERT_FALSE(r.laneless_mode);
  const float with_lanes = lag_adjusted_curvature(r, v, 0.0f, 0.42f);
  EXPECT_LT(std::fabs(with_lanes), 0.3f * kappa) << "차선이 보이면 차선(직진)을 따른다";

  std::vector<float> handoff;
  for (int i = 0; i < 40; ++i) {
    r = lane_mode.update(model_for(0.0f), vehicle, v, 0.0f, true);
    handoff.push_back(lag_adjusted_curvature(r, v, 0.0f, 0.42f));
  }
  ASSERT_TRUE(r.laneless_mode) << "차선을 잃으면 모델 경로 구간";
  const LateralTarget ref = laneless.update(model_for(0.0f), vehicle, v, 0.0f, true);
  EXPECT_NEAR(handoff.back(), lag_adjusted_curvature(ref, v, 0.0f, 0.42f), 1e-6f)
      << "자리 잡은 뒤에는 laneless 모드와 같은 목표";
  EXPECT_NEAR(handoff.back(), kappa, 1e-4f);
  float max_step = 0.0f;
  for (size_t i = 1; i < handoff.size(); ++i)
    max_step = std::max(max_step, std::fabs(handoff[i] - handoff[i - 1]));
  EXPECT_LT(max_step, 0.25f * kappa) << "인계는 한 프레임에 넘어가지 않는다";
}

/* lane_path_weight는 Lane 모드 MPC가 차선 중심에서 벗어난 위치를 되돌리는 세기다. 차가 차선 중심에서
 * 0.3 m 오른쪽에 있으면 가중치 3이 1보다 왼쪽(음) 곡률을 더 크게 요구한다. */
TEST(LateralPlanner, LanePathWeightStrengthensCentering) {
  const float v = 20.0f;
  auto model_for = [&]() {
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
      const float t = model_t_idx(i);
      ms.model_t[i] = t;
      ms.lane_t[i] = t;
      ms.plan[i] = {v * t, 0.0f, 0.0f};
      ms.lanes[1][i] = {v * t, -2.05f, 0.0f};  // 차가 차선 중심보다 0.3 m 오른쪽
      ms.lanes[2][i] = {v * t, 1.45f, 0.0f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = 0.99f;
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    ms.desire_state[0] = 1.0f;
    return ms;
  };
  auto desired = [&](float weight) {
    SteeringParams steering;
    steering.lane_path_weight = weight;
    LateralPlanner planner(steering);
    VehicleCanState vehicle{};
    LateralTarget r;
    for (int i = 0; i < 40; ++i) r = planner.update(model_for(), vehicle, v, 0.0f, true);
    EXPECT_FALSE(r.laneless_mode);
    return lag_adjusted_curvature(r, v, 0.0f, 0.42f);
  };
  const float upstream = desired(1.0f);
  const float stronger = desired(3.0f);
  EXPECT_LT(upstream, 0.0f) << "차선 중심(왼쪽)으로 돌아간다";
  EXPECT_LT(stronger, upstream * 1.2f) << "가중치 3은 더 강하게 되돌린다";
}

}  // namespace
