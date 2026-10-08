/* 차선 변경 상태 머신(DesireHelper, openpilot desire_helper 이식): 깜빡이 다음 운전자가 깜빡이 쪽으로
 * 핸들을 밀어야 변경이 시작되고, 차선선을 0.5초에 빼고 되돌린다. 사각지대면 기다리고, 10초가 넘으면 끈다.
 * 도로 경계 쪽으로는 시작하지 않다가 경계가 사라지면 시작한다. 모델 프레임(20 Hz)마다 부른다. */
#include "planning/desire_helper.h"
#include "car/vehicle_can.h"

#include <gtest/gtest.h>

namespace {

constexpr float kHighwaySpeedMps = 20.0f;

// 차선선 넷이 다 보이고 도로 경계는 흐린(경계 아님) 모델 프레임.
DesireModelInputs open_road(double lane_change_prob = 0.0) {
  DesireModelInputs model;
  model.lane_change_prob = lane_change_prob;
  model.lane_probabilities = {0.8, 0.9, 0.9, 0.8};
  model.road_edge_stds = {1.0, 1.0};
  return model;
}

VehicleCanState with_blinker(int side, int driver_torque = 0) {
  VehicleCanState vehicle;
  vehicle.left_blinker = side < 0;
  vehicle.right_blinker = side > 0;
  vehicle.driver_torque = driver_torque;
  return vehicle;
}

void run(DesireHelper *helper, int frames, const VehicleCanState &vehicle, const DesireModelInputs &model,
         float v_ego = kHighwaySpeedMps) {
  for (int i = 0; i < frames; ++i) helper->update(vehicle, v_ego, true, model);
}

TEST(DesireHelper, LaneChangeStartsWhenTheDriverPushesTowardTheBlinker) {
  DesireHelper helper;
  run(&helper, 1, with_blinker(-1), open_road());
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange);
  ASSERT_EQ(helper.direction(), -1);
  ASSERT_EQ(helper.desire(), Desire::None) << "핸들을 밀기 전에는 모델 desire가 없다";

  run(&helper, 5, with_blinker(-1, -300), open_road());
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "반대로 민 토크는 무시한다";

  run(&helper, 1, with_blinker(-1, 300), open_road(0.9));
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);
  ASSERT_EQ(helper.desire(), Desire::LaneChangeLeft);
  ASSERT_TRUE(helper.changing_lanes());

  run(&helper, 10, with_blinker(-1), open_road(0.9));
  ASSERT_NEAR(helper.lane_change_lane_prob(), 0.0, 1e-9) << "차선선을 0.5초에 뺀다";
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting) << "모델이 변경 중이라 하면 머문다";

  run(&helper, 1, with_blinker(0), open_road(0.0));
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Finishing);
  run(&helper, 10, with_blinker(0), open_road(0.0));
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "0.5초에 되돌리고 깜빡이가 없으면 끝";
  ASSERT_EQ(helper.direction(), 0);
  ASSERT_EQ(helper.desire(), Desire::None);
}

TEST(DesireHelper, BlindSpotHoldsTheStart) {
  DesireHelper helper;
  VehicleCanState vehicle = with_blinker(1);
  run(&helper, 1, vehicle, open_road());
  vehicle.driver_torque = -300;
  vehicle.right_blindspot = true;
  run(&helper, 20, vehicle, open_road());
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "사각지대 차가 있으면 기다린다";
  vehicle.right_blindspot = false;
  run(&helper, 1, vehicle, open_road(0.9));
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);
  ASSERT_EQ(helper.desire(), Desire::LaneChangeRight);
}

TEST(DesireHelper, ChangeTurnsOffAfterTenSeconds) {
  DesireHelper helper;
  run(&helper, 1, with_blinker(-1), open_road());
  run(&helper, 1, with_blinker(-1, 300), open_road(0.9));
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);
  run(&helper, 190, with_blinker(-1), open_road(0.9));
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting) << "9.5초에는 아직 기다린다";
  run(&helper, 20, with_blinker(-1), open_road(0.9));
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "10초가 넘으면 끈다";
  ASSERT_EQ(helper.desire(), Desire::None);
}

TEST(DesireHelper, RoadEdgeBlocksUntilItClears) {
  DesireHelper helper;
  DesireModelInputs edge = open_road();
  edge.road_edge_stds = {1.0, 0.2};             // 오른쪽 경계가 또렷하다
  edge.lane_probabilities = {0.8, 0.9, 0.9, 0.1};  // 그 너머 차선선은 없다
  run(&helper, 10, with_blinker(1), edge);
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "도로 경계 쪽으로는 시작하지 않는다";
  run(&helper, 1, with_blinker(-1), edge);
  ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "반대쪽은 막지 않는다";

  DesireHelper cleared;
  run(&cleared, 10, with_blinker(1), edge);
  run(&cleared, 1, with_blinker(1), open_road());
  ASSERT_EQ(cleared.lane_change_state(), LaneChangeState::PreLaneChange)
      << "경계가 사라지면 깜빡이를 다시 켜지 않아도 시작한다";
}

}  // namespace
