#pragma once

/* openpilot desire_helper(0.9.4 차선선 페이드 포함)의 이식. 깜빡이·운전자 토크·사각지대·도로 경계와
 * 모델의 차선 변경 확률로 차선 변경 단계와 모델에 줄 desire를 정한다. LateralPlanner가 모델 프레임(20 Hz)마다
 * 부른다. */

#include <array>

struct VehicleCanState;

// 차선 변경 단계(상류 LaneChangeState). 정수 값은 LateralTarget·HUD 플래그에 그대로 쓰인다.
enum class LaneChangeState : int {
  Off = 0,
  PreLaneChange = 1,  // 깜빡이를 켰고 운전자가 핸들을 밀기를 기다린다
  Starting = 2,       // 변경 중: 차선선을 빼며 모델 desire를 준다
  Finishing = 3,      // 끝나 가는 중: 차선선을 되돌린다
};

// 모델 desire 입력(0.9.4·master DESIRES).
enum class Desire : int {
  None = 0,
  TurnLeft = 1,
  TurnRight = 2,
  LaneChangeLeft = 3,
  LaneChangeRight = 4,
};

struct DesireHelperParams {
  int steering_pressed_threshold = 150;      // 컨트롤러와 같은 운전자 토크 임계값
  double lane_change_min_speed_mps = 30.0 / 3.6;
};

// 모델 프레임에서 desire 판단에 쓰는 값.
struct DesireModelInputs {
  double lane_change_prob = 0.0;              // desire_state 차선 변경(좌+우)
  std::array<double, 4> lane_probabilities{}; // 차선선 4개
  std::array<double, 2> road_edge_stds{};     // 도로 경계 2개
};

class DesireHelper {
public:
  void update_params(const DesireHelperParams &params) { params_ = params; }
  void update(const VehicleCanState &vehicle, float v_ego, bool active, const DesireModelInputs &model);

  Desire desire() const { return desire_; }
  LaneChangeState lane_change_state() const { return lane_change_state_; }
  int direction() const { return direction_; }  // -1 왼쪽, 1 오른쪽, 0 없음
  // 차선 변경 중 차선선에 곱하는 확률(페이드 아웃·인)
  double lane_change_lane_prob() const { return lane_change_lane_prob_; }
  bool changing_lanes() const {
    return desire_ == Desire::LaneChangeLeft || desire_ == Desire::LaneChangeRight;
  }

private:
  DesireHelperParams params_;
  LaneChangeState lane_change_state_ = LaneChangeState::Off;
  int direction_ = 0;
  Desire desire_ = Desire::None;
  bool previous_one_blinker_ = false;
  double lane_change_lane_prob_ = 1.0;
  double lane_change_timer_ = 0.0;
};
