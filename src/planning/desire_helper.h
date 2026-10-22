#pragma once

// openpilot desire_helper(0.9.4 차선선 페이드 포함)의 이식. 깜빡이·운전자 조향·사각지대·도로 경계와
// 모델의 차선 변경 확률로 차선 변경 단계와 모델에 줄 desire를 정한다. LateralPlanner가 모델 프레임(20 Hz)마다
// 부른다. 상류에 더해, 깜빡이를 켠 회전(핸들을 크게 돌리거나 회전하려고 감속)은 차선 변경으로 시작하지 않고
// 모델에 회전 desire를 주며, 깜빡이 한 번에 차선 변경은 한 번이고, 운전자가 반대로 밀면 취소한다.

#include "controls/lateral_target.h"  // LaneChangeState, Desire

#include <array>

struct VehicleCanState;

struct DesireHelperParams {
    int steering_pressed_threshold = 150;  // 컨트롤러와 같은 운전자 토크 임계값(변경 시작 넛지)
    double lane_change_min_speed_mps = 30.0 / 3.6;
};

// 모델 프레임에서 desire 판단에 쓰는 값.
struct DesireModelInput {
    double lane_change_prob = 0.0;               // desire_state 차선 변경(좌+우)
    std::array<double, 4> lane_probabilities{};  // 차선선 4개
    std::array<double, 2> road_edge_stds{};      // 도로 경계 2개
};

class DesireHelper {
public:
    void update_params(const DesireHelperParams &params) { params_ = params; }
    // lat_active: 지금 조향하는가(상류 carControl.latActive). steering_pressed: 컨트롤러가 거른 운전자 조향
    // (상류 CS.steeringPressed)으로, 변경을 취소할 때 쓴다. 변경 시작 넛지는 이 틱의 운전자 토크로 본다.
    void update(const VehicleCanState &vehicle, float v_ego, bool lat_active, bool steering_pressed,
                const DesireModelInput &model);

    Desire desire() const { return desire_; }
    LaneChangeState lane_change_state() const { return lane_change_state_; }
    int direction() const { return direction_; }  // -1 왼쪽, 1 오른쪽, 0 없음
    // 차선 변경 중 차선선에 곱하는 확률(페이드 아웃·인)
    double lane_change_lane_prob() const { return lane_change_lane_prob_; }
    bool changing_lanes() const { return desire_ == Desire::LaneChangeLeft || desire_ == Desire::LaneChangeRight; }
    // 깜빡이를 켜고 회전하는 중(모델에 회전 desire를 준다). -1 왼쪽, 1 오른쪽, 0 아님
    int turn_direction() const { return turn_direction_; }
    bool turning() const { return turn_direction_ != 0; }

private:
    double record_speed(float v_ego);
    // 깜빡이 쪽으로 크게 돌린 회전의 시작과 끝(turn_direction_)
    void update_turn(float steering_angle_deg, bool one_blinker, int blinker_direction);
    // 조향 중이고 막히지 않았을 때 차선 변경 단계 하나(Off → PreLaneChange → Starting → Finishing)
    void step_lane_change(const VehicleCanState &vehicle, float v_ego, bool steering_pressed,
                          const DesireModelInput &model, bool one_blinker, int blinker_direction, bool below_speed,
                          double decel);

    DesireHelperParams params_;
    LaneChangeState lane_change_state_ = LaneChangeState::Off;
    int direction_ = 0;
    Desire desire_ = Desire::None;
    bool previous_one_blinker_ = false;
    double lane_change_lane_prob_ = 1.0;
    double lane_change_timer_ = 0.0;
    // 이 쪽 깜빡이(-1 왼쪽, 1 오른쪽)로는 차선 변경을 하지 않는다(이미 했거나, 취소했거나, 그 깜빡이로 회전했다).
    // 깜빡이를 끄거나 반대쪽으로 바꾸면 푼다. 0이면 막지 않는다.
    int used_blinker_ = 0;
    // 깜빡이를 켤 때마다 1씩 는다. 변경을 시작한 켜짐이 그대로면 마친 뒤 막고, 그 사이 껐다 켰으면 새 신호로 본다.
    unsigned blinker_activation_ = 0;
    unsigned change_activation_ = 0;
    bool one_blinker_last_ = false;
    // 변경 중 운전자가 반대쪽으로 민 연속 프레임
    int against_frames_ = 0;
    // 회전 중인 쪽(-1 왼쪽, 1 오른쪽, 0 아님)과 회전한 프레임 수. 15초 상한으로 거뒀으면 핸들이 돌아오거나 깜빡이를
    // 끌 때까지 turn_expired_다.
    int turn_direction_ = 0;
    int turn_frames_ = 0;
    bool turn_expired_ = false;
    // 최근 1초의 속도(감속 판단). 모델 프레임마다 하나
    std::array<float, 21> speeds_{};
    int speed_count_ = 0;
    int speed_head_ = 0;
};
