#include "planning/desire_helper.h"

#include "common/utils_time.h"  // kDtMdl

#include "car/vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <tuple>

namespace {

// 깜빡이를 켠 채 핸들을 이만큼 돌리면 회전이다. 차선 변경은 훨씬 덜 돌린다: 2026-10-14·15 주행에서 차선 변경은
// 최대 29도였고, 차선 변경으로 시작해 버린 교차로 회전 4번은 48~128도까지 돌렸다.
constexpr float kTurnAngleDeg = 45.0f;
// 이 속도 밑에서 1초 동안 이만큼 넘게 감속 중이면 회전하려고 서행하는 것으로 보고 차선 변경 시작을 미룬다. 같은
// 주행에서 잘못 시작한 회전은 시작 직전 1초에 0.8~1.3 m/s²를 줄였고, 55 km/h 밑의 차선 변경은 0.4 m/s² 이하였다.
constexpr double kTurnApproachSpeedMps = 55.0 / 3.6;
constexpr double kTurnApproachDecelMps2 = 0.75;
// 변경 중 운전자가 반대쪽으로 이만큼 이어서 밀면 취소한다(carrotpilot desire_helper와 같은 취소)
constexpr int kAgainstFrames = 4;  // 0.2초
// 회전 desire: 깜빡이 쪽으로 kTurnAngleDeg 넘게 돌리면 주고, 핸들이 이 각도 아래로 돌아오면(회전을 마치면) 거둔다.
// 운전자가 넘겨받은 큰 조향각 정지가 15도에서 풀리는 것과 같은 각도다.
constexpr float kTurnEndAngleDeg = 15.0f;
// 이보다 오래 돌면 거둔다(깜빡이를 켠 채 둥근 길을 계속 도는 경우). 거둔 회전은 핸들이 돌아오거나 깜빡이를 끌
// 때까지 다시 주지 않는다: 예전에는 다음 틱에 곧바로 다시 회전이 되어 상한이 한 틱 끊김에 그쳤다.
constexpr double kTurnMaxS = 15.0;
// 모델 desire는 바뀐 틱의 펄스가 5초 이력에 남는다. 회전이 길면 2.5초마다 한 틱 내렸다가 다시 준다.
constexpr int kTurnRepulseFrames = 50;

}  // namespace

// 최근 1초 감속(m/s², 감속이 양수). 기록이 1초 안 되면 0.
double DesireHelper::record_speed(float v_ego) {
    constexpr int n = static_cast<int>(std::tuple_size<decltype(speeds_)>::value);
    speed_head_ = (speed_head_ + 1) % n;
    speeds_[speed_head_] = v_ego;
    speed_count_ = std::min(speed_count_ + 1, n);
    if (speed_count_ < n) return 0.0;
    const float second_ago = speeds_[(speed_head_ + 1) % n];
    return (second_ago - v_ego) / ((n - 1) * kDtMdl);
}

// openpilot desire_helper(0.9.4 차선선 페이드 포함)와 같다. 꺼지는 조건은 조향 비활성과 10초 초과다. 예전에는
// 포크에서 온 두 가지가 더 있었다: 출력 0.8 이상이 0.5초 이어지면 차선 변경 취소(차선이 차를 붙잡아 운전자와
// 싸우는 바로 그 순간 취소됐다), 속도별로 느린 차선선 페이드(60 km/h에서 2.5초). 2026-09-27 실차에서 운전자가
// 핸들을 한참 잡아야 해서 둘 다 upstream으로 되돌렸다. 그 위에 회전 판단, 깜빡이 한 번에 한 번, 반대로 밀면
// 취소를 더한다: 깜빡이를 켜고 교차로를 돌면 회전 쪽으로 미는 핸들이 차선 변경을 시작했다(2026-10-14·15, 27번
// 중 4번). 회전으로 보면 차선 변경 대신 회전 desire를 줘 모델 경로가 회전을 따르게 한다. 2026-10-15 좌회전
// 출구에서 커브 쪽으로 이어진 토크가 안정적이었는데, 잘못 시작한 차선 변경 desire가 경로를 회전 쪽으로 휘게
// 만든 우연이었다.
// 깜빡이 쪽으로 크게 돌리면 회전이다(조향각은 왼쪽이 +). 도중에 깜빡이를 꺼도 회전은 이어지므로 각도로만 끝낸다.
void DesireHelper::update_turn(float steering_angle_deg, bool one_blinker, int blinker_direction) {
    if (turn_direction_ == 0) {
        if (turn_expired_ && (!one_blinker || std::fabs(steering_angle_deg) < kTurnEndAngleDeg)) turn_expired_ = false;
        if (!turn_expired_ && one_blinker && -blinker_direction * steering_angle_deg >= kTurnAngleDeg) {
            turn_direction_ = blinker_direction;
            turn_frames_ = 0;
        }
    } else {
        ++turn_frames_;
        if (std::fabs(steering_angle_deg) < kTurnEndAngleDeg) {
            turn_direction_ = 0;
        } else if (turn_frames_ * kDtMdl > kTurnMaxS) {
            turn_direction_ = 0;
            turn_expired_ = true;
        }
    }
}

namespace {

// 도로 경계가 붙은 쪽(-1 왼쪽, 1 오른쪽, 0 없음): 경계는 또렷하고 그 너머 차선선은 없다(dragonpilot의 도로 경계 판정)
int road_edge_side(const DesireModelInput &model) {
    const double left_edge_prob = std::clamp(1.0 - model.road_edge_stds[0], 0.0, 1.0);
    const double right_edge_prob = std::clamp(1.0 - model.road_edge_stds[1], 0.0, 1.0);
    const double left_nearside_prob = model.lane_probabilities[0];
    const double right_nearside_prob = model.lane_probabilities[3];
    return right_edge_prob > 0.35 && right_nearside_prob < 0.2 && left_nearside_prob >= right_nearside_prob ? 1
           : left_edge_prob > 0.35 && left_nearside_prob < 0.2 && right_nearside_prob >= left_nearside_prob ? -1
                                                                                                            : 0;
}

}  // namespace

void DesireHelper::update(const VehicleCanState &vehicle, float v_ego, bool lat_active, bool steering_pressed,
                          const DesireModelInput &model) {
    const bool one_blinker = vehicle.left_blinker != vehicle.right_blinker;
    const bool below_speed = v_ego < params_.lane_change_min_speed_mps;
    const int blinker_direction = vehicle.left_blinker ? -1 : 1;
    const double decel = record_speed(v_ego);
    if (one_blinker && !one_blinker_last_) ++blinker_activation_;
    one_blinker_last_ = one_blinker;
    if (!one_blinker || used_blinker_ != blinker_direction) used_blinker_ = 0;
    if (one_blinker && std::fabs(vehicle.steering_angle_deg) >= kTurnAngleDeg) used_blinker_ = blinker_direction;
    const bool blinker_used = used_blinker_ != 0;
    update_turn(vehicle.steering_angle_deg, one_blinker, blinker_direction);

    // 깜빡이 쪽이 도로 경계면 차선 변경을 시작하지 않는다. 막힌 동안 깜빡이를 꺼진 것으로 두므로, 경계가 사라지면
    // 깜빡이를 켠 채로도 그 틱에 시작한다. 기다리는 동안(PreLaneChange)에도 본다: 경계가 한 프레임 끊겨 기다림에 들어간
    // 뒤 경계가 돌아와도 밀면 경계 쪽으로 시작했다.
    const int lane_direction = vehicle.left_blinker ? -1 : vehicle.right_blinker ? 1 : 2;
    const bool road_edge_blocked =
        (lane_change_state_ == LaneChangeState::Off || lane_change_state_ == LaneChangeState::PreLaneChange) &&
        road_edge_side(model) == lane_direction;

    if (road_edge_blocked || !lat_active || lane_change_timer_ > 10.0 || blinker_used) {
        lane_change_state_ = LaneChangeState::Off;
        direction_ = 0;
    } else {
        step_lane_change(vehicle, v_ego, steering_pressed, model, one_blinker, blinker_direction, below_speed, decel);
    }
    if (lane_change_state_ != LaneChangeState::Starting) against_frames_ = 0;

    const bool changing =
        lane_change_state_ == LaneChangeState::Starting || lane_change_state_ == LaneChangeState::Finishing;
    lane_change_timer_ = changing ? lane_change_timer_ + kDtMdl : 0.0;
    // 상류: 조향하지 않는 동안 켜 둔 깜빡이는 조향이 다시 켜지면 새로 켠 것으로 본다
    previous_one_blinker_ = road_edge_blocked ? false : one_blinker && lat_active;
    desire_ = changing && direction_ == -1  ? Desire::LaneChangeLeft
              : changing && direction_ == 1 ? Desire::LaneChangeRight
                                            : Desire::None;
    if (turn_direction_ != 0) {
        const bool repulse = turn_frames_ % kTurnRepulseFrames == kTurnRepulseFrames - 1;
        desire_ = repulse ? Desire::None : turn_direction_ < 0 ? Desire::TurnLeft : Desire::TurnRight;
    }
}

void DesireHelper::step_lane_change(const VehicleCanState &vehicle, float v_ego, bool steering_pressed,
                                    const DesireModelInput &model, bool one_blinker, int blinker_direction,
                                    bool below_speed, double decel) {
    const bool toward =
        (vehicle.driver_torque > 0 && direction_ == -1) || (vehicle.driver_torque < 0 && direction_ == 1);
    const bool against =
        (vehicle.driver_torque > 0 && direction_ == 1) || (vehicle.driver_torque < 0 && direction_ == -1);
    // 시작 넛지는 이 틱의 토크로 본다. 상류의 거른 steeringPressed(6틱 연속)를 쓰면 짧게 툭 친 넛지(30~40 ms)가
    // 시작하지 않았다(녹화 재생 130번 중 4번). 취소는 거른 값으로 본다.
    const bool torque_applied = std::abs(vehicle.driver_torque) > params_.steering_pressed_threshold && toward;
    // 상류의 사각지대(LCA11) 막음은 없다: K7은 Panda 버스에 LCA11을 내지 않는다
    const bool slowing_for_turn = v_ego < kTurnApproachSpeedMps && decel > kTurnApproachDecelMps2;
    if (lane_change_state_ == LaneChangeState::Off && one_blinker && !previous_one_blinker_ && !below_speed) {
        lane_change_state_ = LaneChangeState::PreLaneChange;
        direction_ = blinker_direction;
        lane_change_lane_prob_ = 1.0;
    } else if (lane_change_state_ == LaneChangeState::PreLaneChange) {
        if (!one_blinker || below_speed) {
            lane_change_state_ = LaneChangeState::Off;
        } else {
            direction_ = blinker_direction;  // 상류: 기다리는 동안 깜빡이 방향을 다시 읽는다
            if (!slowing_for_turn && torque_applied) {
                lane_change_state_ = LaneChangeState::Starting;
                change_activation_ = blinker_activation_;
            }
        }
    } else if (lane_change_state_ == LaneChangeState::Starting) {
        against_frames_ = steering_pressed && against ? against_frames_ + 1 : 0;
        if (against_frames_ >= kAgainstFrames) {
            used_blinker_ = one_blinker ? blinker_direction : 0;
            lane_change_state_ = LaneChangeState::Off;
            direction_ = 0;
        } else {
            // 0.5초에 걸쳐 차선선을 뺀다(openpilot "fade out over .5s").
            lane_change_lane_prob_ = std::max(0.0, lane_change_lane_prob_ - 2.0 * kDtMdl);
            if (model.lane_change_prob < 0.02 && lane_change_lane_prob_ < 0.01)
                lane_change_state_ = LaneChangeState::Finishing;
        }
    } else if (lane_change_state_ == LaneChangeState::Finishing) {
        // 복구 0.5초 (openpilot 기본 1.0초). 변경 직후 새 차선 적응을 당긴다.
        lane_change_lane_prob_ = std::min(1.0, lane_change_lane_prob_ + 2.0 * kDtMdl);
        if (lane_change_lane_prob_ > 0.99) {
            // 상류는 깜빡이가 켜져 있으면 다음 변경을 기다린다. 여기서는 깜빡이를 껐다 켜야 다시 한다:
            // 램프나 회전 출구에서 켜 둔 깜빡이가 4.5초마다 차선 변경을 다시 시작했다(2026-10-05, 10-15).
            // 마친 쪽 깜빡이를 켜 둔 채면 막는다. 끝나 가는 동안 반대쪽으로 바꿨으면 상류처럼 그쪽으로 기다린다
            // (깜빡이가 꺼진 틈이 없어 새로 켠 것으로 잡히지 않는다).
            if (one_blinker && blinker_direction != direction_) {
                lane_change_state_ = LaneChangeState::PreLaneChange;
                direction_ = blinker_direction;
            } else {
                // 변경 중 깜빡이를 껐다 다시 켰으면 한 번 더 바꾸라는 신호라 막지 않는다(상류처럼 기다린다)
                const bool same_signal = one_blinker && blinker_activation_ == change_activation_;
                if (same_signal) used_blinker_ = blinker_direction;
                lane_change_state_ =
                    one_blinker && !same_signal ? LaneChangeState::PreLaneChange : LaneChangeState::Off;
                direction_ = lane_change_state_ == LaneChangeState::PreLaneChange ? blinker_direction : 0;
            }
        }
    }
}
