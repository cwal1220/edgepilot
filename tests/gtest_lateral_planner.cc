// 횡 플래너(LateralPlanner): laneless는 openpilot 메인의 get_curvature_from_plan과, 차선 변경은
// 상류 desire_helper와 대조한다. path_offset_m의 적용 범위도 본다. MPC 자체의 최적성은
// gtest_lateral_mpc가 본다.
#include "car/vehicle_can.h"
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "controls/control_params.h"
#include "controls/drive_helpers.h"
#include "planning/lateral_planner.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

// 조향 중(latActive)인 차의 플래너 입력(곡률 0). steering_pushed_input은 운전자가 핸들을 미는 중이다.
LateralPlannerInput steering_input(float v_ego) {
    LateralPlannerInput in;
    in.v_ego = v_ego;
    in.lat_active = true;
    return in;
}
LateralPlannerInput steering_pushed_input(float v_ego) {
    LateralPlannerInput in = steering_input(v_ego);
    in.steering_pressed = true;
    return in;
}

// laneless 모드는 openpilot 메인의 get_curvature_from_plan이다: 모델 plan의 yaw·yaw rate만 쓰고
// plan 위치·경로 오프셋·MPC를 쓰지 않는다.
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
            ms.plan[i] = {v * t, lateral_offset, 0.0f};
            ms.plan_yaw[i] = kappa * v * t;
            ms.plan_yaw_rate[i] = kappa * v;
        }
        return ms;
    };
    VehicleCanState vehicle{};
    // 일정 곡률 plan: 2·ψ(t)/(v·t) − ψ̇/v = 2κ − κ = κ
    LateralTarget turn = planner.update(model_for(0.002f, 0.0f), vehicle, steering_input(v));
    ASSERT_TRUE(turn.valid);
    ASSERT_TRUE(turn.laneless_mode);
    ASSERT_TRUE(turn.mpc_solution_valid);
    EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.0f, 0.34f), 0.002f, 1e-5f) << "일정 곡률 plan은 그 곡률";
    EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.1f, 0.34f), 0.002f, 1e-5f) << "plan 나이와 무관";
    // yaw가 0이면 plan이 옆으로 0.5 m 떨어져 있고 오프셋이 −0.3이어도 목표는 직진이다
    LateralTarget straight = planner.update(model_for(0.0f, 0.5f), vehicle, steering_input(v));
    EXPECT_NEAR(lag_adjusted_curvature(straight, v, 0.05f, 0.34f), 0.0f, 1e-7f)
        << "위치와 경로 오프셋은 laneless 곡률에 들어가지 않는다";
}

// 차선 변경은 openpilot desire_helper와 같다: 깜빡이 + 그 방향 핸들 토크로 시작하고, 모델이
// 끝났다고 할 때(lane_change_prob < 0.02)나 10초·비활성으로만 끝난다. 차선선은 0.5초에 뺀다.
TEST(LateralPlanner, LaneChangeFollowsUpstreamDesireHelper) {
    SteeringParams steering;
    LateralPlanner planner(steering);
    const float v = 20.0f;
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        ms.plan[i] = {v * t, 0.0f, 0.0f};
    }
    ms.desire_state[0] = 1.0f;
    VehicleCanState vehicle{};
    LateralTarget r = planner.update(ms, vehicle, steering_input(v));
    ASSERT_EQ(r.desire, Desire::None);
    vehicle.left_blinker = true;
    r = planner.update(ms, vehicle, steering_input(v));
    EXPECT_EQ(r.desire, Desire::None) << "깜빡이만으로는 시작하지 않는다";
    EXPECT_EQ(r.lane_change_state, LaneChangeState::PreLaneChange) << "HUD: 핸들을 밀기를 기다린다";
    EXPECT_EQ(r.lane_change_direction, -1);
    vehicle.driver_torque = 300;  // 왼쪽으로 민다(컨트롤러가 거른 운전자 조향)
    r = planner.update(ms, vehicle, steering_pushed_input(v));
    EXPECT_EQ(r.desire, Desire::LaneChangeLeft) << "깜빡이 방향으로 밀면 laneChangeLeft";
    EXPECT_EQ(r.lane_change_state, LaneChangeState::Starting) << "HUD: 변경 중";
    // 모델이 차선 변경 중이라고 하는 동안(prob 높음)은 핸들을 놓아도 3초 넘게 이어진다.
    vehicle.driver_torque = 0;
    ms.desire_state[0] = 0.1f;
    ms.desire_state[3] = 0.9f;
    for (int i = 0; i < 60; ++i) r = planner.update(ms, vehicle, steering_input(v));
    EXPECT_EQ(r.desire, Desire::LaneChangeLeft) << "출력이 커져도 취소하지 않는다(예전 출력 0.8 취소 제거)";
    // 모델이 끝났다고 하면 마무리(차선선을 0.5초에 되살림) 동안은 openpilot 0.9.4 DESIRES처럼
    // laneChangeLeft를 유지하고, 끝나면 내린다.
    ms.desire_state[0] = 1.0f;
    ms.desire_state[3] = 0.0f;
    vehicle.left_blinker = false;
    r = planner.update(ms, vehicle, steering_input(v));
    EXPECT_EQ(r.desire, Desire::LaneChangeLeft) << "마무리 단계";
    for (int i = 0; i < 12; ++i) r = planner.update(ms, vehicle, steering_input(v));
    EXPECT_EQ(r.desire, Desire::None) << "차선선을 되살리면 끝난다";
    EXPECT_EQ(r.lane_change_state, LaneChangeState::Off);
}

// 차선 변경 속도 아래에서 켠 깜빡이는 desire를 주지 않는다(상류 desire_helper). 빠를 때 시작한 차선 변경은
// 그 속도 아래로 감속해도 이어진다.
TEST(LateralPlanner, SlowBlinkerGivesNoDesire) {
    SteeringParams steering;
    LateralPlanner planner(steering);
    ModelState ms{};
    ms.valid = 1;
    const float v = 5.0f;  // 18 km/h < 30
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        ms.plan[i] = {v * t, 0.0f, 0.0f};
    }
    for (int l = 0; l < 4; ++l) ms.lane_probabilities[l] = 0.9f;
    ms.desire_state[0] = 1.0f;
    VehicleCanState vehicle{};
    vehicle.left_blinker = true;
    for (int i = 0; i < 60; ++i) {  // 3초
        const LateralTarget r = planner.update(ms, vehicle, steering_input(v));
        ASSERT_EQ(r.desire, Desire::None) << i;
        ASSERT_EQ(r.lane_change_state, LaneChangeState::Off) << i;
    }

    LateralPlanner changing(steering);
    vehicle = VehicleCanState{};
    vehicle.left_blinker = true;
    changing.update(ms, vehicle, steering_input(35.0f / 3.6f));
    vehicle.driver_torque = 300;  // 왼쪽 넛지(컨트롤러가 거른 운전자 조향)
    EXPECT_EQ(changing.update(ms, vehicle, steering_pushed_input(35.0f / 3.6f)).desire, Desire::LaneChangeLeft)
        << "차선 변경 시작";
    vehicle.driver_torque = 0;
    EXPECT_EQ(changing.update(ms, vehicle, steering_input(25.0f / 3.6f)).desire, Desire::LaneChangeLeft)
        << "시작한 변경은 감속해도 이어진다";
}

// path_offset_m은 차선 중심에만 적용된다. 차선이 없어 모델 경로로 넘어가면(교차로) 적용하지
// 않는다: 차 기준인 모델 경로에 더하면 위치 고정점 없이 차가 오프셋 쪽으로 계속 밀린다.
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
            ms.plan[i] = {v * t, 0.0f, 0.0f};
            ms.lanes[1][i] = {-1.75f, 0.0f};
            ms.lanes[2][i] = {1.75f, 0.0f};
        }
        ms.lane_probabilities[1] = ms.lane_probabilities[2] = lane_prob;
        ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
        ms.desire_state[0] = 1.0f;
        return ms;
    };
    VehicleCanState vehicle{};
    LateralTarget r;
    for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.99f), vehicle, steering_input(v));
    ASSERT_FALSE(r.laneless_mode);
    EXPECT_NEAR(r.target_y_m, -0.3f, 0.03f) << "차선이 보이면 차선 중심에서 오프셋만큼";
    for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.0f), vehicle, steering_input(v));
    ASSERT_TRUE(r.laneless_mode);
    EXPECT_NEAR(r.target_y_m, 0.0f, 0.01f) << "모델 경로로 넘어가면 오프셋을 더하지 않는다";
}

// Lane 모드에서 차선이 안 보이면(교차로) laneless 모드와 같은 openpilot 메인 계산으로 목표를 낸다:
// plan 위치가 직진이어도 plan yaw가 곡률 κ를 말하면 목표는 κ다. 차선을 잃는 순간에는 한 프레임에
// 바꾸지 않고 0.5초에 걸쳐 섞는다.
// 깜빡이만 켠 단계(PreLaneChange)는 차선을 빼지 않으므로, 차선이 약하면 평소처럼 모델 경로로 대체한다. 차선을
// 실제로 빼는 단계(변경 중)에서만 대체하지 않는다. 예전에는 깜빡이를 켜면 대체가 꺼져 약한 차선이 경로에 섞였다.
TEST(LateralPlanner, BlinkerAloneKeepsTheModelPathFallback) {
    const float v = 15.0f;  // 54 km/h: 차선 변경 속도 위
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        ms.plan[i] = {v * t, 0.0f, 0.0f};
        ms.plan_yaw[i] = 0.01f * v * t;
        ms.plan_yaw_rate[i] = 0.01f * v;
        ms.lanes[1][i] = {-1.75f, 0.0f};
        ms.lanes[2][i] = {1.75f, 0.0f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = 0.25f;  // 약한 차선
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    ms.desire_state[0] = 1.0f;
    SteeringParams steering;
    LateralPlanner planner(steering);
    VehicleCanState vehicle{};
    LateralTarget r;
    for (int i = 0; i < 40; ++i) r = planner.update(ms, vehicle, steering_input(v));
    ASSERT_TRUE(r.laneless_mode) << "약한 차선이면 모델 경로";
    vehicle.left_blinker = true;
    for (int i = 0; i < 40; ++i) r = planner.update(ms, vehicle, steering_input(v));
    EXPECT_EQ(r.lane_change_state, LaneChangeState::PreLaneChange);
    EXPECT_TRUE(r.laneless_mode) << "깜빡이만으로는 대체를 끄지 않는다";
    vehicle.driver_torque = 300;  // 왼쪽으로 밀어 변경을 시작한다
    r = planner.update(ms, vehicle, steering_pushed_input(v));
    EXPECT_EQ(r.lane_change_state, LaneChangeState::Starting);
    EXPECT_FALSE(r.laneless_mode) << "변경 중에는 대체하지 않는다";
}

TEST(LateralPlanner, LaneModeFallbackMatchesLaneless) {
    const float v = 6.0f;
    const float kappa = 0.02f;
    auto model_for = [&](float lane_prob) {
        ModelState ms{};
        ms.valid = 1;
        for (int i = 0; i < kTrajectorySize; ++i) {
            const float t = model_t_idx(i);
            ms.plan[i] = {v * t, 0.0f, 0.0f};  // 위치는 직진
            ms.plan_yaw[i] = kappa * v * t;    // yaw는 κ로 회전
            ms.plan_yaw_rate[i] = kappa * v;
            ms.lanes[1][i] = {-1.75f, 0.0f};
            ms.lanes[2][i] = {1.75f, 0.0f};
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
    for (int i = 0; i < 60; ++i) r = lane_mode.update(model_for(0.99f), vehicle, steering_input(v));
    ASSERT_FALSE(r.laneless_mode);
    const float with_lanes = lag_adjusted_curvature(r, v, 0.0f, 0.42f);
    EXPECT_LT(std::fabs(with_lanes), 0.3f * kappa) << "차선이 보이면 차선(직진)을 따른다";

    std::vector<float> handoff;
    for (int i = 0; i < 40; ++i) {
        r = lane_mode.update(model_for(0.0f), vehicle, steering_input(v));
        handoff.push_back(lag_adjusted_curvature(r, v, 0.0f, 0.42f));
    }
    ASSERT_TRUE(r.laneless_mode) << "차선을 잃으면 모델 경로 구간";
    const LateralTarget ref = laneless.update(model_for(0.0f), vehicle, steering_input(v));
    EXPECT_NEAR(handoff.back(), lag_adjusted_curvature(ref, v, 0.0f, 0.42f), 1e-6f)
        << "자리 잡은 뒤에는 laneless 모드와 같은 목표";
    EXPECT_NEAR(handoff.back(), kappa, 1e-4f);
    float max_step = 0.0f;
    for (size_t i = 1; i < handoff.size(); ++i) max_step = std::max(max_step, std::fabs(handoff[i] - handoff[i - 1]));
    EXPECT_LT(max_step, 0.25f * kappa) << "인계는 한 프레임에 넘어가지 않는다";
}

// 상류와 다르게 둔 차선 확률 보정: 폭 밴드를 [4.5, 5.5] m로 올렸고(상류 [4, 5], 국내 분기구간의 4.2~5.0 m 폭),
// std 컷오프를 0.4로 늦췄으며(상류 0.3), 유효 확률은 초당 2까지만 바뀐다(모델 경로 대체가 한 프레임에 뒤집히지 않게).
TEST(LateralPlanner, LaneProbabilityAdjustmentsDifferFromUpstream) {
    const float v = 20.0f;
    const auto model_for = [&](float half_width, float std, float prob) {
        ModelState ms{};
        ms.valid = 1;
        for (int i = 0; i < kTrajectorySize; ++i) {
            const float t = model_t_idx(i);
            ms.plan[i] = {v * t, 0.0f, 0.0f};
            ms.lanes[1][i] = {-half_width, 0.0f};
            ms.lanes[2][i] = {half_width, 0.0f};
        }
        ms.lane_probabilities[1] = ms.lane_probabilities[2] = prob;
        ms.lane_stds[1] = ms.lane_stds[2] = std;
        ms.desire_state[0] = 1.0f;
        return ms;
    };
    // 또렷한 차선(폭 3.5 m, std 0.05)으로 Lane 모드에 자리 잡은 뒤 model로 바뀌었을 때 차선을 따르는가
    const auto lane_mode_after = [&](const ModelState &model) {
        LateralPlanner planner{SteeringParams{}};
        LateralTarget r;
        for (int i = 0; i < 40; ++i)
            r = planner.update(model_for(1.75f, 0.05f, 0.9f), VehicleCanState{}, steering_input(v));
        for (int i = 0; i < 40; ++i) r = planner.update(model, VehicleCanState{}, steering_input(v));
        return !r.laneless_mode;
    };
    EXPECT_TRUE(lane_mode_after(model_for(2.4f, 0.05f, 0.9f))) << "4.8 m 폭은 차선을 따른다(상류는 0.2배라 대체)";
    EXPECT_FALSE(lane_mode_after(model_for(2.8f, 0.05f, 0.9f))) << "5.6 m는 차선이 아니다";
    // 대체 판정의 std 보정은 0.25~0.5(블렌드 가중치는 0.15~0.4)
    EXPECT_TRUE(lane_mode_after(model_for(1.75f, 0.3f, 0.9f))) << "std 0.3은 0.8배(0.72, 상류는 0)";
    EXPECT_TRUE(lane_mode_after(model_for(1.75f, 0.4f, 0.9f))) << "std 0.4는 0.4배(0.36)라 대체 문턱 0.3 위에 남는다";
    EXPECT_FALSE(lane_mode_after(model_for(1.75f, 0.45f, 0.9f))) << "std 0.45는 0.2배(0.18)";

    // 확률이 0.9에서 0으로 떨어져도 유효 확률이 초당 2로 내려와 0.3 밑에서야(0.3초 뒤) 대체한다
    LateralPlanner planner{SteeringParams{}};
    LateralTarget r;
    for (int i = 0; i < 40; ++i)
        r = planner.update(model_for(1.75f, 0.05f, 0.9f), VehicleCanState{}, steering_input(v));
    ASSERT_FALSE(r.laneless_mode);
    int frames = 0;
    while (!r.laneless_mode && frames < 40) {
        r = planner.update(model_for(1.75f, 0.05f, 0.0f), VehicleCanState{}, steering_input(v));
        ++frames;
    }
    EXPECT_EQ(frames, 6) << "0.9에서 한 프레임 0.1씩: 여섯 번째 프레임에 0.3 밑";
}

// 모델 경로로 달리다 차선이 다시 보이면 std가 아직 높아도(0.25~0.5) 돌아온다. 판정값이 초당 2로 0에서 올라 0.4를 넘는
// 프레임에 차선으로 바꾼다. 블렌드 밴드(0.15~0.4)로 판정하던 때는 std 0.35인 차선(0.2배, 0.18)으로는 돌아오지 않았다.
TEST(LateralPlanner, ReacquiresLanesWhoseStdIsStillHigh) {
    const float v = 15.0f;
    const auto model_for = [&](float prob, float std) {
        ModelState ms{};
        ms.valid = 1;
        for (int i = 0; i < kTrajectorySize; ++i) {
            ms.plan[i] = {v * model_t_idx(i), 0.0f, 0.0f};
            ms.lanes[1][i] = {-1.75f, 0.0f};
            ms.lanes[2][i] = {1.75f, 0.0f};
        }
        ms.lane_probabilities[1] = ms.lane_probabilities[2] = prob;
        ms.lane_stds[1] = ms.lane_stds[2] = std;
        ms.desire_state[0] = 1.0f;
        return ms;
    };
    // 차선이 없는 교차로(확률 0, std 1)를 지나 std가 std_now인 차선이 나타난 뒤 Lane으로 돌아오기까지 프레임 수(없으면 -1)
    const auto frames_to_lanes = [&](float std_now) {
        LateralPlanner planner{SteeringParams{}};
        LateralTarget r;
        for (int i = 0; i < 40; ++i) r = planner.update(model_for(0.0f, 1.0f), VehicleCanState{}, steering_input(v));
        EXPECT_TRUE(r.laneless_mode);
        for (int frames = 1; frames <= 40; ++frames) {
            r = planner.update(model_for(0.9f, std_now), VehicleCanState{}, steering_input(v));
            if (!r.laneless_mode) return frames;
        }
        return -1;
    };
    EXPECT_EQ(frames_to_lanes(0.05f), 5) << "또렷한 차선: 0.1씩 올라 다섯 번째 프레임에 0.5";
    EXPECT_EQ(frames_to_lanes(0.35f), 5) << "std 0.35는 0.6배(0.54)";
    EXPECT_EQ(frames_to_lanes(0.45f), -1) << "std 0.45는 0.2배(0.18)라 모델 경로에 남는다";
}

// 모델 차선 std는 차선이 또렷해도 0.4를 잠깐 넘는다. 블렌드 가중치는 그 프레임에 내려가지만, 모델 경로 대체는
// std가 0.5초(10프레임) 넘게 높을 때만 한다(판정은 최근 10프레임 std의 최솟값을 본다).
TEST(LateralPlanner, BriefLaneStdSpikeKeepsTheLanes) {
    const float v = 20.0f;
    const auto model_for = [&](float std) {
        ModelState ms{};
        ms.valid = 1;
        for (int i = 0; i < kTrajectorySize; ++i) {
            ms.plan[i] = {v * model_t_idx(i), 0.0f, 0.0f};
            ms.lanes[1][i] = {-1.75f, 0.0f};
            ms.lanes[2][i] = {1.75f, 0.0f};
        }
        ms.lane_probabilities[1] = ms.lane_probabilities[2] = 0.9f;
        ms.lane_stds[1] = ms.lane_stds[2] = std;
        ms.desire_state[0] = 1.0f;
        return ms;
    };
    LateralPlanner planner{SteeringParams{}};
    LateralTarget r;
    for (int i = 0; i < 40; ++i) r = planner.update(model_for(0.05f), VehicleCanState{}, steering_input(v));
    ASSERT_FALSE(r.laneless_mode);
    for (int i = 0; i < 9; ++i) {
        r = planner.update(model_for(0.6f), VehicleCanState{}, steering_input(v));
        ASSERT_FALSE(r.laneless_mode) << "frame " << i;
    }
    EXPECT_LT(r.lane_d_prob, 0.1f) << "차선 가중치는 그 프레임의 std로 내려간다";
    for (int i = 0; i < 20; ++i) r = planner.update(model_for(0.05f), VehicleCanState{}, steering_input(v));
    int frames = 0;
    while (!r.laneless_mode && frames < 40) {
        r = planner.update(model_for(0.6f), VehicleCanState{}, steering_input(v));
        ++frames;
    }
    EXPECT_EQ(frames, 15) << "창이 높은 std로 다 차는 10번째 프레임부터 초당 2로 내려가 15번째 프레임에 0.3 밑";
}

// lane_path_weight는 Lane 모드 MPC가 차선 중심에서 벗어난 위치를 되돌리는 세기다. 차가 차선 중심에서
// 0.3 m 오른쪽에 있으면 가중치 3이 1보다 왼쪽(음) 곡률을 더 크게 요구한다.
// Lane MPC는 직전 해의 곡률에 잰 곡률을 0.25만큼 섞어 출발한다(0.7.x는 잰 곡률, 0.8.x는 직전 해). 곧은 차로 가운데서
// 자리 잡은 뒤 차가 0.01 1/m로 돈다고 재면 다음 해의 첫 곡률은 0.0025다. 잰 곡률이 직전 해와 같으면 그대로다.
TEST(LateralPlanner, MpcStartsPartlyFromTheMeasuredCurvature) {
    const float v = 20.0f;
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
        ms.plan[i] = {v * model_t_idx(i), 0.0f, 0.0f};
        ms.lanes[1][i] = {-1.75f, 0.0f};
        ms.lanes[2][i] = {1.75f, 0.0f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = 0.9f;
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    ms.desire_state[0] = 1.0f;
    LateralPlanner planner{SteeringParams{}};
    LateralTarget r;
    for (int i = 0; i < 40; ++i) r = planner.update(ms, VehicleCanState{}, steering_input(v));
    ASSERT_FALSE(r.laneless_mode);
    ASSERT_NEAR(r.curvatures[0], 0.0f, 1e-6f) << "곧은 차로 가운데: 곡률 0에서 출발";
    LateralPlannerInput turning = steering_input(v);
    turning.measured_curvature = 0.01f;
    r = planner.update(ms, VehicleCanState{}, turning);
    EXPECT_NEAR(r.curvatures[0], 0.0025f, 1e-6f);
}

TEST(LateralPlanner, LanePathWeightStrengthensCentering) {
    const float v = 20.0f;
    auto model_for = [&]() {
        ModelState ms{};
        ms.valid = 1;
        for (int i = 0; i < kTrajectorySize; ++i) {
            const float t = model_t_idx(i);
            ms.plan[i] = {v * t, 0.0f, 0.0f};
            ms.lanes[1][i] = {-2.05f, 0.0f};  // 차가 차선 중심보다 0.3 m 오른쪽
            ms.lanes[2][i] = {1.45f, 0.0f};
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
        for (int i = 0; i < 40; ++i) r = planner.update(model_for(), vehicle, steering_input(v));
        EXPECT_FALSE(r.laneless_mode);
        return lag_adjusted_curvature(r, v, 0.0f, 0.42f);
    };
    const float upstream = desired(1.0f);
    const float stronger = desired(3.0f);
    EXPECT_LT(upstream, 0.0f) << "차선 중심(왼쪽)으로 돌아간다";
    EXPECT_LT(stronger, upstream * 1.2f) << "가중치 3은 더 강하게 되돌린다";
}

}  // namespace
