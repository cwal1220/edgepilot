// 토크 횡제어기(TorqueController, openpilot LatControlTorque 이식): 넘겨받은 조향 지연만큼의 요청
// 버퍼, 속도별 비례 이득(KP_INTERP), 라이브 뱅크·latAccelOffset feedforward, 그리고 학습값 소비.
// 실제 곡률은 opendbc calc_curvature의 독립 전사본과, 출력 구조는 상류의 횡가속 공간 PID와 대조한다.
#include "car/car_specs.h"
#include "controls/control_params.h"
#include "controls/lateral_torque.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

// 토크 제어기 입력: 앞쪽 다섯과 편경사·학습값. 핸들 조작·안전 한계·요레이트는 기본값(없음)이다.
TorqueController::Input torque_in(bool active, float speed_mps, float curvature, float angle_deg, float delay_s,
                                  float bank = 0.0f, const LiveLateralParams &live = LiveLateralParams{}) {
    TorqueController::Input in;
    in.active = active;
    in.speed_mps = speed_mps;
    in.desired_curvature = curvature;
    in.steering_angle_deg = angle_deg;
    in.lat_delay_s = delay_s;
    in.road_bank_lat_accel = bank;
    in.live = live;
    return in;
}

// v0.11식 지연 보정: 요청 스텝 직후 delay 동안은 P가 과거 요청(0)과 현재
// 측정(0)을 비교해 오차가 없어야 하고, 토크는 FF만으로 나와야 한다.
TEST(LateralTorque, DelayCompensatedError) {
    TorqueController torque;
    SteeringParams params;
    params.enabled = true;
    params.steer_actuator_delay = 0.30f;
    // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
    params.angle_offset_deg = 0.0f;
    const float v = 20.0f;

    // 요청 0으로 버퍼를 채운다
    for (int i = 0; i < 120; ++i) torque.update(torque_in(true, v, 0.0f, 0.0f, params.steer_actuator_delay), params);
    ASSERT_LT(std::fabs(torque.error()), 1e-6f) << "0 요청이 이어지면 오차가 없다";

    // 곡률 스텝. 조향각은 아직 0(차가 반응 전).
    torque.update(torque_in(true, v, 0.01f, 0.0f, params.steer_actuator_delay), params);
    ASSERT_LT(std::fabs(torque.error()), 1e-4f) << "스텝 직후 오차는 거의 0이다(지연 보상)";
    ASSERT_GT(torque.feedforward(), 1.0f) << "스텝은 feedforward가 바로 싣는다";

    // delay(31프레임)를 넘겨도 차가 반응하지 않으면 그때 오차가 나타난다
    for (int i = 0; i < 40; ++i) torque.update(torque_in(true, v, 0.01f, 0.0f, params.steer_actuator_delay), params);
    ASSERT_GT(torque.error(), 1.0f) << "지연이 지나도 못 따라간 만큼은 오차로 나타난다";
}

// inactive 동안에도 요청 버퍼가 갱신되어야 다시 결합할 때 낡은 요청과 비교되지 않는다.
TEST(LateralTorque, ReengageHasNoStaleBufferSpike) {
    TorqueController torque;
    SteeringParams params;
    params.enabled = true;
    params.steer_actuator_delay = 0.30f;
    // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
    params.angle_offset_deg = 0.0f;
    const float v = 20.0f;

    // 커브 요청으로 버퍼를 채운 뒤 해제
    for (int i = 0; i < 120; ++i) torque.update(torque_in(true, v, 0.01f, 0.0f, params.steer_actuator_delay), params);
    // inactive 동안 요청은 0으로 돌아간다 (직선 수동 주행)
    for (int i = 0; i < 120; ++i) torque.update(torque_in(false, v, 0.0f, 0.0f, params.steer_actuator_delay), params);
    // 직선에서 다시 결합: 버퍼가 신선하면 오차 ~0, 얼었다면 큰 스파이크
    torque.update(torque_in(true, v, 0.0f, 0.0f, params.steer_actuator_delay), params);
    ASSERT_LT(std::fabs(torque.error()), 1e-4f) << "다시 결합할 때 해제 전의 낡은 요청과 비교하지 않는다";
}

// 85도 일시정지가 풀릴 때처럼, 쉬는 동안(LateralController가 측정 곡률을 요청으로 넣는다) 핸들이 45도에서 풀리다가
// 조향을 다시 시작하면 지연 틱 수 동안의 비교 대상은 그때의 곡률이라 오차가 회전 쪽으로 크다. 그 동안은 오차를 쓰지
// 않는다: 다시 시작할 때 조향각만 다른 두 제어기의 토크가 지연 동안 같고, 그 뒤에는 갈린다.
TEST(LateralTorque, RestartUsesFeedforwardOnlyForTheDelay) {
    SteeringParams params;
    params.enabled = true;
    params.angle_offset_deg = 0.0f;
    const float v = 8.0f, delay = 0.5f;
    const int delay_frames = static_cast<int>(delay / 0.01f) + 1;
    TorqueController a, b;
    float angle = 45.0f;
    for (int i = 0; i < 150; ++i) {
        if (i >= 100) angle = 45.0f - 30.0f * static_cast<float>(i - 99) / 50.0f;  // 마지막 0.5초에 15도까지 푼다
        const float measured = VehicleModel::make(params).curvature_at_angle(v, angle);
        a.update(torque_in(false, v, measured, angle, delay), params);
        b.update(torque_in(false, v, measured, angle, delay), params);
    }
    int first_split = -1;
    for (int i = 0; i < 2 * delay_frames; ++i) {
        // 계획은 직진. b만 조향각이 다르다(측정만 다른 같은 기록).
        const int ta = a.update(torque_in(true, v, 0.0f, 15.0f, delay), params);
        const int tb = b.update(torque_in(true, v, 0.0f, 5.0f, delay), params);
        if (i == 0) ASSERT_GT(std::fabs(a.error()), 0.5f) << "비교 대상은 핸들을 풀기 전의 곡률이다";
        // 그 오차를 쓰면 P가 회전 쪽 최대 토크로 민다. 피드포워드(직진 요청, 편향, 저크 마찰)만이면 작다.
        if (i < delay_frames - 1) EXPECT_LT(std::fabs(a.normalized_output()), 0.2f) << "tick " << i;
        if (first_split < 0 && ta != tb) first_split = i;
    }
    EXPECT_EQ(first_split, delay_frames - 1) << "지연 동안은 측정을 보지 않고 그 뒤부터 오차로 따른다";
}

// 토크 컨트롤러는 넘겨받은 조향 지연(상류 lat_delay)만큼 전의 요청을 지금 측정과 비교한다:
// 계단 요청이 오차로 나타나는 틱이 지연을 따른다(예전에는 늘 steer_actuator_delay).
TEST(LateralTorque, TorqueSetpointFollowsLateralDelay) {
    SteeringParams params;
    params.enabled = true;
    params.steer_actuator_delay = 0.34f;
    params.angle_offset_deg = 0.0f;  // 조향각 0이면 측정 곡률 0
    for (const float delay : {0.34f, 0.5f}) {
        TorqueController torque;
        int first = -1;
        for (int i = 0; i < 150; ++i) {
            const float desired = i >= 20 ? 0.002f : 0.0f;
            torque.update(torque_in(true, 20.0f, desired, 0.0f, delay), params);
            if (first < 0 && std::fabs(torque.error()) > 1e-6f) first = i - 20;
        }
        EXPECT_EQ(first, static_cast<int>(delay / 0.01f)) << delay;
    }
}

// 속도별 비례 이득(openpilot KP_INTERP). 오차는 횡가속도만으로 내고 저속 보강은
// 이 곡선이 맡는다. 곡률 오차 항(LOW_SPEED_Y)은 더 이상 없다.
TEST(LateralTorque, KpSpeedSchedule) {
    SteeringParams base;
    base.enabled = true;
    base.angle_offset_deg = 0.0f;
    base.torque_friction = 0.0f;          // P항만 남긴다
    base.torque_lat_accel_offset = 0.0f;  // 오차가 순수 횡가속도인지 본다
    base.torque_ki = 0.0f;
    // 조향각 3도를 실제 곡률로 두고 요청 곡률 0을 준다. 오차 = -actual_lat_accel이라
    // 출력은 kp(v) x kf x v^2 x |actual_curvature|에 비례한다.
    const auto run = [&](float v, float kp) {
        SteeringParams p = base;
        p.torque_kp = kp;
        TorqueController torque;
        for (int i = 0; i < 120; ++i) torque.update(torque_in(true, v, 0.0f, 3.0f, p.steer_actuator_delay), p);
        return torque.normalized_output();
    };
    // 이득 곡선의 노드에서 출력비가 KP_INTERP 비율 x v^2 비율과 맞아야 한다.
    const auto gain_at = [&](float v) {
        const float out = run(v, 0.8f);
        TorqueController probe;
        SteeringParams p = base;
        for (int i = 0; i < 120; ++i) probe.update(torque_in(true, v, 0.0f, 3.0f, p.steer_actuator_delay), p);
        return std::fabs(out / (probe.error() == 0.0f ? 1.0f : probe.error()));
    };
    // 5 m/s 노드는 11.5, 10 m/s 노드는 3.5 -> 이득비 3.2857
    const float g5 = gain_at(5.0f), g10 = gain_at(10.0f);
    // 두 속도 노드 모두에서 이득이 나온다
    ASSERT_GT(g5, 0.0f);
    ASSERT_GT(g10, 0.0f);
    const float ratio = g5 / g10;
    // 5 m/s 노드는 10 m/s 노드의 11.5/3.5배
    ASSERT_GT(ratio, 3.2f);
    ASSERT_LT(ratio, 3.4f);
    // 30 m/s 위는 torque_kp가 그대로 끝점이다.
    const float top8 = std::fabs(run(35.0f, 0.8f));
    const float top16 = std::fabs(run(35.0f, 1.6f));
    // 커브 정점 경우는 포화하지 않는다
    ASSERT_GT(top8, 1e-4f);
    ASSERT_LT(top8, 0.95f);
    ASSERT_NEAR(top16 / top8, 2.0f, 0.05f) << "30 m/s 위에서는 이득이 torque_kp에 비례한다";
    // LOW_SPEED_Y가 남아 있으면 곡률 항이 저속에서 오차를 수십 배로 키운다.
    // 오차가 순수 횡가속도인지 확인한다.
    TorqueController t;
    SteeringParams p = base;
    for (int i = 0; i < 120; ++i) t.update(torque_in(true, 4.0f, 0.002f, 3.0f, p.steer_actuator_delay), p);
    const float curvature_error = 0.002f - t.actual_curvature();
    ASSERT_NEAR(t.error(), curvature_error * 16.0f, 1e-4f) << "오차는 횡가속도뿐이고 저속 곡률 항이 없다";
}

// 라이브 뱅크: 편경사에 해당하는 만큼 FF가 이동해야 한다.
TEST(LateralTorque, LiveBankCompensation) {
    TorqueController with_bank, without_bank;
    SteeringParams params;
    params.enabled = true;
    params.live_bank_compensation = true;
    SteeringParams off = params;
    off.live_bank_compensation = false;
    for (int i = 0; i < 120; ++i) {
        with_bank.update(torque_in(true, 20.0f, 0.002f, 1.0f, params.steer_actuator_delay, -0.117f), params);
        without_bank.update(torque_in(true, 20.0f, 0.002f, 1.0f, off.steer_actuator_delay, -0.117f), off);
    }
    // bank -0.117(우측 기움) -> 중력이 우로 끄니 FF는 좌로 0.117 이동해야 한다
    const float diff = with_bank.feedforward() - without_bank.feedforward();
    ASSERT_LT(std::fabs(diff + 0.117f), 1e-3f) << "실시간 뱅크는 feedforward를 +bank만큼 옮긴다(중력이 반대로 작용)";
}

// latAccelOffset: 상수 편향이 FF에서 그대로 빠져야 한다.
TEST(LateralTorque, LatAccelOffsetShiftsFeedforward) {
    TorqueController a, b;
    SteeringParams params;
    params.enabled = true;
    params.torque_lat_accel_offset = 0.0f;
    SteeringParams offset_params = params;
    offset_params.torque_lat_accel_offset = 0.25f;
    for (int i = 0; i < 120; ++i) {
        a.update(torque_in(true, 20.0f, 0.002f, 1.0f, params.steer_actuator_delay), params);
        b.update(torque_in(true, 20.0f, 0.002f, 1.0f, offset_params.steer_actuator_delay), offset_params);
    }
    const float diff = a.feedforward() - b.feedforward();
    ASSERT_NEAR(diff, 0.25f, 1e-3f) << "lat_accel_offset은 feedforward에서 정확히 빠진다";
}

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

// 켜면 실제 곡률은 opendbc VehicleModel(update_params(x, sr) → calc_curvature(sa, u, roll))을
// 부호 반전한 값이다. double 독립 전사본과 비교한다. 강성은 질량당 값이고 질량은 1이다(opendbc 2026-10-12).
double upstream_measured_curvature(const SteeringParams &p, const LiveLateralParams &live, double angle_deg, double u) {
    const double civic_m = 1462.0, civic_l = 2.70, civic_af = civic_l * 0.4;
    const double civic_ar = civic_l - civic_af;
    const double m = 1.0, l = kWheelbaseM, af = kCenterToFrontM, ar = l - af;
    const double tsf = p.tire_stiffness_factor, x = std::max<double>(live.stiffness_factor, 0.1);
    const double cf = 192150.0 * tsf / civic_m * (ar / l) / (civic_ar / civic_l) * x;
    const double cr = 202500.0 * tsf / civic_m * (af / l) / (civic_af / civic_l) * x;
    const double sf = m * (cf * af - cr * ar) / (l * l * cf * cr);
    const double factor = 1.0 / (1.0 - sf * u * u) / l;  // 상류 (1 - steerRatioRear) = 1
    const double sa = (angle_deg - live.angle_offset_deg) * 3.14159265358979323846 / 180.0;
    const double roll = std::fabs(sf) < 1e-6 ? 0.0 : 9.81 * live.roll_rad / ((1.0 / sf) - u * u);
    return -(factor * sa / std::max<double>(live.steer_ratio, 0.1) + roll);
}

TEST(LateralTorque, LiveVehicleParamsFollowVehicleModel) {
    SteeringParams params;
    params.enabled = true;
    LiveLateralParams live = odd_live_params();
    live.use_torque = false;
    for (float u : {3.0f, 12.0f, 27.0f}) {
        for (float angle : {-30.0f, 0.0f, 4.0f}) {
            TorqueController torque;
            const float got = torque.estimate_actual_curvature(u, angle, VehicleModel::make(params, live));
            const double want = upstream_measured_curvature(params, live, angle, u);
            ASSERT_NEAR(got, want, 2e-6 * std::fabs(want) + 1e-9)
                << "실시간 SR·강성·오프셋·롤이 opendbc calc_curvature를 따른다";
        }
    }
    // 롤은 FF에서 roll·g를 빼고 편경사 추정은 쓰지 않는다(마찰은 포화 구간이라 같다)
    TorqueController with_roll, without_roll;
    LiveLateralParams flat = live;
    flat.roll_rad = 0.0f;
    params.live_bank_compensation = true;
    for (int i = 0; i < 150; ++i) {
        with_roll.update(torque_in(true, 20.0f, 0.004f, 1.0f, params.steer_actuator_delay, -0.5f, live), params);
        without_roll.update(torque_in(true, 20.0f, 0.004f, 1.0f, params.steer_actuator_delay, -0.5f, flat), params);
    }
    ASSERT_NEAR(with_roll.feedforward() - without_roll.feedforward(), -live.roll_rad * 9.81f, 1e-4f)
        << "실시간 롤은 feedforward에서 roll*g를 빼고 뱅크 추정을 대신한다";
}

// 상류는 PID를 횡가속 공간에서 돌리고 끝에서 latAccelFactor로 나눈다. 그러면 마찰이 없을 때
// 출력 × latAccelFactor가 배율과 무관하다(사전값 경로 포함). 마찰은 토크 공간에 그대로,
// 절편은 −offset/latAccelFactor로 더해진다.
TEST(LateralTorque, LiveTorqueParamsMatchUpstreamStructure) {
    SteeringParams params;
    params.enabled = true;
    params.torque_friction = 0.0f;
    params.torque_lat_accel_offset = 0.0f;  // 사전값 경로도 절편 없이 비교한다
    params.live_bank_compensation = false;
    const float prior = params.torque_lat_accel_factor;
    auto run = [&](bool use, float factor, float offset, float friction, std::vector<float> *out) {
        TorqueController torque;
        LiveLateralParams live;
        live.use_torque = use;
        live.lat_accel_factor = factor;
        live.lat_accel_offset = offset;
        live.friction = friction;
        for (int i = 0; i < 300; ++i) {
            const float desired = 0.0015f * std::sin(0.03f * i);
            const float angle = 1.5f * std::sin(0.03f * i - 0.4f);
            torque.update(torque_in(true, 20.0f, desired, angle, params.steer_actuator_delay, 0.0f, live), params);
            out->push_back(torque.normalized_output());
        }
    };
    std::vector<float> base, low, high, offset, friction;
    run(false, 0.0f, 0.0f, 0.0f, &base);
    run(true, 3.0f, 0.0f, 0.0f, &low);
    run(true, 5.5f, 0.0f, 0.0f, &high);
    run(true, 3.0f, 0.1f, 0.0f, &offset);
    run(true, 3.0f, 0.0f, 0.05f, &friction);
    float worst_scale = 0.0f, worst_offset = 0.0f, worst_friction = 0.0f;
    for (size_t i = 0; i < base.size(); ++i) {
        // 출력이 포화하지 않는다
        ASSERT_LT(std::fabs(base[i]), 0.9f);
        ASSERT_LT(std::fabs(low[i]), 0.9f);
        const float ref = base[i] * prior;
        worst_scale = std::max({worst_scale, std::fabs(low[i] * 3.0f - ref), std::fabs(high[i] * 5.5f - ref)});
        worst_offset = std::max(worst_offset, std::fabs((offset[i] - low[i]) * 3.0f + kTorqueOutputSign * 0.1f));
        // 마찰은 |오차| < 0.2에서 선형이라 차이가 토크 공간 0.05 이하, 부호는 오차를 따른다
        worst_friction = std::max(worst_friction, std::fabs(friction[i] - low[i]) - 0.05f);
    }
    ASSERT_LT(worst_scale, 2e-5f) << "출력×latAccelFactor는 배율과 무관하다";
    ASSERT_LT(worst_offset, 2e-5f) << "학습한 오프셋은 -offset/latAccelFactor로 들어간다";
    ASSERT_LT(worst_friction, 1e-6f) << "마찰은 토크 공간의 항이고 계수로 제한된다";
}

// 비활성이면 토크 0을 내고 적분기를 지운다. 상류(#24606)는 남기지만, 녹화 폐루프 재생에서 남긴 적분이 정차 뒤
// 출발의 차선 오차를 키웠다(lateral_torque.cc). 다시 켜면 새로 쌓는다.
TEST(LateralTorque, IntegratorResetsWhenInactive) {
    SteeringParams params;
    params.enabled = true;
    params.angle_offset_deg = 0.0f;
    params.torque_friction = 0.0f;
    TorqueController torque;
    for (int i = 0; i < 300; ++i)
        torque.update(torque_in(true, 20.0f, 0.001f, 0.0f, params.steer_actuator_delay), params);
    ASSERT_GT(torque.integral(), 0.05f) << "오차가 이어지면 적분이 쌓인다";
    ASSERT_EQ(torque.update(torque_in(false, 20.0f, 0.001f, 0.0f, params.steer_actuator_delay), params), 0);
    EXPECT_EQ(torque.normalized_output(), 0.0f);
    EXPECT_EQ(torque.integral(), 0.0f) << "비활성 틱에 적분기를 지운다";
}

// 상류 common/pid.py: 이미 한계에 걸린 방향으로는 적분이 움직이지 않는다(적분이 음수여도 양의 오차로 0 쪽으로
// 풀지 않는다). 적분 자체도 ±latAccelFactor 안이다. 예전 이식(0.8 PIController)은 한계에서도 0 쪽이면 풀었다.
TEST(LateralTorque, IntegratorDoesNotMoveWhileClipping) {
    SteeringParams params;
    params.enabled = true;
    params.angle_offset_deg = 0.0f;
    params.torque_friction = 0.0f;
    params.torque_lat_accel_offset = 0.0f;
    TorqueController torque;
    for (int i = 0; i < 1500; ++i)
        torque.update(torque_in(true, 20.0f, -0.0008f, 0.0f, params.steer_actuator_delay), params);
    const float negative = torque.integral();
    ASSERT_LT(negative, -0.1f);
    ASSERT_GT(negative, -params.torque_lat_accel_factor);
    // 요청을 크게 바꾸면 FF가 바로 출력을 양의 한계에 붙인다. 오차는 조향 지연만큼 뒤에 양수가 된다.
    float held = 0.0f;
    int positive_frames = 0;
    for (int i = 0; i < 300; ++i) {
        torque.update(torque_in(true, 20.0f, 0.03f, 0.0f, params.steer_actuator_delay), params);
        ASSERT_GE(std::fabs(torque.normalized_output()), 1.0f - 1e-6f);
        if (torque.error() <= 0.0f) continue;
        if (positive_frames++ == 0) held = torque.integral();
        ASSERT_EQ(torque.integral(), held) << "출력이 양의 한계에 붙어 있으면 양의 오차로 적분을 풀지 않는다 " << i;
    }
    ASSERT_GT(positive_frames, 100);
    EXPECT_LT(held, 0.0f) << "적분은 음수인 채다";

    TorqueController bounded;
    SteeringParams strong = params;
    strong.torque_kp = 0.0f;  // 30 m/s 위: P 없이 적분만 한계까지 쌓는다
    for (int i = 0; i < 20000; ++i)
        bounded.update(torque_in(true, 35.0f, 0.0004f, 0.0f, strong.steer_actuator_delay), strong);
    // 출력이 한계를 넘기 직전 값에서 적분이 멈춘다: 적분 + FF <= 한계
    const float feedforward = 0.0004f * 35.0f * 35.0f;
    EXPECT_LE(bounded.integral() + feedforward, strong.torque_lat_accel_factor + 1e-5f);
    EXPECT_GT(std::fabs(bounded.normalized_output()), 0.99f);
}

// 마찰 보상 deadzone(상류 steeringAngleDeadzoneDeg): 조향각 deadzone이 만드는 횡가속 안의 오차에는 마찰 보상이 없다.
// 횡가속으로는 v^2에 비례해 넓어지므로 40 km/h부터 줄여 60 km/h 위에서는 꺼진다.
TEST(LateralTorque, FrictionDeadzoneOnlyAtLowSpeed) {
    SteeringParams params;
    params.enabled = true;
    params.angle_offset_deg = 0.0f;
    params.torque_ki = 0.0f;
    params.torque_lat_accel_offset = 0.0f;
    params.live_bank_compensation = false;
    const auto feedforward = [&](float v, float angle_deg, float deadzone_deg) {
        SteeringParams p = params;
        p.steering_angle_deadzone_deg = deadzone_deg;
        TorqueController torque;
        for (int i = 0; i < 120; ++i) torque.update(torque_in(true, v, 0.0f, angle_deg, p.steer_actuator_delay), p);
        return torque.feedforward();
    };
    // 8 m/s에서 조향각 0.5도의 오차는 1도 deadzone 안이다
    EXPECT_NEAR(feedforward(8.0f, 0.5f, 1.0f), 0.0f, 1e-6f) << "deadzone 안의 오차에는 마찰 보상이 없다";
    EXPECT_GT(std::fabs(feedforward(8.0f, 0.5f, 0.0f)), 0.01f) << "deadzone 0이면 상류처럼 마찰 보상이 붙는다";
    // deadzone 밖은 deadzone만큼 줄어든 오차로 보상한다
    EXPECT_LT(std::fabs(feedforward(8.0f, 2.0f, 1.0f)), std::fabs(feedforward(8.0f, 2.0f, 0.0f)));
    EXPECT_GT(std::fabs(feedforward(8.0f, 2.0f, 1.0f)), 0.0f);
    // 60 km/h 위에서는 deadzone이 없다
    EXPECT_EQ(feedforward(20.0f, 0.2f, 1.0f), feedforward(20.0f, 0.2f, 0.0f));
    // 40~60 km/h 사이는 중간이다: 50 km/h에서는 0.5도만 쓴다
    EXPECT_NEAR(feedforward(50.0f / 3.6f, 0.4f, 1.0f), 0.0f, 1e-6f);
    EXPECT_GT(std::fabs(feedforward(50.0f / 3.6f, 0.6f, 1.0f)), 1e-3f);
}

// 저속 직선 이득 배율: 36 km/h 아래 직선에서만 비례 이득에 곱한다. 회전(요청 곡률 0.008 위)과 54 km/h 위는
// 원래 이득이고, 회전에서 빠져나온 직후에도 지연 비교 대상이 곡률을 들고 있는 동안은 원래 이득이다.
TEST(LateralTorque, StraightKpScaleOnlyOnSlowStraights) {
    SteeringParams params;
    params.enabled = true;
    params.angle_offset_deg = 0.0f;
    params.torque_ki = 0.0f;
    params.torque_friction = 0.0f;
    params.torque_lat_accel_offset = 0.0f;
    params.live_bank_compensation = false;
    params.steering_angle_deadzone_deg = 0.0f;
    const auto output = [&](float v, float desired, float angle_deg, float scale) {
        SteeringParams p = params;
        p.torque_straight_kp_scale = scale;
        TorqueController torque;
        for (int i = 0; i < 120; ++i) torque.update(torque_in(true, v, desired, angle_deg, p.steer_actuator_delay), p);
        return torque.normalized_output();
    };
    // 8 m/s 직선(요청 0, 조향각 3도): FF가 없어 출력이 P뿐이고 배율만큼 줄어든다
    const float straight = output(8.0f, 0.0f, 3.0f, 0.7f) / output(8.0f, 0.0f, 3.0f, 1.0f);
    EXPECT_NEAR(straight, 0.7f, 1e-4f);
    // 8 m/s 회전(요청 0.02, 조향각 -45도면 거의 따라가 출력이 포화하지 않는다. 양의 곡률은 음의 조향각이다):
    // 배율과 무관하다
    const float turn = output(8.0f, 0.02f, -45.0f, 1.0f);
    ASSERT_LT(std::fabs(turn), 0.95f);
    EXPECT_EQ(output(8.0f, 0.02f, -45.0f, 0.7f), turn);
    // 16 m/s 직선: 배율과 무관하다
    EXPECT_EQ(output(16.0f, 0.0f, 1.0f, 0.7f), output(16.0f, 0.0f, 1.0f, 1.0f));
    // 12.5 m/s 직선: 10~15 m/s 사이라 배율의 절반만 쓴다
    const float half = output(12.5f, 0.0f, 1.0f, 0.7f) / output(12.5f, 0.0f, 1.0f, 1.0f);
    EXPECT_NEAR(half, 0.85f, 1e-4f);

    // 회전에서 직선으로: 요청이 0이 된 뒤에도 조향 지연 동안은 지연 비교 대상이 곡률을 들고 있어 원래 이득이다
    TorqueController scaled, full;
    SteeringParams p_scaled = params, p_full = params;
    p_full.torque_straight_kp_scale = 1.0f;
    for (int i = 0; i < 120; ++i) {
        scaled.update(torque_in(true, 8.0f, 0.02f, -45.0f, p_scaled.steer_actuator_delay), p_scaled);
        full.update(torque_in(true, 8.0f, 0.02f, -45.0f, p_full.steer_actuator_delay), p_full);
    }
    for (int i = 0; i < 20; ++i) {
        scaled.update(torque_in(true, 8.0f, 0.0f, -45.0f, p_scaled.steer_actuator_delay), p_scaled);
        full.update(torque_in(true, 8.0f, 0.0f, -45.0f, p_full.steer_actuator_delay), p_full);
        ASSERT_LT(std::fabs(full.normalized_output()), 0.95f);
        ASSERT_EQ(scaled.normalized_output(), full.normalized_output()) << "회전 탈출 직후 " << i;
    }
}

// 유한하지 않은 값이 들어와도 한쪽 최대 토크가 아니라 0을 낸다(상류 controlsd: 유한하지 않은 actuator 값은 0).
// clamp_float는 NaN을 하한으로 돌려주므로 막지 않으면 PID가 -한계를 내고 +steer_max가 나갔다(2026-09-24 NaN 속도).
TEST(LateralTorque, NonFiniteInputsGiveZeroTorque) {
    SteeringParams params;
    // 학습 조향비가 NaN이면 실제 곡률이 NaN이 된다: 토크는 0이고 적분기에 NaN이 남지 않는다
    LiveLateralParams nan_ratio;
    nan_ratio.use_vehicle = true;
    nan_ratio.steer_ratio = NAN;
    TorqueController torque;
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(
            torque.update(torque_in(true, 20.0f, 0.001f, 1.0f, params.steer_actuator_delay, 0.0f, nan_ratio), params),
            0)
            << "틱 " << i;
        ASSERT_TRUE(std::isfinite(torque.integral()));
    }
    EXPECT_EQ(torque.nonfinite_ticks(), 100u);
    EXPECT_EQ(torque.normalized_output(), 0.0f);
    // 값이 돌아오면 다음 틱부터 다시 계산한다
    EXPECT_NE(torque.update(torque_in(true, 20.0f, 0.001f, 1.0f, params.steer_actuator_delay), params), 0);
}

}  // namespace
