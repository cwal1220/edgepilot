/* 토크 횡제어기(TorqueController, openpilot LatControlTorque 이식): 넘겨받은 조향 지연만큼의 요청
 * 버퍼, 속도별 비례 이득(KP_INTERP), 라이브 뱅크·latAccelOffset feedforward, 그리고 학습값 소비.
 * 실제 곡률은 opendbc calc_curvature의 독립 전사본과, 출력 구조는 상류의 횡가속 공간 PID와 대조한다. */
#include "controls/control_params.h"
#include "controls/lateral_torque.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

/* v0.11식 지연 보정: 요청 스텝 직후 delay 동안은 P가 과거 요청(0)과 현재
 * 측정(0)을 비교해 오차가 없어야 하고, 토크는 FF만으로 나와야 한다. */
TEST(LateralTorque, DelayCompensatedError) {
  TorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
  params.angle_offset_deg = 0.0f;
  const float v = 20.0f;

  // 요청 0으로 버퍼를 채운다
  for (int i = 0; i < 120; ++i)
    torque.update(true, v, 0.0f, 0.0f, false, false, params, params.steer_actuator_delay);
  ASSERT_LT(std::fabs(torque.error()), 1e-6f) << "0 요청이 이어지면 오차가 없다";

  // 곡률 스텝. 조향각은 아직 0(차가 반응 전).
  torque.update(true, v, 0.01f, 0.0f, false, false, params, params.steer_actuator_delay);
  ASSERT_LT(std::fabs(torque.error()), 1e-4f)
      << "스텝 직후 오차는 거의 0이다(지연 보상)";
  ASSERT_GT(torque.feedforward(), 1.0f) << "스텝은 feedforward가 바로 싣는다";

  // delay(31프레임)를 넘겨도 차가 반응하지 않으면 그때 오차가 나타난다
  for (int i = 0; i < 40; ++i)
    torque.update(true, v, 0.01f, 0.0f, false, false, params, params.steer_actuator_delay);
  ASSERT_GT(torque.error(), 1.0f) << "지연이 지나도 못 따라간 만큼은 오차로 나타난다";
}

// inactive 동안에도 요청 버퍼가 갱신되어야 재engage 때 낡은 요청과 비교되지 않는다.
TEST(LateralTorque, ReengageHasNoStaleBufferSpike) {
  TorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
  params.angle_offset_deg = 0.0f;
  const float v = 20.0f;

  // 커브 요청으로 버퍼를 채운 뒤 disengage
  for (int i = 0; i < 120; ++i)
    torque.update(true, v, 0.01f, 0.0f, false, false, params, params.steer_actuator_delay);
  // inactive 동안 요청은 0으로 돌아간다 (직선 수동 주행)
  for (int i = 0; i < 120; ++i)
    torque.update(false, v, 0.0f, 0.0f, false, false, params, params.steer_actuator_delay);
  // 직선에서 re-engage: 버퍼가 신선하면 오차 ~0, 얼었다면 큰 스파이크
  torque.update(true, v, 0.0f, 0.0f, false, false, params, params.steer_actuator_delay);
  ASSERT_LT(std::fabs(torque.error()), 1e-4f)
      << "다시 engage할 때 disengage 전의 낡은 요청과 비교하지 않는다";
}

/* 토크 컨트롤러는 넘겨받은 조향 지연(상류 lat_delay)만큼 전의 요청을 지금 측정과 비교한다:
 * 계단 요청이 오차로 나타나는 틱이 지연을 따른다(예전에는 늘 steer_actuator_delay). */
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
      torque.update(true, 20.0f, desired, 0.0f, false, false, params, delay);
      if (first < 0 && std::fabs(torque.error()) > 1e-6f) first = i - 20;
    }
    EXPECT_EQ(first, static_cast<int>(delay / 0.01f)) << delay;
  }
}

/* 속도별 비례 이득(openpilot KP_INTERP). 오차는 횡가속도만으로 내고 저속 보강은
 * 이 곡선이 맡는다. 곡률 오차 항(LOW_SPEED_Y)은 더 이상 없다. */
TEST(LateralTorque, KpSpeedSchedule) {
  SteeringParams base;
  base.enabled = true;
  base.angle_offset_deg = 0.0f;
  base.torque_friction = 0.0f;   // P항만 남긴다
  base.torque_lat_accel_offset = 0.0f;  // 오차가 순수 횡가속도인지 본다
  base.torque_ki = 0.0f;
  /* 조향각 3도를 실제 곡률로 두고 요청 곡률 0을 준다. 오차 = -actual_lat_accel이라
   * 출력은 kp(v) x kf x v^2 x |actual_curvature|에 비례한다. */
  const auto run = [&](float v, float kp) {
    SteeringParams p = base;
    p.torque_kp = kp;
    TorqueController torque;
    for (int i = 0; i < 120; ++i) torque.update(true, v, 0.0f, 3.0f, false, false, p, p.steer_actuator_delay);
    return torque.normalized_output();
  };
  // 이득 곡선의 노드에서 출력비가 KP_INTERP 비율 x v^2 비율과 맞아야 한다.
  const auto gain_at = [&](float v) {
    const float out = run(v, 0.8f);
    TorqueController probe;
    SteeringParams p = base;
    for (int i = 0; i < 120; ++i) probe.update(true, v, 0.0f, 3.0f, false, false, p, p.steer_actuator_delay);
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
  /* LOW_SPEED_Y가 남아 있으면 곡률 항이 저속에서 오차를 수십 배로 키운다.
   * 오차가 순수 횡가속도인지 확인한다. */
  TorqueController t;
  SteeringParams p = base;
  for (int i = 0; i < 120; ++i) t.update(true, 4.0f, 0.002f, 3.0f, false, false, p, p.steer_actuator_delay);
  const float curvature_error = 0.002f - t.actual_curvature();
  ASSERT_NEAR(t.error(), curvature_error * 16.0f, 1e-4f)
      << "오차는 횡가속도뿐이고 저속 곡률 항이 없다";
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
    with_bank.update(true, 20.0f, 0.002f, 1.0f, false, false, params, params.steer_actuator_delay,
                     0.0f, false, -0.117f);
    without_bank.update(true, 20.0f, 0.002f, 1.0f, false, false, off, off.steer_actuator_delay,
                        0.0f, false, -0.117f);
  }
  // bank -0.117(우측 기움) -> 중력이 우로 끄니 FF는 좌로 0.117 이동해야 한다
  const float diff = with_bank.feedforward() - without_bank.feedforward();
  ASSERT_LT(std::fabs(diff + 0.117f), 1e-3f)
      << "실시간 뱅크는 feedforward를 +bank만큼 옮긴다(중력이 반대로 작용)";
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
    a.update(true, 20.0f, 0.002f, 1.0f, false, false, params, params.steer_actuator_delay);
    b.update(true, 20.0f, 0.002f, 1.0f, false, false, offset_params, offset_params.steer_actuator_delay);
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

/* 켜면 실제 곡률은 opendbc VehicleModel(update_params(x, sr) → calc_curvature(sa, u, roll))을
 * 부호 반전한 값이다. double 독립 전사본과 비교한다. */
double upstream_measured_curvature(const SteeringParams &p, const LiveLateralParams &live,
                                   double angle_deg, double u) {
  const double civic_m = 1326.0 + 136.0, civic_l = 2.70, civic_af = civic_l * 0.4;
  const double civic_ar = civic_l - civic_af;
  const double m = p.mass_kg, l = p.wheelbase_m, af = p.center_to_front_m(), ar = l - af;
  const double tsf = p.tire_stiffness_factor, x = std::max<double>(live.stiffness_factor, 0.1);
  const double cf = 192150.0 * tsf * m / civic_m * (ar / l) / (civic_ar / civic_l) * x;
  const double cr = 202500.0 * tsf * m / civic_m * (af / l) / (civic_af / civic_l) * x;
  const double sf = m * (cf * af - cr * ar) / (l * l * cf * cr);
  const double factor = (1.0 - p.steer_ratio_rear) / (1.0 - sf * u * u) / l;
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
      const float got = torque.estimate_actual_curvature(u, angle, params, 0.0f, false, live);
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
    with_roll.update(true, 20.0f, 0.004f, 1.0f, false, false, params, params.steer_actuator_delay, 0.0f, false, -0.5f, live);
    without_roll.update(true, 20.0f, 0.004f, 1.0f, false, false, params, params.steer_actuator_delay, 0.0f, false, -0.5f, flat);
  }
  ASSERT_NEAR(with_roll.feedforward() - without_roll.feedforward(), -live.roll_rad * 9.81f, 1e-4f)
      << "실시간 롤은 feedforward에서 roll*g를 빼고 뱅크 추정을 대신한다";
}

/* 상류는 PID를 횡가속 공간에서 돌리고 끝에서 latAccelFactor로 나눈다. 그러면 마찰이 없을 때
 * 출력 × latAccelFactor가 배율과 무관하다(사전값 경로 포함). 마찰은 토크 공간에 그대로,
 * 절편은 −offset/latAccelFactor로 더해진다. */
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
      torque.update(true, 20.0f, desired, angle, false, false, params, params.steer_actuator_delay, 0.0f, false, 0.0f, live);
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

}  // namespace
