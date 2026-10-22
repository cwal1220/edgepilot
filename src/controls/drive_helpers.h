#pragma once

// 계획을 조향 요청 곡률로 바꾸는 상류 openpilot drive_helpers의 함수와 한계: plan을 지연만큼 앞에서 읽고
// (get_curvature_from_plan) 횡저크·횡가속·최대 곡률로 자른다(clip_curvature). 상태가 없어 컨트롤러, 학습한 토크
// 모델의 입력, replay_planner, 검사가 같은 구현을 쓴다.

#include "common/model_output.h"
#include "common/utils_math.h"
#include "controls/lateral_target.h"

#include <algorithm>
#include <cmath>

// lag 보상 곡률의 고정 한계. 런타임 튜닝 항목이 아니다. 진단용 참조 구현이
// 같은 값을 쓰도록 헤더에 둔다(값이 갈리면 재생 검증이 조용히 썩는다).
// 직전 출력 대비 틱당 변화율 창. openpilot clip_curvature는 DT_CTRL을 쓴다.
// v0.9.4는 플랜 노드 기준 DT_MDL(0.05) 편차 창이라 틱간 제한이 없었다.
constexpr float kCurvatureRateWindowS = 0.01f;
// openpilot MAX_CURVATURE. 12 km/h 아래에서만 횡가속 한계보다 먼저 물고,
// 실주행 1~12 km/h 곡률은 p99 0.017 / 최대 0.064라 닿지 않는다.
constexpr float kMaxCurvature = 0.2f;
// openpilot drive_helpers.MIN_SPEED
constexpr float kMinCurvatureSpeedMps = 1.0f;
// EU 안전 한계(openpilot drive_helpers MAX_LATERAL_JERK, MAX_LATERAL_ACCEL_NO_ROLL).
constexpr float kMaxLateralJerk = 5.0f;
constexpr float kMaxLateralAccel = 3.0f;
// openpilot drive_helpers.MIN_STABLE_DELAY: 이보다 짧은 지연으로 plan을 읽으면 2·ψ/(v·t) 식이 잡음을
// 키우므로 이 지점의 yaw를 지연 비율만큼 줄여 쓴다(get_curvature_from_plan).
constexpr float kMinStableDelayS = 0.3f;
// lag 보상에 더하는 plan 나이의 상한. 이 이상 낡은 plan은 staleness gate가
// 별도로 차단한다.
constexpr float kMaxPlanAgeCompS = 0.25f;

// 곡률 상한: 횡가속 ±kMaxLateralAccel(학습 롤만큼 이동, 속도 하한 kMinCurvatureSpeedMps)과 ±kMaxCurvature.
// clip_curvature와 학습한 토크 모델의 입력 궤적이 같이 쓴다.
inline float limit_curvature(float speed_mps, float curvature, float roll_rad) {
    const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
    const float roll_lat_accel = roll_rad * 9.81f;
    curvature = clamp_float(curvature, (-kMaxLateralAccel + roll_lat_accel) / (speed * speed),
                            (kMaxLateralAccel + roll_lat_accel) / (speed * speed));
    return clamp_float(curvature, -kMaxCurvature, kMaxCurvature);
}

// LateralTarget 격자(model_t_idx(0..kLateralControlN-1)) 위의 값을 시각 x(s)에서 선형 보간한다. 0 이하는 첫 값,
// 끝 너머는 마지막 값이다.
inline float interp_lateral(float x, const float *values) {
    if (x <= 0.0f) return values[0];
    for (int i = 1; i < kLateralControlN; ++i) {
        const float high_x = model_t_idx(i);
        if (x <= high_x) {
            const float low_x = model_t_idx(i - 1);
            const float p = (x - low_x) / (high_x - low_x);
            return values[i - 1] + p * (values[i] - values[i - 1]);
        }
    }
    return values[kLateralControlN - 1];
}

// lateral MPC plan을 actuator delay + plan 나이만큼 앞에서 읽은 곡률. 상류에선 modeld의
// action.desiredCurvature 자리다.
inline float lag_adjusted_curvature(const LateralTarget &target, float speed_mps, float plan_age_s,
                                    float steer_actuator_delay_s) {
    // plan은 카메라 캡처 시점 기준이므로 소비 시점까지의 실측 나이를 actuator
    // delay에 더해 보간한다. 부수 효과로 desired curvature가 20Hz 계단 대신
    // 매 tick plan 위를 따라 전진한다.
    const float delay = std::max(0.01f, steer_actuator_delay_s) + clamp_float(plan_age_s, 0.0f, kMaxPlanAgeCompS);
    const float current_curvature = target.curvatures[0];
    const float psi = delay < kMinStableDelayS
                          ? delay / kMinStableDelayS * interp_lateral(kMinStableDelayS, target.psis)
                          : interp_lateral(delay, target.psis);
    // openpilot drive_helpers.MIN_SPEED. 하한이 낮으면 psi/(v*delay)가 정지
    // 부근에서 발산해 작은 plan 오차가 곡률 상한까지 증폭된다.
    const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
    const float curvature_from_psi = psi / (speed * delay);
    return current_curvature + 2.0f * (curvature_from_psi - current_curvature);
}

// openpilot drive_helpers.clip_curvature: 직전 출력 기준 횡저크, 횡가속(롤만큼 이동), 최대 곡률.
// limited가 있으면 횡가속·최대 곡률 한계에 걸렸는지 적는다(저크 제한은 치지 않는다, 상류와 같음).
inline float clip_curvature(float speed_mps, float prev_curvature, float new_curvature, float roll_rad = 0.0f,
                            bool *limited = nullptr) {
    if (limited) *limited = false;
    // 바퀴 속도가 끊기면 NaN이다. 그대로 두면 clamp_float가 -kMaxCurvature를 낸다.
    if (!std::isfinite(speed_mps)) return prev_curvature;
    const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
    // ISO 횡저크 한계를 직전 출력 기준 틱당 변화율로 건다. 플랜 노드 기준 창이던
    // v0.9.4와 달리 틱간 계단을 실제로 막아, 변화율 제한된 와이어가 예산을
    // 노이즈에 쓰지 않는다.
    const float max_curvature_rate = kMaxLateralJerk / (speed * speed);
    const float rate_limited = clamp_float(new_curvature, prev_curvature - max_curvature_rate * kCurvatureRateWindowS,
                                           prev_curvature + max_curvature_rate * kCurvatureRateWindowS);
    const float curvature = limit_curvature(speed_mps, rate_limited, roll_rad);
    if (limited) *limited = curvature != rate_limited;
    return curvature;
}
