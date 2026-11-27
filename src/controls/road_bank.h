#pragma once

// 도로 편경사(뱅크)를 ESP12 횡가속과 요레이트로 추정한다: bank = lat + yaw_rate·v(실측 검증식, 2026-08-27 직선
// -0.117 재현). 직선(|yaw·v| < 0.4)에서만 갱신하고 커브에서는 유지한다: 커브에서는 차체 롤의 중력 누설이 섞인다
// (2026-08-30 drive10: 커브 방향 반상관 ±0.2~0.5 + 탈출 꼬리). 시정수 2초. 30초 넘게 갱신이 없으면 낡은 값을 0으로
// 감쇠하고 다시 초기화를 허용한다. 컨트롤러는 100 Hz(dt 0.01 s)로, 데이터셋 도구는 ControlState 간격으로 부른다.

#include "common/utils_math.h"

#include <cmath>

class RoadBankEstimator {
public:
    static constexpr float kRcS = 2.0f;
    static constexpr float kMinSpeedMps = 8.0f;
    static constexpr float kMaxYawAccel = 0.4f;  // |yaw_rate·v|, m/s²: 이보다 크면 커브로 본다
    static constexpr double kStaleS = 30.0;

    void update(float dt_s, float speed_mps, float lat_accel_mps2, bool lat_accel_valid, float yaw_rate_rad_s,
                bool yaw_rate_valid) {
        const float alpha = dt_s / (kRcS + dt_s);
        if (yaw_rate_valid && lat_accel_valid && speed_mps > kMinSpeedMps &&
            std::fabs(yaw_rate_rad_s * speed_mps) < kMaxYawAccel) {
            const float bank = clamp_float(lat_accel_mps2 + yaw_rate_rad_s * speed_mps, -2.0f, 2.0f);
            if (!init_) {
                value_ = bank;
                init_ = true;
            }
            value_ += alpha * (bank - value_);
            stale_s_ = 0.0;
        } else if ((stale_s_ += dt_s) > kStaleS) {
            value_ += alpha * (0.0f - value_);
            init_ = false;
        }
    }

    float lat_accel() const { return value_; }

private:
    float value_ = 0.0f;
    bool init_ = false;
    double stale_s_ = 0.0;
};
