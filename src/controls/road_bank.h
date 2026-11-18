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

// 주행 중 ESP12 요레이트 바이어스를 locationd 요레이트(IMU, 자이로 바이어스를 뺀 값)와 비교해 배운다. 편경사 추정
// (lat + yaw·v)에서 바이어스 b는 b·v의 가짜 편경사가 되고, 학습한 토크 모델은 이를 롤 입력(bank/g)으로 받는다.
// 2026-11-15 주행 넷(11-08 ~ 11-15)에서 ESP12는 직선 주행 중 locationd보다 오른쪽으로 3.5~6.3 mrad/s 더 돌았고(조향각
// 차량 모델 곡률은 locationd와 같았다: 기울기 0.99, 편향 0.00/km), 30 m/s에서 롤 입력이 +0.018 g(학습 표준편차 0.020)
// 밀렸다. 정차 중 바이어스(YawBiasEstimator, 학습기)는 같은 부호지만 -2.1 ~ -3.7 mrad/s로 작다.
// 직선(|yaw·v| < kMaxYawAccel)에서만 배운다: 커브에서는 locationd가 조향각보다 0.16 s 늦어 차이가 오차를 섞는다.
// 요레이트는 둘 다 ESP12 관례(왼쪽 +)로 받는다. locationd가 없거나 낡으면 마지막 값을 유지한다.
class EspYawBiasEstimator {
public:
    static constexpr float kRcS = 20.0f;
    static constexpr float kMinSpeedMps = 8.0f;
    static constexpr float kMaxYawAccel = 0.4f;
    static constexpr float kMaxBias = 0.02f;  // rad/s(1.1°/s)

    void update(float dt_s, float speed_mps, float esp_yaw_rad_s, bool esp_valid, float imu_yaw_rad_s, bool imu_valid) {
        if (!(esp_valid && imu_valid && speed_mps > kMinSpeedMps && std::isfinite(esp_yaw_rad_s) &&
              std::isfinite(imu_yaw_rad_s) && std::fabs(imu_yaw_rad_s * speed_mps) < kMaxYawAccel))
            return;
        const float alpha = dt_s / (kRcS + dt_s);
        bias_ = clamp_float(bias_ + alpha * (esp_yaw_rad_s - imu_yaw_rad_s - bias_), -kMaxBias, kMaxBias);
    }

    float bias() const { return bias_; }

private:
    float bias_ = 0.0f;
};
