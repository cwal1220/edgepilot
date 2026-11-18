#pragma once

// controlsd와 대조 도구가 같이 쓴다. locationd 상태(IPC)를 학습기 표본으로 옮기고 쓸지를 돌려준다:
// 스위치가 켜져 있고, 0.5초 안의 값이고, 필터가 유효해야 한다(상류는 livePose가 무효면
// paramsd가 아예 돌지 않는다. 여기서는 ESP12로 계속 배운다). inputsOK는 상류 paramsd처럼
// 보지 않는다(자이로 교차검증 실패로 내려가도 자세·각속도 추정은 멀쩡하다). 학습기 라이브러리가
// IPC 레이아웃에 의존하지 않도록 여기 둔다.

#include "common/ipc_messages.h"
#include "learners/lateral_learners.h"

constexpr double kLocalizerMaxAgeS = 0.5;  // 이보다 낡은 locationd 상태는 학습기에 쓰지 않는다

inline bool localizer_sample_from(const LocalizationState &loc, bool enabled, double age_s, double sample_t_s,
                                  LocalizerSample *out) {
    out->stamp_ns = loc.timestamp_ns;
    out->t_s = sample_t_s;
    out->yaw_rate_rad_s = loc.angular_velocity_calib[2];
    out->yaw_rate_std_rad_s = loc.angular_velocity_calib_std[2];
    out->roll_rad = loc.orientation_calib[0];
    out->roll_std_rad = loc.orientation_std[0];
    out->pose_ok = (loc.flags & kLocalizationPosenetOk) != 0;
    out->roll_ok = (loc.flags & kLocalizationSensorsOk) != 0;
    return enabled && age_s >= 0.0 && age_s < kLocalizerMaxAgeS && (loc.flags & kLocalizationFilterValid) != 0;
}

// 컨트롤러의 ESP12 요레이트 바이어스 학습(EspYawBiasEstimator)에 넘기는 locationd 요레이트. ESP12 관례(왼쪽 +)로
// 바꾼다: angular_velocity_calib[2]는 오른쪽 +다(2026-11-15 주행에서 ESP12 요레이트와 상관 -0.99). 0.5초 안의 값이고
// 필터가 유효하며 자이로 입력이 거부되지 않았을 때만 쓴다.
inline bool imu_yaw_left_from(const LocalizationState &loc, double age_s, float *yaw_left_rad_s) {
    *yaw_left_rad_s = -loc.angular_velocity_calib[2];
    return age_s >= 0.0 && age_s < kLocalizerMaxAgeS && (loc.flags & kLocalizationFilterValid) != 0 &&
           (loc.input_flags & kLocalizationInvalidGyro) == 0;
}
