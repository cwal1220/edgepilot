#ifndef LATERAL_TARGET_H
#define LATERAL_TARGET_H

/* LateralPlanner(횡방향 MPC)가 만들고 LateralController가 소비하는
 * 횡방향 계획. 생산자는 controlsd의 MPC 하나뿐이다. */

#include <cstdint>

constexpr int kLateralControlN = 17;

struct LateralTarget {
    bool valid = false;
    /* 이 계획의 근거가 된 카메라 프레임 캡처 시각(monotonic_now_ns 기준).
     * 컨트롤러가 lag 보상과 staleness gate에 쓴다. 0이면 미상(테스트 등)으로
     * 취급해 나이 보정을 건너뛴다. */
    uint64_t capture_timestamp_ns = 0;
    bool mpc_solution_valid = false;
    bool laneless_mode = false;
    float target_y_m = 0.0f;
    float heading_rad = 0.0f;
    float curvature = 0.0f;
    int desire = 0;
    // 차선 변경 상태(0 없음, 1 운전자 토크 대기, 2 변경 중, 3 마무리)와 방향(-1 왼쪽, 1 오른쪽). HUD 표시용.
    int lane_change_state = 0;
    int lane_change_direction = 0;
    float psis[kLateralControlN] = {};
    float curvatures[kLateralControlN] = {};
    // 차선 관측값(로그 전용). 모델 좌표계(+y=오른쪽), 오프셋 미적용.
    float lane_left_y_m = 0.0f;
    float lane_right_y_m = 0.0f;
    float lane_width_m = 0.0f;
    float lane_left_prob = 0.0f;
    float lane_right_prob = 0.0f;
    float lane_left_std = 0.0f;
    float lane_right_std = 0.0f;
    float lane_d_prob = 0.0f;
};

#endif
