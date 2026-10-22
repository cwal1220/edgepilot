#include "controls/angle_fault_guard.h"

#include <algorithm>
#include <cmath>

#include "car/hyundai_can.h"

namespace {

// 운전자가 넘겨받은 큰 회전에서 조향을 다시 켜는 각도(carrotpilot lat_suspend_control의 resume_angle)
constexpr float kDriverReleaseAngleDeg = 15.0f;
// 운전자가 넘겨받은 정지라도 핸들이 고장 각도(85도)보다 이만큼 아래이고 계획 곡률이 지금 핸들 곡률의 ±25% 안(같은 쪽,
// LateralController::plan_matches_wheel)인 채로 1초 가면 다시 조향한다. 램프처럼 운전자가 85도를 잠깐 넘긴 뒤 커브를
// 붙잡고 있는 경우다. 교차로를 빠져나오며 펴는 핸들은 계획(직진)과 어긋나 그대로 쉰다: 2026-10-03 2:54 탈출은 계획이
// 직진인데 핸들이 77도인 채 다시 켜져 최대 토크로 핸들을 밀었다. 0.3초면 탈출 중 잠깐 멈춘 핸들에서도 켜졌다(녹화
// 15개 재생: 0.3초 26번 중 25번이 교차로 탈출, 1초는 9번 중 8번이고 기존보다 0.5~2.2초 일찍. 2026-10-05 16:46 램프는
// 두 경우 다 켜져 7~10초 일찍 돕는다).
constexpr float kHeldTurnResumeMarginDeg = 5.0f;
constexpr int kHeldTurnResumeFrames = 100;

}  // namespace

// avoid_lkas_fault를 끈 경우: 상류 hyundai carcontroller의 common_fault_avoidance(MAX_ANGLE, MAX_ANGLE_FRAMES,
// MAX_ANGLE_CONSECUTIVE_FRAMES). 요청을 낸 채 85도 위에 머문 프레임을 세어 89를 넘으면 2프레임 요청을 끄고
// (ToiFlt를 켜고 토크는 그대로) 다시 센다. K7은 끈 요청을 85도 위에서 다시 켜면 곧 고장을 내므로 기본값은
// 켠 쪽(update_hold)이다.
bool AngleFaultGuard::update_cut(const SteeringParams &params, bool steer_request, float steering_angle_deg) {
    if (params.avoid_lkas_fault_enabled) {
        angle_limit_counter_ = 0;
        return false;
    }
    const bool fault_condition = std::fabs(steering_angle_deg) >= params.avoid_lkas_fault_max_angle_deg;
    angle_limit_counter_ = steer_request && fault_condition ? angle_limit_counter_ + 1 : 0;
    const bool cut = angle_limit_counter_ > params.avoid_lkas_fault_max_frames;
    if (angle_limit_counter_ >= params.avoid_lkas_fault_max_frames + params.avoid_lkas_fault_cut_frames)
        angle_limit_counter_ = 0;
    return cut;
}

// 큰 조향각 고장 회피. K7 MDPS는 steer 요청이 켜진 채 |조향각|이 85도 위에 0.98~1.12초 머물면
// 토크와 무관하게 ToiFlt+FailStat을 내고, 85도 아래로 와야 푼다(2026-09-18 실측). 요청을 내는
// 동안(active, 또는 정차 같은 가용성 대기) 85도 위 체류를 세어 avoid_lkas_fault_max_frames에 닿으면
// 요청을 끈다. 그 프레임까지 토크는 torque_cap으로 0까지 내려 둔다. openpilot식 2프레임 컷은 85도 위에서
// 요청을 다시 켜는 순간 3~14 ms 뒤 고장을 냈고(2026-10-03, 4번 중 4번), 정차 대기 중에는 세지도
// 않아 1초 뒤 고장이 났다. 그래서 85도 위에서는 요청을 새로 켜지 않는다(결합이나 차단 해제가
// 85도 위에서 일어나도 같다).
// 끈 요청은 85도 아래로 오면 다시 켠다. 다만 그 체류 중 운전자가 핸들을 돌렸으면(넘겨받은 회전) 핸들이
// kDriverReleaseAngleDeg(15도) 아래로 오고 손을 떼거나(carrotpilot lat_suspend_control과 같은 해제 조건), 80도
// 아래에서 계획이 지금 핸들과 같은 커브를 1초 가리킬 때까지 끈 채로 둔다. 회전을 빠져나오며 핸들을 펴는 운전자를
// 시스템이 미리 밀지 않고(2026-10-03 2:54 탈출에서 최대 213), 램프에서 85도를 잠깐 넘긴 뒤 커브를 붙잡은 운전자에게는
// 커브가 끝날 때까지 손을 놓게 하지 않는다(2026-10-05 16:46 램프는 14초 동안 토크 0).
bool AngleFaultGuard::update_hold(const SteeringParams &params, bool steer_requested, float steering_angle_deg,
                                  bool steering_pressed, bool plan_matches_wheel, bool steer_req_sent) {
    const float angle = std::fabs(steering_angle_deg);
    const bool above = params.avoid_lkas_fault_enabled && angle >= params.avoid_lkas_fault_max_angle_deg;
    if (above && steer_requested && steering_pressed) driver_took_wheel_ = true;
    if (!above) {
        fault_angle_frames_ = 0;
        if (large_angle_hold_ && driver_took_wheel_) {
            const bool on_planned_turn =
                angle < params.avoid_lkas_fault_max_angle_deg - kHeldTurnResumeMarginDeg && plan_matches_wheel;
            held_turn_frames_ = on_planned_turn ? held_turn_frames_ + 1 : 0;
            const bool centered = angle < kDriverReleaseAngleDeg && !steering_pressed;
            if (!centered && held_turn_frames_ < kHeldTurnResumeFrames) return true;
        }
        large_angle_hold_ = false;
        driver_took_wheel_ = false;
        held_turn_frames_ = 0;
        return false;
    }
    held_turn_frames_ = 0;
    if (!steer_requested) {
        fault_angle_frames_ = 0;
        return large_angle_hold_;
    }
    if (!steer_req_sent) {
        large_angle_hold_ = true;
    } else if (!large_angle_hold_ && ++fault_angle_frames_ >= params.avoid_lkas_fault_max_frames) {
        large_angle_hold_ = true;
    }
    return large_angle_hold_;
}

int AngleFaultGuard::torque_cap(const SteeringParams &params) const {
    if (fault_angle_frames_ <= 0) return kHyundaiSteerMax;
    return std::min(kHyundaiSteerMax,
                    kHyundaiSteerDeltaDown * std::max(0, params.avoid_lkas_fault_max_frames - fault_angle_frames_));
}
