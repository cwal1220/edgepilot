#pragma once

// 큰 조향각 고장 회피(85도 규칙)의 상태. K7 MDPS는 steer 요청이 켜진 채 |조향각|이 85도 위에 0.98~1.12초 머물면
// 토크와 무관하게 ToiFlt+FailStat을 내고, 85도 아래로 와야 푼다(2026-09-18 실측). LateralController가 틱마다
// update_cut과 update_hold를 부르고, 조향할 때 torque_cap으로 토크를 묶는다.

#include "controls/control_params.h"

class AngleFaultGuard {
public:
    // avoid_lkas_fault를 끈 경우: 상류 common_fault_avoidance로 85도 위에서 요청을 잠깐 끊는다. 끊는 틱이면 참.
    bool update_cut(const SteeringParams &params, bool steer_request, float steering_angle_deg);

    // 85도 위 체류를 세고 요청을 끌지 정한다(참 = 끈다). plan_matches_wheel은 운전자가 넘겨받은 정지를 계획 곡률로
    // 풀지 볼 때 쓰고(held_by_driver일 때만 의미가 있다), steer_req_sent는 지난 틱에 요청 비트를 실제로 보냈는지다.
    bool update_hold(const SteeringParams &params, bool steer_requested, float steering_angle_deg,
                     bool steering_pressed, bool plan_matches_wheel, bool steer_req_sent);

    // 이번 85도 위 체류 중 운전자가 핸들을 돌렸다(steering pressed)
    bool driver_took_wheel() const { return driver_took_wheel_; }
    // 운전자가 넘겨받은 회전에서 요청을 끈 채다(다시 조향할 때 계획과 핸들을 대조한다)
    bool held_by_driver() const { return large_angle_hold_ && driver_took_wheel_; }

    // 85도 위에서도 토크를 그대로 내되, 요청을 끄는 프레임(avoid_lkas_fault_max_frames)에 0에 닿도록 크기를
    // steer_delta_down × 남은 프레임으로 묶는다. 하강 레이트 한계로 내려와도 늦지 않는 만큼만 남기는 것이라, 최대
    // 토크로 85도를 넘으면 약 0.35초는 그대로 두고 0.55초에 걸쳐 내린다(2026-10-03까지는 0.69초 선형 램프로 0).
    // 요청은 토크가 0일 때 꺼지므로 어시스트가 빠지는 "탁"이 없다.
    int torque_cap(const SteeringParams &params) const;

    void reset() { *this = AngleFaultGuard(); }

private:
    // steer 요청을 낸 채 85도 위에 머문 연속 프레임(요청을 끄면 멈춘다)
    int fault_angle_frames_ = 0;
    // 85도 위에서 요청을 끈 상태. 85도 아래로 와야 풀린다(운전자가 넘겨받았으면 15도 아래에서 손을 떼거나, 80도
    // 아래에서 계획이 지금 핸들과 같은 커브를 가리켜야).
    bool large_angle_hold_ = false;
    bool driver_took_wheel_ = false;
    // 운전자가 넘겨받은 정지에서 80도 아래이고 계획이 핸들과 맞은 연속 프레임
    int held_turn_frames_ = 0;
    // 상류 CarController.angle_limit_counter(avoid_lkas_fault를 끈 경우만)
    int angle_limit_counter_ = 0;
};
