#pragma once

// openpilot latcontrol_torque(v0.11)의 C++ 이식.

#include <cmath>
#include <cstdint>

#include "controls/control_params.h"

// 보내는 조향 토크의 부호. 이 코드의 횡제어는 오른쪽이 양수이고 MDPS 토크(LKAS11 요청, MDPS12 운전자
// 토크)는 왼쪽이 양수다(현대·기아 공통, openpilot과 같다). 토크 제한과 운전자 토크 비교는 MDPS 쪽 부호로
// 한다.
constexpr int kTorqueOutputSign = -1;

// 학습값. 상류 controlsd가 vehicleParameters·lateralTorqueParameters를 쓰는 자리다.
// 끈 쪽은 SteeringParams를 그대로 쓴다.
struct LiveLateralParams {
    bool use_vehicle = false;  // paramsd: SR·강성·영점 합계·롤
    float steer_ratio = 0.0f;
    float stiffness_factor = 1.0f;
    float angle_offset_deg = 0.0f;
    float roll_rad = 0.0f;    // 양수 = 오른쪽이 낮다
    bool use_torque = false;  // torqued 필터값
    float lat_accel_factor = 0.0f;
    float lat_accel_offset = 0.0f;
    float friction = 0.0f;
};

class TorqueController {
public:
    // update의 측정과 요청(틱마다). 기본값은 학습값·편경사가 없고 운전자가 핸들을 잡지 않은 상태다.
    struct Input {
        // 상류 CC.latActive. 거짓이면 토크 0을 내고 PID 상태를 지운다(상류 #24606은 적분기를 남기지만 폐루프
        // 재생에서 정차 뒤 차선 오차를 키워 지운다, update 참고). 요청 버퍼와 저크 필터는 늘 갱신한다. 다시 참이 된
        // 뒤 지연 틱 수만큼은 오차 없이 피드포워드만 낸다.
        bool active = false;
        float speed_mps = 0.0f;
        float desired_curvature = 0.0f;
        float steering_angle_deg = 0.0f;
        bool steering_pressed = false;
        bool steer_limited_by_safety = false;
        // 조향 지연(상류 LatControlTorque.update의 lat_delay). 요청 버퍼에서 이만큼 전의 요청을 지금 측정과
        // 비교한다. 컨트롤러는 경로를 읽는 지연과 같은 값(lagd 사용 중이면 추정값, 아니면 steer_actuator_delay)을
        // 넘긴다.
        float lat_delay_s = 0.0f;
        float yaw_rate_rad_s = 0.0f;
        bool yaw_rate_valid = false;
        float road_bank_lat_accel = 0.0f;
        LiveLateralParams live{};
    };
    // 학습한 토크 모델(LateralNnModel)의 피드포워드(제어 부호, kHyundaiSteerMax로 나눈 토크)와 섞는 법. weight(0..1)만큼
    // 선형 피드포워드(목표 횡가속, 편향, 롤·편경사, 마찰)와 섞는다: 1이면 모델에 곡선(|목표 횡가속| 0.2→0.6 m/s²)에서만
    // 선형의 마찰 보상을 더한 값(curve_friction이 거짓이면 모델만: 마찰 몫을 빼지 않고 배운 모델). curve_gain(모델 파일
    // curve_gain)은 같은 곡선 범위와 30→50 km/h에서 모델 출력을 (1 + 이것)배로 키운다. NaN이거나 무게가 0이면 선형만.
    // 켜고 끌 때 피드포워드가 건너뛰지 않게 호출자가 무게를 천천히 바꾼다.
    struct NnFeedforward {
        float torque = NAN;
        float weight = 1.0f;
        bool curve_friction = true;
        float curve_gain = 0.0f;
        // 계획의 |목표 횡가속|이 커지는 빠르기(m/s³). 곡선 이득은 이것이 양수일 때(곡선 진입)만 건다.
        float curve_rise = 0.0f;
    };

    // openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다. PID는 횡가속 공간에서 돈다.
    int update(const Input &in, const SteeringParams &params, const NnFeedforward &nn);
    int update(const Input &in, const SteeringParams &params) { return update(in, params, NnFeedforward()); }

    // 조향각이 만드는 곡률(차량 모델: 학습 SR·강성·오프셋·롤 포함, 제어 부호). estimate_actual_curvature의
    // 각도 경로와 같은 식이고 로그용 값을 건드리지 않는다. 컨트롤러의 조향각 상한이 쓴다.
    float curvature_at_angle(float speed_mps, float steering_angle_deg, const SteeringParams &params,
                             const LiveLateralParams &live = LiveLateralParams{});

    // curvature_at_angle의 역. 속도와 차량 모델이 같으면 조향각(도, MDPS 부호) = gain·곡률 + offset이라, 학습한 토크
    // 모델의 목표 조향각 입력은 틱마다 한 번 만들어 여러 시점에 쓴다.
    struct AngleMap {
        float gain = 0.0f;
        float offset = 0.0f;
        float operator()(float curvature) const { return gain * curvature + offset; }
    };
    AngleMap angle_map(float speed_mps, const SteeringParams &params,
                       const LiveLateralParams &live = LiveLateralParams{});

    // 현재 조향각/속도에서 차량 모델 기반 실제 curvature를 추정한다.
    float estimate_actual_curvature(float speed_mps, float steering_angle_deg, const SteeringParams &params,
                                    float yaw_rate_rad_s = 0.0f, bool yaw_rate_valid = false,
                                    const LiveLateralParams &live = LiveLateralParams{});

    float normalized_output() const { return normalized_output_; }
    float error() const { return error_; }
    float feedforward() const { return feedforward_; }
    // 선형 피드포워드의 마찰 보상 몫(정규화 토크, 제어 부호). 모델을 쓸 때 실제로 더한 몫과 상관없이 선형 기준이다.
    // 학습 쪽(nnlc)이 정답 토크에서 이 몫을 빼는 데 쓴다(replay_controls --dump의 fr).
    float friction_torque() const { return friction_torque_; }
    // 마지막 update가 학습한 토크 모델의 피드포워드를 (조금이라도) 섞었다
    bool nn_feedforward_used() const { return nn_feedforward_used_; }
    // PID 적분항(횡가속도 공간, 상류 LateralTorqueState.i)
    float integral() const { return i_; }
    // 유한하지 않은 입력(오차·피드포워드·이득) 때문에 토크 0을 낸 틱 수(처음부터 센다)
    uint64_t nonfinite_ticks() const { return nonfinite_ticks_; }
    float actual_curvature() const { return actual_curvature_; }
    // 조향각 차량 모델 곡률(제어가 쓰는 실제 곡률)과 ESP12 요레이트 곡률(주행 로그용).
    float actual_curvature_vm() const { return actual_curvature_vm_; }
    float actual_curvature_yaw() const { return actual_curvature_yaw_; }

private:
    // 상류 torque_params: torqued를 쓰면 학습값, 아니면 SteeringParams의 사전값.
    struct TorqueTuning {
        float lat_accel_factor = 1.0f;
        float friction = 0.0f;  // 토크 공간(상류도 그렇다)
        float lat_accel_offset = 0.0f;
    };
    static TorqueTuning torque_tuning(const SteeringParams &params, const LiveLateralParams &live);

    // 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
    void update_vehicle_model(const SteeringParams &params);
    // 차량 모델 값(학습을 쓰면 학습 SR·강성을 넣은 사본 live_vehicle_params_)과 조향각 영점. 곡률↔조향각 변환이 같이 쓴다.
    const SteeringParams &vehicle_model_params(const SteeringParams &params, const LiveLateralParams &live,
                                               float *angle_offset_deg);

    // 조향각과 속도에서 실제 curvature를 계산한다.
    float vehicle_model_curvature(float steering_angle_rad, float speed_mps, const SteeringParams &params);

    // opendbc VehicleModel.roll_compensation. vehicle_model_curvature 뒤에 부른다.
    float roll_compensation(float roll_rad, float speed_mps) const;

    // 상류 common/pid.py PIDController.update(k_d 0). 횡가속도 공간에서 돌고 출력과 적분기는
    // ±limit(= latAccelFactor, 상류 update_limits)로 묶인다. kp는 속도별 곡선에 저속 직선 배율을 곱한 값이다.
    float pid_update(float error, float feedforward, bool freeze_integrator, float kp, float ki, float limit);

    // PID 항(횡가속도 공간). 비활성이면 지운다.
    float p_ = 0.0f;
    float i_ = 0.0f;
    float f_ = 0.0f;
    float slip_factor_ = 0.0f;
    float last_tire_stiffness_factor_ = -1.0f;
    float last_steer_ratio_ = -1.0f;
    float normalized_output_ = 0.0f;
    float error_ = 0.0f;
    float feedforward_ = 0.0f;
    float friction_torque_ = 0.0f;
    bool nn_feedforward_used_ = false;
    uint64_t nonfinite_ticks_ = 0;
    float actual_curvature_ = 0.0f;
    float actual_curvature_vm_ = 0.0f;
    float actual_curvature_yaw_ = 0.0f;
    SteeringParams live_vehicle_params_{};  // 학습 SR·강성을 넣은 사본

    // 지연 보정 링버퍼(100Hz 1초): 오차 = delay 전 요청 - 지금 측정.
    static constexpr int kRequestBufferLen = 100;
    float lat_accel_request_[kRequestBufferLen] = {};
    int request_head_ = 0;
    // 조향을 시작한 뒤 지난 활성 틱 수(kRequestBufferLen에서 멈춘다). 지연 틱 수보다 작으면 오차를 쓰지 않는다(update).
    int active_ticks_ = 0;
    // 저크 선행 마찰용 1.2Hz 저역통과 상태
    float jerk_filtered_ = 0.0f;
};
