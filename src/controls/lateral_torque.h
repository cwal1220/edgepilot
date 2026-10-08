#ifndef LATERAL_TORQUE_H
#define LATERAL_TORQUE_H

// openpilot latcontrol_torque(v0.11)의 C++ 이식.

#include "controls/control_params.h"

/* 보내는 조향 토크의 부호. 이 코드의 횡제어는 오른쪽이 양수이고 MDPS 토크(LKAS11 요청, MDPS12 운전자
 * 토크)는 왼쪽이 양수다(현대·기아 공통, openpilot과 같다). 토크 제한과 운전자 토크 비교는 MDPS 쪽 부호로
 * 한다. */
constexpr int kTorqueOutputSign = -1;

/* 학습값. 상류 controlsd가 vehicleParameters·lateralTorqueParameters를 쓰는 자리다.
 * 끈 쪽은 SteeringParams를 그대로 쓴다. */
struct LiveLateralParams {
  bool use_vehicle = false;  // paramsd: SR·강성·영점 합계·롤
  float steer_ratio = 0.0f;
  float stiffness_factor = 1.0f;
  float angle_offset_deg = 0.0f;
  float roll_rad = 0.0f;  // 양수 = 오른쪽이 낮다
  bool use_torque = false;  // torqued 필터값
  float lat_accel_factor = 0.0f;
  float lat_accel_offset = 0.0f;
  float friction = 0.0f;
};

class TorqueController {
public:
  // openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다.
  /* active: 상류 CC.latActive. 거짓이면 토크 0을 내고 PID 상태를 지운다(상류 #24606은 적분기를 남기지만
   * 폐루프 재생에서 정차 뒤 차선 오차를 키워 지운다, update 참고). 요청 버퍼와 저크 필터는 늘 갱신한다.
   * lat_delay_s: 조향 지연(상류 LatControlTorque.update의 lat_delay). 요청 버퍼에서 이만큼 전의
   * 요청을 지금 측정과 비교한다. 컨트롤러는 경로를 읽는 지연과 같은 값(lagd 사용 중이면 추정값,
   * 아니면 steer_actuator_delay)을 넘긴다. */
  int update(bool active,
             float speed_mps,
             float desired_curvature,
             float steering_angle_deg,
             bool steering_pressed,
             bool steer_limited_by_safety,
             const SteeringParams &params,
             float lat_delay_s,
             float yaw_rate_rad_s = 0.0f,
             bool yaw_rate_valid = false,
             float road_bank_lat_accel = 0.0f,
             const LiveLateralParams &live = LiveLateralParams{});

  /* 조향각이 만드는 곡률(차량 모델: 학습 SR·강성·오프셋·롤 포함, 제어 부호). estimate_actual_curvature의
   * 각도 경로와 같은 식이고 로그용 값을 건드리지 않는다. 컨트롤러의 조향각 상한이 쓴다. */
  float curvature_at_angle(float speed_mps,
                           float steering_angle_deg,
                           const SteeringParams &params,
                           const LiveLateralParams &live = LiveLateralParams{});

  // 현재 조향각/속도에서 차량 모델 기반 실제 curvature를 추정한다.
  float estimate_actual_curvature(float speed_mps,
                                  float steering_angle_deg,
                                  const SteeringParams &params,
                                  float yaw_rate_rad_s = 0.0f,
                                  bool yaw_rate_valid = false,
                                  const LiveLateralParams &live = LiveLateralParams{});

  float normalized_output() const { return normalized_output_; }
  float error() const { return error_; }
  float feedforward() const { return feedforward_; }
  // PID 적분항(횡가속도 공간, 상류 LateralTorqueState.i)
  float integral() const { return i_; }
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

  // 조향각과 속도에서 실제 curvature를 계산한다.
  float vehicle_model_curvature(float steering_angle_rad,
                                float speed_mps,
                                const SteeringParams &params);

  // opendbc VehicleModel.roll_compensation. vehicle_model_curvature 뒤에 부른다.
  float roll_compensation(float roll_rad, float speed_mps) const;

  /* 상류 common/pid.py PIDController.update(k_d 0). 횡가속도 공간에서 돌고 출력과 적분기는
   * ±limit(= latAccelFactor, 상류 update_limits)로 묶인다. 비례 이득은 속도별 곡선을 따른다. */
  float pid_update(float error,
                   float feedforward,
                   bool freeze_integrator,
                   const SteeringParams &params,
                   float limit,
                   float speed_mps);

  // PID 항(횡가속도 공간). 비활성이면 지운다.
  float p_ = 0.0f;
  float i_ = 0.0f;
  float f_ = 0.0f;
  float slip_factor_ = 0.0f;
  float last_mass_kg_ = -1.0f;
  float last_wheelbase_m_ = -1.0f;
  float last_center_to_front_m_ = -1.0f;
  float last_tire_stiffness_factor_ = -1.0f;
  float last_steer_ratio_ = -1.0f;
  float normalized_output_ = 0.0f;
  float error_ = 0.0f;
  float feedforward_ = 0.0f;
  float actual_curvature_ = 0.0f;
  float actual_curvature_vm_ = 0.0f;
  float actual_curvature_yaw_ = 0.0f;
  SteeringParams live_vehicle_params_{};  // 학습 SR·강성을 넣은 사본

  // 지연 보정 링버퍼(100Hz 1초): 오차 = delay 전 요청 - 지금 측정.
  static constexpr int kRequestBufferLen = 100;
  float lat_accel_request_[kRequestBufferLen] = {};
  int request_head_ = 0;
  // 저크 선행 마찰용 1.2Hz 저역통과 상태
  float jerk_filtered_ = 0.0f;
};

#endif  // LATERAL_TORQUE_H
