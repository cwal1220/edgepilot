#pragma once

/* controlsd 안의 paramsd·torqued 연결(LateralLearners). 차량 CAN 입력과 locationd 표본을 학습기
 * 입력으로 만들고, 출력을 컨트롤러가 쓰는 LiveLateralParams로 모은다. 학습기 본체는
 * vehicle_params_learner.h(paramsd)·torque_estimator.h(torqued), locationd 상태 변환은
 * localizer_inputs.h. */

#include <cstdint>
#include <deque>
#include <string>

#include "controls/control_params.h"
#include "controls/lateral_torque.h"
#include "learners/torque_estimator.h"
#include "car/vehicle_can.h"
#include "learners/vehicle_params_learner.h"

/* ESP12 자이로 바이어스. 상류는 locationd가 빼 주지만 원시값에는 남아 있다
 * (2026-09-22 정차 중앙값 −0.06~−0.11°/s). 휠속도는 1~2 km/h 밑에서 0을 내서 핸들을
 * 꺾고 기어가는 동안도 정차로 보이므로, 2초 연속 정차이고 거의 돌지 않을 때만 따라간다. */
class YawBiasEstimator {
public:
  explicit YawBiasEstimator(double initial_bias = 0.0) : bias_(initial_bias) {}
  /* 보정된 요레이트를 돌려준다. */
  double update(double t_s, double speed_mps, bool yaw_valid, double yaw_rate_rad_s);
  double bias() const { return bias_; }

private:
  double bias_ = 0.0;
  bool standing_ = false;
  double standing_since_s_ = 0.0;
};

// ---------------------------------------------------------------- controlsd 연결

/* locationd 한 표본. t_s는 학습기 시계(controlsd now_s)로 옮긴 추정 시각(마지막 IMU 샘플). */
struct LocalizerSample {
  double t_s = 0.0;
  double yaw_rate_rad_s = 0.0;  // 보정 좌표계, 우측 양수
  double yaw_rate_std_rad_s = 0.0;
  double roll_rad = 0.0;        // 오른쪽이 낮으면 양수
  double roll_std_rad = 0.0;
  bool pose_ok = false;         // 상류 posenetOK: 요레이트·롤을 관측하는 조건
  bool roll_ok = false;         // 상류 sensorsOK: 롤을 관측하는 조건
};

/* controlsd 안의 paramsd·torqued. 제어 틱 끝에 이번 틱 값(보낸 토크 포함)으로 갱신하고
 * 컨트롤러는 다음 틱에 live()를 쓴다. 상류 controlsd가 직전 메시지를 쓰는 것과 같다.
 * 사전값은 생성 시 파라미터로 고정하고(상류 CarParams처럼), torqued 지연은 컨트롤러의 조향 지연을
 * 따른다(set_lateral_delay, 상류 lateralDelay). */
class LateralLearners {
public:
  /* vehicle_json·torque_cache는 저장 파일 내용이다(없으면 빈 문자열). */
  LateralLearners(const SteeringParams &params, const std::string &vehicle_json,
                  const std::string &torque_cache, uint64_t seed,
                  const VehicleParamsOptions &options = {});

  void update(const VehicleCanState &vehicle, double now_s, double timeout_s, bool lat_active,
              int apply_torque, bool steering_pressed);

  /* use_vehicle = paramsd가 한 번이라도 냈다(상류 sm.seen). use_torque = 봉투가 유효한
   * torqued 메시지를 받은 적이 있다. 토크 값은 그 뒤 봉투가 유효한 메시지로만 바뀐다
   * (상류 sm.all_checks(['lateralTorqueParameters'])). */
  LiveLateralParams live() const { return live_; }
  bool vehicle_valid() const { return vehicle_.params().valid; }
  const VehicleParams &vehicle_params() const { return vehicle_.params(); }
  const TorqueParams &torque_params() const { return torque_.params(); }
  double yaw_bias_rad_s() const { return bias_.bias(); }
  double prior_steer_ratio() const { return constants_.steer_ratio; }
  bool vehicle_published() const { return vehicle_published_; }
  bool torque_published() const { return torque_published_; }

  // 저장: 이번 틱에 새 내용이 생겼으면 참. 쓰기는 호출자가 제어 루프 밖에서 한다.
  bool vehicle_persist_due() const { return vehicle_persist_due_; }
  std::string vehicle_persist_json() const;
  bool torque_persist_due() const { return torque_persist_due_; }
  const std::string &torque_cache() const { return torque_.cache(); }
  // 상류는 복원이 거부된 paramsd 저장과 깨진 torqued 캐시를 지운다
  bool vehicle_restore_rejected() const { return vehicle_restore_rejected_; }
  TorqueRestore torque_restore_status() const { return torque_.restore_status(); }
  bool vehicle_restored() const { return vehicle_restored_; }

  /* locationd를 paramsd·torqued 입력으로 쓸지와 그 최신 표본(제어 틱마다 부른다). use면
   * 학습기는 kLocalizerDelayS만큼 늦은 타임라인에서 돌고, 각 틱의 조향각·속도·토크와 같은
   * 시각의 요레이트·롤을 받은 표본 사이에서 보간해 관측한다. 표본은 IMU 묶음마다(10 Hz)
   * 평균 70 ms 늦게 오므로, 받은 즉시 지금 시각으로 관측하면 요레이트가 조향보다 늦어 강성을
   * 낮게 배운다(2026-10-01 재생 0.92 → 0.71). 상류는 메시지 시각 순서로 넣는다.
   * 그 시각의 표본이 없거나 자세가 무효(상류 posenetOK)인 틱은 ESP12 입력을 쓴다. */
  static constexpr double kLocalizerDelayS = 0.25;
  void set_localizer(bool use, const LocalizerSample &sample);
  /* 조향 지연(컨트롤러가 경로를 읽는 지연과 같은 값). 상류 torqued처럼 점의 토크·활성 이력을 이만큼
   * 밀어 요레이트와 맞춘다. */
  void set_lateral_delay(double delay_s) { torque_.set_lag(delay_s); }
  bool localizer_inputs() const { return use_localizer_; }

  // 대조 도구용: 마지막으로 학습기에 넣은 입력(locationd 모드면 kLocalizerDelayS 전 틱)
  const VehicleParamsInput &last_vehicle_input() const { return last_vehicle_input_; }
  const TorqueEstimatorInput &last_torque_input() const { return last_torque_input_; }
  TorqueEstimator &torque_estimator() { return torque_; }
  const TorqueEstimator &torque_estimator() const { return torque_; }

private:
  struct Tick {  // 한 제어 틱에서 만든 입력(ESP12 요레이트·롤 기준)
    VehicleParamsInput vin;
    TorqueEstimatorInput tin;
  };
  static VehicleModelConstants constants(const SteeringParams &params);
  bool localizer_at(double t_s, LocalizerSample *out) const;
  void feed(Tick tick);

  VehicleModelConstants constants_;
  VehicleParamsInit init_;
  bool vehicle_restored_ = false;
  bool vehicle_restore_rejected_ = false;
  YawBiasEstimator bias_;
  VehicleParamsLearner vehicle_;
  TorqueEstimator torque_;
  int steer_max_ = 1;
  LiveLateralParams live_{};
  bool vehicle_published_ = false;
  bool torque_published_ = false;
  bool vehicle_persist_due_ = false;
  bool torque_persist_due_ = false;
  VehicleParamsInput last_vehicle_input_{};
  TorqueEstimatorInput last_torque_input_{};
  bool use_localizer_ = false;
  std::deque<Tick> pending_;               // locationd 모드에서 아직 넣지 않은 틱
  std::deque<LocalizerSample> samples_;    // 받은 locationd 표본(시각 순)
};
