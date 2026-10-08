#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "controls/control_block.h"
#include "controls/control_params.h"
#include "car/hyundai_can.h"
#include "controls/lateral_path.h"
#include "controls/lateral_target.h"
#include "controls/lateral_torque.h"
#include "car/vehicle_can.h"

/* lag 보상 곡률의 고정 한계. 런타임 튜닝 항목이 아니다. 진단용 참조 구현이
 * 같은 값을 쓰도록 헤더에 둔다(값이 갈리면 리플레이 검증이 조용히 썩는다). */
/* 직전 출력 대비 틱당 변화율 창. openpilot clip_curvature는 DT_CTRL을 쓴다.
 * v0.9.4는 플랜 노드 기준 DT_MDL(0.05) 편차 창이라 틱간 제한이 없었다. */
constexpr float kCurvatureRateWindowS = 0.01f;
// openpilot MAX_CURVATURE. 12 km/h 아래에서만 횡가속 한계보다 먼저 물고,
// 실주행 1~12 km/h 곡률은 p99 0.017 / 최대 0.064라 닿지 않는다.
constexpr float kMaxCurvature = 0.2f;
// openpilot drive_helpers.MIN_SPEED
constexpr float kMinCurvatureSpeedMps = 1.0f;
// EU 안전 한계(openpilot drive_helpers MAX_LATERAL_JERK, MAX_LATERAL_ACCEL_NO_ROLL).
constexpr float kMaxLateralJerk = 5.0f;
constexpr float kMaxLateralAccel = 3.0f;
/* openpilot drive_helpers.MIN_STABLE_DELAY: 이보다 짧은 지연으로 plan을 읽으면 2·ψ/(v·t) 식이 잡음을
 * 키우므로 이 지점의 yaw를 지연 비율만큼 줄여 쓴다(get_curvature_from_plan). */
constexpr float kMinStableDelayS = 0.3f;
/* lag 보상에 더하는 plan 나이의 상한. 이 이상 낡은 plan은 staleness gate가
 * 별도로 차단한다. */
constexpr float kMaxPlanAgeCompS = 0.25f;
// openpilot selfdrived SOFT_DISABLE_TIME: 해제 전 경고를 띄운 채 조향을 유지하는 시간.
constexpr double kSoftDisableS = 3.0;
/* openpilot LatControl 포화 판정. steerLimitTimer는 opendbc hyundai 값, 속도 하한은
 * sat_check_min_speed. */
constexpr float kSteerLimitTimerS = 0.4f;
constexpr float kSatCheckMinSpeedMps = 10.0f;
/* openpilot controlsd steer_limited_by_safety: 보낸 토크가 컨트롤러 출력과 정규화 토크로 이만큼 넘게
 * 다르면(토크 레이트·운전자 토크 한계) 다음 틱의 적분과 포화 판정을 멈춘다. */
constexpr float kSteerLimitedBySafetyThreshold = 1e-2f;
// openpilot controlsd: 이 속도 이하(최소 조향 속도가 더 크면 그 속도)는 정차로 보고 조향하지 않는다.
constexpr float kStandstillSteerSpeedMps = 0.3f;
/* openpilot car_events: MDPS 일시 고장 때 운전자가 핸들을 놓은 지 이 시간이 안 됐으면(또는 정차면)
 * 해제 예고 대신 조용한 경고만 낸다. */
constexpr int kSilentSteerWarningFrames = 150;

/* lateral MPC plan을 actuator delay + plan 나이만큼 앞에서 읽은 곡률. 상류에선 modeld의
 * action.desiredCurvature 자리다. */
float lag_adjusted_curvature(const LateralTarget &target, float speed_mps, float plan_age_s,
                             float steer_actuator_delay_s);

/* openpilot drive_helpers.clip_curvature: 직전 출력 기준 횡저크, 횡가속(롤만큼 이동), 최대 곡률.
 * limited가 있으면 횡가속·최대 곡률 한계에 걸렸는지 적는다(저크 제한은 치지 않는다, 상류와 같음). */
float clip_curvature(float speed_mps, float prev_curvature, float new_curvature,
                     float roll_rad = 0.0f, bool *limited = nullptr);

// 둘을 잇는다. plan이 무효면 0. 컨트롤러와 replay_planner가 같은 구현을 쓴다.
float lag_adjusted_desired_curvature(const LateralTarget &target, float speed_mps,
                                     float plan_age_s, float steer_actuator_delay_s,
                                     float prev_curvature, float roll_rad = 0.0f);

struct LateralControllerConfig {
  bool force_engaged = false;
  SteeringParams steering_params{};
  DrivingParams driving_params{};
};

struct LateralControlResult {
  bool engaged = false;
  bool active = false;
  /* 지금 조향 중이다(상류 CC.latActive): active이고 정차·MDPS 일시 고장·큰 조향각 요청 끔이 아니다. 거짓이면
   * 토크는 0이고 목표 곡률은 실제 곡률을 따라간다. */
  bool lat_active = false;
  bool engage_rejected = false;
  bool should_send = false;
  bool path_usable = false;
  bool seeds_ready = false;
  bool vehicle_fresh = false;
  bool left_lane = false;
  bool right_lane = false;
  float cluster_speed_kph = 0.0f;
  float control_speed_kph = 0.0f;
  float desired_curvature = 0.0f;
  float actual_curvature = 0.0f;
  // 조향각 차량 모델 곡률과 ESP12 요레이트 곡률(주행 로그의 curveVm·curveYaw).
  float actual_curvature_vm = 0.0f;
  float actual_curvature_yaw = 0.0f;
  float curvature_error = 0.0f;
  float normalized_output = 0.0f;
  float feedforward = 0.0f;
  int desired_torque = 0;
  int apply_torque = 0;
  bool steering_pressed = false;
  /* MDPS 일시 고장(상류 steerFaultTemporary: MDPS12 ToiUnavail 또는 ToiFlt). 그동안 조향하지 않고 요청
   * 비트도 내린다(상류 latActive=false). */
  bool steer_fault = false;
  /* avoid_lkas_fault를 끈 경우의 큰 조향각 컷(상류 common_fault_avoidance): 85도 위에서 요청을 89프레임
   * 낸 뒤 2프레임 요청을 끄고 ToiFlt를 켠다. 토크는 그대로 보낸다. */
  bool cut_steer_temp = false;
  /* 큰 조향각 고장 회피: 85도 위에서 요청을 끈 상태. 토크가 0일 때 들어가고 ToiFlt 없이 요청
   * 비트만 내린다. 85도 아래로 오면 풀리고, 운전자가 넘겨받은 회전이면 15도 아래에서 손을 떼야 풀린다.
   * 그동안 조향은 비활성처럼 쉰다(토크 0, 목표 곡률은 실제를 따라감). */
  bool large_angle_hold = false;
  // 그중 운전자가 넘겨받은 회전이라 15도 아래에서 손을 떼야 풀리는 경우
  bool large_angle_hold_by_driver = false;
  /* 해제 예고: 캘리브레이션 같은 SoftDisable 사유로 3초 뒤 해제된다. 조향은 계속한다(MDPS 일시 고장이
   * 사유면 고장 동안은 쉰다). */
  bool soft_disabling = false;
  // openpilot steerSaturated: 커브가 조향 한계를 넘어 목표 곡률을 못 따라간다.
  bool steer_saturated = false;
  BlockReason active_block = BlockReason::None;
  std::vector<CanFrame> frames;
};

class LateralController {
public:
  float road_bank_lat_accel() const { return road_bank_lat_accel_; }
  explicit LateralController(LateralControllerConfig config = LateralControllerConfig{});

  // 제어 상태를 유지한 채 런타임 파라미터를 즉시 교체한다.
  void update_params(const SteeringParams &steering_params,
                     const DrivingParams &driving_params);

  /* 학습기의 최신 출력. use_*는 값이 있다는 뜻이고 실제 사용은 스위치가 정한다.
   * 차량 값을 쓰는 중 vehicle_valid가 거짓이고 캘리브가 끝났으면 차단한다
   * (상류 paramsdTemporaryError). */
  void set_live_params(const LiveLateralParams &live, bool vehicle_valid, bool calibrated);

  /* modeld 온라인 캘리브레이션 상태(CalibrationStatus: 0 미완료, 1 완료, 2 범위 밖,
   * 3 재보정). 완료가 아니면 openpilot처럼 engage를 막고 engage 중이면 경고 후 해제한다.
   * 기본값은 완료라 단위 테스트와 리플레이 도구는 영향이 없다. */
  void set_calibration_status(uint32_t status) { calibration_status_ = status; }
  /* lagd 추정 지연. valid는 확정(5블록)이고 신선할 때. use_live_delay가 켜져 있으면 경로
   * 지연에 쓴다. */
  void set_live_delay(float delay_s, bool valid);
  bool live_delay_in_use() const;
  // 목표 곡률을 읽는 경로 지연: lagd 사용 중이면 추정값(0.15~0.65 s), 아니면 steer_actuator_delay
  float plan_delay_s() const;
  /* plan 나이를 잴 시계(monotonic_now_ns와 같은 시간축). 기본은 monotonic_now_ns이고, 검사와 재생
   * 도구가 기록 시각으로 바꾼다. */
  void set_clock(std::function<uint64_t()> clock) { clock_ = std::move(clock); }

  // 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
  LateralControlResult update(const LateralPath &path,
                              const LateralTarget &target,
                              const VehicleCanState &vehicle_state,
                              double now_s,
                              int frame,
                              bool panda_ready = true,
                              bool panda_controls_allowed = true);

private:
  // CLU 버튼 edge로 engage/disengage 상태를 갱신한다.
  void update_button_state(int button, double now_s);
  // update의 단계들(순서대로 부른다). 설명은 정의에 있다.
  LateralPath debounce_path(const LateralPath &path, double now_s);
  float plan_age_s(const LateralTarget &target) const;
  void disengage(double now_s);
  void resolve_engagement(BlockKind kind, bool logical_engaged, bool engage_requested, double now_s,
                          LateralControlResult *result);
  void update_road_bank(const VehicleCanState &vehicle_state, float speed_mps, bool yaw_rate_valid);
  /* 클립 전 목표 곡률. plan_curvature에는 묶기 전의 plan 곡률(상류 modelV2.action.desiredCurvature,
   * 포화 경고가 쓴다)을 적는다. */
  float requested_curvature(const LateralTarget &target, const VehicleCanState &vehicle_state, float speed_mps,
                            float plan_age_s, bool steering, bool steering_pressed, bool yaw_rate_valid,
                            const LiveLateralParams &live, float *plan_curvature);
  void steer(const VehicleCanState &vehicle_state, float speed_mps, double now_s, bool steering_pressed,
             bool yaw_rate_valid, bool curvature_limited, float plan_curvature, const LiveLateralParams &live,
             LateralControlResult *result);
  bool update_saturation(const LateralControlResult &result, float plan_curvature, float speed_mps, double now_s,
                         bool steering_pressed, bool curvature_limited);
  void copy_torque_state(LateralControlResult *result) const;

  // active를 막는 현재 gate reason을 계산한다.
  BlockReason active_block_reason(const LateralPath &path,
                                  const LateralTarget &target,
                                  const VehicleCanState &vehicle_state,
                                  bool seeds_ready,
                                  bool vehicle_fresh,
                                  bool panda_ready,
                                  bool panda_controls_allowed,
                                  float speed_kph,
                                  float plan_age_s,
                                  bool steer_fault_alert) const;

  /* 상류 car_events의 MDPS 일시 고장 경고 분류. 참이면 steerTempUnavailable(해제 예고), 거짓이면 조용한
   * 경고이거나 경고 없음(운전자가 넘겨받는 중, 최근에 넘겨받음, 정차). */
  bool update_steer_fault_alert(bool steer_fault, bool steering_pressed, bool standstill);

  // avoid_lkas_fault를 끈 경우: 상류 common_fault_avoidance로 85도 위에서 요청을 잠깐 끊는다.
  bool update_cut_steer_state(bool steer_request, float steering_angle_deg);

  // 큰 조향각 고장 회피: 85도 위 체류를 세고 요청을 끌지 정한다.
  bool update_large_angle_hold(bool steer_requested, float steering_angle_deg,
                               bool steering_pressed);

  // 노이즈가 있는 운전자 조향 토크를 openpilot 방식으로 필터링한다.
  bool update_steering_pressed(int driver_torque);

  // 제어 내부 상태를 초기값으로 되돌린다.
  void reset_control_state();


  // 최종 송신 frame 묶음을 만든다.
  std::vector<CanFrame> build_frames(const VehicleCanState &vehicle_state,
                                     const LateralControlResult &result,
                                     int frame);

  // LKAS11 counter를 seed frame 기준으로 openpilot 방식에 맞춰 증가시킨다.
  int next_lkas11_counter(const VehicleCanState &vehicle_state);

  // 스위치를 적용한 학습값
  LiveLateralParams live_params() const;

  LateralControllerConfig config_{};
  std::function<uint64_t()> clock_;
  TorqueController torque_controller_;
  LiveLateralParams live_{};
  bool live_vehicle_valid_ = true;
  bool live_calibrated_ = false;
  float live_delay_s_ = 0.0f;
  bool live_delay_valid_ = false;
  uint32_t calibration_status_ = 1;
  double soft_disable_start_s_ = -1.0;
  // openpilot LatControl.sat_time와 selfdrived의 최근 핸들 조작 시각
  float sat_time_ = 0.0f;
  double last_steering_pressed_s_ = -1000.0;
  // openpilot car_events의 조향 고장 경고 상태
  int steering_unpressed_frames_ = 0;
  bool last_steer_fault_ = false;
  bool no_steer_warning_ = false;
  bool silent_steer_warning_ = false;
  bool engaged_ = false;
  /* clip_curvature의 직전 출력. 비활성 중엔 상류 controlsd처럼 실제 곡률을 따라가
   * 재활성 때 거기서 한계 안으로 출발한다. */
  float prev_desired_curvature_ = 0.0f;
  /* path 유효성 디바운스: 차단은 즉시, 복귀는 연속 유효 0.5s 후.
   * 정지 부근에서 plan 도달거리가 경계를 넘나들며 active가 깜빡이고
   * 클러스터가 천이마다 부저를 울리는 것을 막는다. */
  double path_valid_since_s_ = -1.0;
  bool path_usable_debounced_ = false;
  bool path_seen_invalid_ = false;
  // 가용성 대기 중 steer_req/스푸프 유지(토크는 0) — 정차 부저 방지
  bool steer_availability_hold_ = false;
  int last_button_ = 0;
  int last_torque_ = 0;
  bool steer_limited_by_safety_ = false;
  double last_disengage_s_ = -1000.0;
  // steer 요청을 낸 채 85도 위에 머문 연속 프레임(요청을 끄면 멈춘다)
  int fault_angle_frames_ = 0;
  // 85도 위에서 요청을 끈 상태. 85도 아래로 와야 풀린다(운전자가 넘겨받았으면 15도 아래, 손을 떼야).
  bool large_angle_hold_ = false;
  // 이번 85도 위 체류 중 운전자가 핸들을 돌렸다(steering pressed)
  bool driver_took_wheel_ = false;
  // 지난 프레임에 steer 요청 비트를 실제로 보냈다(85도 위에서 새로 켜지 않기 위해)
  bool steer_req_sent_ = false;
  // 상류 CarController.angle_limit_counter(avoid_lkas_fault를 끈 경우만)
  int angle_limit_counter_ = 0;
  int steering_pressed_counter_ = 0;
  // 라이브 편경사 추정: bank = lat실측 + yaw_rate*v, 2초 저역통과, 직선에서만 갱신
  float road_bank_lat_accel_ = 0.0f;
  bool road_bank_init_ = false;
  int road_bank_stale_frames_ = 0;
  bool lkas11_counter_valid_ = false;
  int lkas11_counter_ = 0;
  // Panda health는 100 Hz 컨트롤러보다 낮은 주기로 발행된다.
  // 비동기 허가가 도착할 때까지 SET 요청을 잠시 보류하고, 이후에도 Panda나
  // 다른 gate가 차단 중이면 요청을 거부한다.
  bool panda_engage_pending_ = false;
  double panda_engage_pending_s_ = -1000.0;
};
