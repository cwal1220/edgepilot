#include "controls/lateral_controller.h"

#include "common/model_output.h"
#include "common/utils_math.h"
#include "common/utils_time.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPandaEngageGraceS = 1.0;
// 운전자가 넘겨받은 큰 회전에서 조향을 다시 켜는 각도(carrotpilot lat_suspend_control의 resume_angle)
constexpr float kDriverReleaseAngleDeg = 15.0f;
// 운전자가 넘겨받은 정지라도 핸들이 고장 각도(85도)보다 이만큼 아래이고 계획 곡률이 지금 핸들 곡률의 ±25% 안(같은 쪽)인
// 채로 1초 가면 다시 조향한다. 램프처럼 운전자가 85도를 잠깐 넘긴 뒤 커브를 붙잡고 있는 경우다. 교차로를 빠져나오며
// 펴는 핸들은 계획(직진)과 어긋나 그대로 쉰다: 2026-10-03 2:54 탈출은 계획이 직진인데 핸들이 77도인 채 다시 켜져
// 최대 토크로 핸들을 밀었다. 0.3초면 탈출 중 잠깐 멈춘 핸들에서도 켜졌다(녹화 15개 재생: 0.3초 26번 중 25번이 교차로
// 탈출, 1초는 9번 중 8번이고 기존보다 0.5~2.2초 일찍. 2026-10-05 16:46 램프는 두 경우 다 켜져 7~10초 일찍 돕는다).
constexpr float kHeldTurnResumeMarginDeg = 5.0f;
constexpr float kHeldTurnPlanTolerance = 0.25f;
constexpr int kHeldTurnResumeFrames = 100;

// 학습한 토크 모델 피드포워드를 섞는 무게의 틱당 변화(100 Hz에서 1초 램프)
constexpr float kNnBlendStep = 0.01f;
// 학습 데이터는 3 m/s 위만 썼다(nnlc csvdata.usable_mask). 그 아래에서는 선형 피드포워드로 돌아간다.
constexpr float kNnMinSpeedMps = 3.0f;
// 연이은 계획을 섞는 틱 수(50 ms, update_target_blend)
constexpr int kTargetBlendTicks = 5;
// 모델이 읽는 가장 먼 시각에 계획 나이 보정을 더해도 계획 격자 끝(model_t_idx(kLateralControlN - 1) = 2.5 s) 안이다
static_assert(kLateralNnLatestS + kMaxPlanAgeCompS <= 10.0f * (kLateralControlN - 1) * (kLateralControlN - 1) /
                                                              ((kTrajectorySize - 1.0f) * (kTrajectorySize - 1.0f)) +
                                                          1e-4f,
              "lateral NN inputs must stay inside the plan");

// plan 나이: 근거 프레임 캡처 시각부터 now_ns까지. lag 보상과 staleness gate가 함께 쓴다. 타임스탬프가 없으면(테스트,
// 초기값) 0으로 둔다.
float plan_age_s(const LateralTarget &target, uint64_t now_ns) {
    if (!target.valid || target.capture_timestamp_ns == 0 || now_ns <= target.capture_timestamp_ns) return 0.0f;
    return static_cast<float>(static_cast<double>(now_ns - target.capture_timestamp_ns) * 1e-9);
}

}  // namespace

LateralController::LateralController(LateralControllerConfig config) : config_(config) {}

// 제어 상태를 유지한 채 런타임 파라미터를 즉시 교체한다.
void LateralController::update_params(const SteeringParams &steering_params) {
    config_.steering_params = steering_params;
}

void LateralController::set_live_params(const LiveLateralParams &live, bool vehicle_valid) {
    live_ = live;
    live_vehicle_valid_ = vehicle_valid;
}

void LateralController::set_live_delay(float delay_s, bool valid) {
    live_delay_s_ = delay_s;
    live_delay_valid_ = valid;
}

bool LateralController::live_delay_in_use() const {
    return config_.steering_params.use_live_delay && live_delay_valid_;
}

float LateralController::plan_delay_s() const {
    // lagd가 지연을 min_lag~max_lag(0.15~0.65 s)로 묶는다(lateral_lag.cc)
    return live_delay_in_use() ? live_delay_s_ : config_.steering_params.steer_actuator_delay;
}

LiveLateralParams LateralController::live_params() const {
    LiveLateralParams live = live_;
    live.use_vehicle = live_.use_vehicle && config_.steering_params.use_live_vehicle_params;
    live.use_torque = live_.use_torque && config_.steering_params.use_live_torque_params;
    return live;
}

// 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
LateralControlOutput LateralController::update(const LateralPath &path, const LateralTarget &target,
                                               const VehicleCanState &vehicle_state, double now_s, int frame,
                                               const PandaGateOutput &panda, uint64_t now_ns) {
    nn_evaluated_ = false;
    const bool engage_requested =
        !config_.force_engaged && !engaged_ && vehicle_state.clu_button == 0 && last_button_ == kCruiseButtonSet;
    update_button_state(vehicle_state.clu_button, now_s);
    const bool logical_engaged = config_.force_engaged || engaged_;

    LateralControlOutput result;
    result.engaged = logical_engaged;
    const LateralPath gated_path = debounce_path(path, now_s);
    result.path_usable = gated_path.usable_for_steering;
    result.seeds_ready = seed_frames_ready(vehicle_state);
    result.vehicle_fresh = vehicle_state_fresh(vehicle_state, now_s, kVehicleStateTimeoutS);
    result.esp12_fresh = signal_time_fresh(vehicle_state.esp12_time_s, now_s, kVehicleStateTimeoutS);
    result.cluster_speed_kph = cluster_speed_kph(vehicle_state);
    const float raw_speed_kph = vehicle_speed_kph(vehicle_state, now_s, kVehicleStateTimeoutS);
    result.control_speed_kph = static_cast<float>(v_ego_filter_.update(raw_speed_kph / 3.6) * 3.6);
    const float speed_mps = result.control_speed_kph / 3.6f;
    const LiveLateralParams live = live_params();
    const float plan_age = plan_age_s(target, now_ns);
    // 노이즈가 있는 운전자 조향 토크를 openpilot 방식으로 거른다
    const bool steering_pressed =
        steering_pressed_.update(vehicle_state.driver_torque, config_.steering_params.steering_pressed_threshold);
    result.steering_pressed = steering_pressed;
    if (steering_pressed) last_steering_pressed_s_ = now_s;
    const bool wheels_stopped = vehicle_standstill(vehicle_state);
    result.steer_fault = vehicle_state.steer_fault_temporary;
    const BlockReason soft_reason =
        soft_disable_reason(update_steer_fault_alert(result.steer_fault, steering_pressed, wheels_stopped));
    result.active_block =
        active_block_reason(gated_path, target, vehicle_state, result.seeds_ready, result.vehicle_fresh, panda,
                            result.control_speed_kph, plan_age, soft_reason);
    const BlockKind kind = block_kind(result.active_block);
    resolve_engagement(kind, soft_reason != BlockReason::None, logical_engaged, engage_requested, now_s, &result);
    // 가용성 대기와 입력 끊김 동안에는 토크만 0으로 하고 steer_req/MDPS 속도 스푸프는
    // 유지한다 — 매 정차마다 끊기면 MDPS/클러스터가 천이 경보를 낸다.
    // 결함/해제는 즉시 끊는다.
    const bool steer_availability_hold =
        result.engaged && !result.active && (kind == BlockKind::Availability || kind == BlockKind::Interrupt);
    // LKAS가 켜져 있다(MDPS 속도 스푸프): 활성이거나 가용성 대기
    const bool lkas_active = result.active || steer_availability_hold;
    // MDPS가 고장을 알리는 동안은 요청 비트를 끈다(상류 latActive=false면 요청 비트도 0)
    const bool steer_request = lkas_active && !result.steer_fault;
    result.cut_steer_temp = update_cut_steer_state(steer_request, vehicle_state.steering_angle_deg);
    const bool plan_matches = large_angle_hold_ && driver_took_wheel_ &&
                              plan_matches_wheel(target, speed_mps, plan_age, vehicle_state.steering_angle_deg, live);
    result.large_angle_hold =
        update_large_angle_hold(steer_request, vehicle_state.steering_angle_deg, steering_pressed, plan_matches);
    result.large_angle_hold_by_driver = result.large_angle_hold && driver_took_wheel_;
    // 상류 latActive: 정차(최소 조향 속도·0.3 m/s 이하 또는 바퀴 정지)와 MDPS 일시 고장에서는 조향하지 않는다.
    // 큰 조향각에서 요청을 끈 동안도 활성이라도 조향을 쉰다.
    const bool standstill =
        !(speed_mps > std::max(config_.steering_params.min_steer_speed_mps, kStandstillSteerSpeedMps)) ||
        wheels_stopped;
    const bool steering = result.active && !result.large_angle_hold && !result.steer_fault && !standstill;
    result.lat_active = steering;
    const bool yaw_rate_valid = result.esp12_fresh && vehicle_state.yaw_rate_valid;
    road_bank_.update(kDtCtrlF, speed_mps, vehicle_state.lat_accel_mps2, vehicle_state.lat_accel_valid,
                      vehicle_state.yaw_rate_rad_s, yaw_rate_valid);

    const CurvatureBounds hold = hold_angle_bounds(speed_mps, steering, steering_pressed, live);
    update_target_blend(target);
    bool curvature_limited = false;
    float plan_curvature = 0.0f;
    result.desired_curvature = clip_curvature(speed_mps, prev_desired_curvature_,
                                              requested_curvature(target, vehicle_state, speed_mps, plan_age, steering,
                                                                  yaw_rate_valid, live, hold, &plan_curvature),
                                              live.use_vehicle ? live.roll_rad : 0.0f, &curvature_limited);
    prev_desired_curvature_ = result.desired_curvature;
    const float nn_feedforward =
        lateral_nn_feedforward(target, vehicle_state, speed_mps, plan_age, steering, live, hold);

    // 토크 제어기의 측정과 요청(조향하지 않는 틱에도 요청 버퍼와 저크 필터를 갱신한다)
    TorqueController::Input torque_in;
    torque_in.speed_mps = speed_mps;
    torque_in.desired_curvature = result.desired_curvature;
    torque_in.steering_angle_deg = vehicle_state.steering_angle_deg;
    torque_in.steer_limited_by_safety = steer_limited_by_safety_;
    torque_in.lat_delay_s = plan_delay_s();
    torque_in.yaw_rate_rad_s = vehicle_state.yaw_rate_rad_s;
    torque_in.yaw_rate_valid = yaw_rate_valid;
    torque_in.road_bank_lat_accel = road_bank_.lat_accel();
    torque_in.live = live;
    if (steering) {
        steer(vehicle_state, torque_in, now_s, steering_pressed, curvature_limited, plan_curvature, nn_feedforward,
              &result);
    } else {
        // 상류 LaC.reset(): 포화 시간을 지운다(PID는 토크 제어기가 비활성 틱에 지운다)
        sat_time_ = 0.0f;
        // 0을 넘기면 커브 중 결합할 때 지연 버퍼가 0-setpoint로 P를 튀게 한다
        torque_controller_.update(torque_in, config_.steering_params);
        copy_torque_state(&result);
        result.desired_torque = 0;
        result.apply_torque = 0;
    }

    // 결합 중이거나 해제 직후(kInactiveReleaseS) 동안은 토크 0으로 계속 보내 MDPS가 부드럽게 놓게 한다.
    result.should_send =
        result.seeds_ready && (result.active || result.engaged || now_s - last_disengage_s_ < kInactiveReleaseS);
    // 보내는 요청 비트: 큰 조향각에서 요청을 끈 동안은 내리고, cut_steer_temp(상류 2프레임 컷)는 패커가 내린다
    const bool steer_req_bit = steer_request && !result.large_angle_hold;
    if (result.should_send) {
        result.frames = build_frames(vehicle_state, result, steer_req_bit, lkas_active, frame);
    } else {
        lkas11_counter_valid_ = false;
    }

    steer_req_sent_ = result.should_send && steer_req_bit && !result.cut_steer_temp;
    last_torque_ = result.apply_torque;
    // 상류는 컨트롤러 출력(CC.actuators.torque)과 보낸 토크(actuatorsOutput.torque)를 정규화해 비교한다
    const float steer_max = static_cast<float>(kHyundaiSteerMax);
    steer_limited_by_safety_ =
        std::fabs(result.normalized_output * steer_max - static_cast<float>(result.apply_torque)) >
        kSteerLimitedBySafetyThreshold * steer_max;
    if (!result.active) {
        last_torque_ = 0;
        steer_limited_by_safety_ = false;
    }
    return result;
}

// path 복귀 디바운스: 무효는 즉시 반영, 무효를 겪은 뒤의 복귀는 0.5s
// 연속 유효를 요구한다(경계 깜빡임 차단). 최초 유효는 바로 통과.
LateralPath LateralController::debounce_path(const LateralPath &path, double now_s) {
    if (!path.usable_for_steering) {
        path_valid_since_s_ = -1.0;
        path_usable_debounced_ = false;
        path_seen_invalid_ = true;
    } else if (!path_usable_debounced_) {
        if (path_valid_since_s_ < 0.0) path_valid_since_s_ = now_s;
        if (!path_seen_invalid_ || now_s - path_valid_since_s_ >= 0.5) path_usable_debounced_ = true;
    }
    LateralPath gated_path = path;
    gated_path.usable_for_steering = path_usable_debounced_;
    return gated_path;
}

// 결합과 Panda 유예를 풀고 제어 상태를 처음으로 돌린다(해제·결합 거부·CANCEL 공통).
void LateralController::disengage(double now_s) {
    panda_engage_pending_ = false;
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
}

// 이번 틱의 차단 사유(kind)와 SoftDisable 사유(soft_condition)로 해제·결합 거부·Panda 유예를 정하고, result의
// engaged, soft_disabling, engage_rejected, active를 채운다.
void LateralController::resolve_engagement(BlockKind kind, bool soft_condition, bool logical_engaged,
                                           bool engage_requested, double now_s, LateralControlOutput *result) {
    // SoftDisable: 결합 중 사유가 이어지는 동안 3초를 세며 경고하고, 그 뒤에는 Hard처럼 해제한다. 정차·경로 무효
    // 같은 앞선 사유가 끼어도 타이머는 이어 센다(상류 softDisabling). 그 사유가 첫 사유일 때만 조향을 이어 간다.
    // 결합 요청이 Panda 허가를 기다리는 동안은 세지 않는다: 허가가 오면 이 사유로 결합을 거부한다(상류 NO_ENTRY).
    bool soft_disable_expired = false;
    if (logical_engaged && !engage_requested && !panda_engage_pending_ && soft_condition) {
        if (soft_disable_start_s_ < 0.0) soft_disable_start_s_ = now_s;
        soft_disable_expired = now_s - soft_disable_start_s_ >= kSoftDisableS;
    } else {
        soft_disable_start_s_ = -1.0;
    }
    const bool soft_disabling = soft_disable_start_s_ >= 0.0 && !soft_disable_expired;
    // 입력 끊김(Interrupt)과 결합 중 Panda 허가 철회(핸드셰이크가 아닌 Transient): 조향은 바로 쉬고 같은 해제
    // 예고를 띄운 채 센다. 그 안에 돌아오면 다시 조향하고, 넘으면 해제한다. 예전에는 해제도 경고도 없이 쉬다가
    // 입력이 돌아오면 운전자 확인 없이 다시 조향했다.
    const bool interrupted = logical_engaged && !engage_requested &&
                             (kind == BlockKind::Interrupt || (kind == BlockKind::Transient && !panda_engage_pending_));
    bool interrupt_expired = false;
    if (interrupted) {
        if (interrupt_start_s_ < 0.0) interrupt_start_s_ = now_s;
        interrupt_expired = now_s - interrupt_start_s_ >= kInterruptDisengageS;
    } else {
        interrupt_start_s_ = -1.0;
    }
    result->soft_disabling =
        soft_disabling || (interrupted && !interrupt_expired && now_s - interrupt_start_s_ >= kInterruptAlertS);
    if (logical_engaged && (kind == BlockKind::Hard || soft_disable_expired || interrupt_expired)) {
        disengage(now_s);
        result->engaged = config_.force_engaged;
        result->soft_disabling = false;
    }

    if (engage_requested) {
        if (kind == BlockKind::Transient) {
            panda_engage_pending_ = true;
            panda_engage_pending_s_ = now_s;
        } else {
            panda_engage_pending_ = false;
        }
    }

    // 차량/컨트롤러의 정적 gate는 실제 결합 요청 실패로 처리한다(그 요청의 Panda 유예는 위에서 내렸다).
    bool rejected = engage_requested && (kind == BlockKind::Hard || kind == BlockKind::Reject ||
                                         kind == BlockKind::SoftDisable || kind == BlockKind::Interrupt);
    if (panda_engage_pending_) {
        const bool panda_waiting = kind == BlockKind::Transient;
        const bool grace_elapsed = now_s - panda_engage_pending_s_ >= kPandaEngageGraceS;
        if (kind == BlockKind::None || kind == BlockKind::Availability) {
            // Panda 허가가 도착했고 나머지는 가용성 상태뿐이면 결합을 유지한다.
            panda_engage_pending_ = false;
        } else if (!panda_waiting || grace_elapsed) {
            // 정적 실패를 저장했다가 gate가 해소되면 조용히 결합하지 않는다. Panda에는
            // 짧은 비동기 health handshake 유예만 허용하며, 완료되지 않으면 실제 차단
            // 사유를 한 번 보고한다.
            rejected = true;
        }
    }
    if (rejected) {
        disengage(now_s);
        result->engaged = config_.force_engaged;
        result->engage_rejected = true;
    }
    result->active = kind == BlockKind::None || (kind == BlockKind::SoftDisable && soft_disabling);
}

// 연이은 계획 사이의 출력 섞기. 계획은 20 Hz로 바뀌는데 목표 곡률은 100 Hz 틱마다 새 계획 위로 계단처럼 뛰고, 토크
// 변화율 한계(올림 3, 내림 7)가 그 계단에 비대칭으로 걸려 곡선에서 보낸 토크가 요구의 0.81~0.90에 머물렀다(2026-10-10~
// 10-18 주행). 상류 controlsd는 같은 자리를 LAT_SMOOTH_SECONDS로 거른다. 섞는 50 ms는 폐루프 재생(2026-10-22)에서 토크
// 단계 RMS를 2.65 → 2.18, 곡선의 변화율 한계 걸림을 59 → 37 %로 줄였다(100 ms는 덜 줄었다). 혼자서는 곡선에서 차가
// 0.6~1.3 cm 안쪽으로 갔고, Lane MPC의 출발 곡률 섞기(lateral_planner)와 함께면 남지 않았다.
void LateralController::update_target_blend(const LateralTarget &target) {
    TargetBlend &blend = target_blend_;
    if (!blend.have_current || target.capture_timestamp_ns != blend.current.capture_timestamp_ns) {
        blend.have_previous = blend.have_current && blend.current.valid;
        if (blend.have_previous) blend.previous = blend.current;
        blend.current = target;
        blend.have_current = true;
        blend.ticks = 0;
    } else {
        ++blend.ticks;
    }
    blend.previous_extra_age_s =
        blend.have_previous
            ? static_cast<float>(static_cast<double>(static_cast<int64_t>(blend.current.capture_timestamp_ns -
                                                                          blend.previous.capture_timestamp_ns)) *
                                 1e-9)
            : 0.0f;
    blend.weight = blend.have_previous
                       ? std::min(1.0f, static_cast<float>(blend.ticks + 1) / static_cast<float>(kTargetBlendTicks))
                       : 1.0f;
}

// 클립 전 목표 곡률. 상류 controlsd: 활성이면 plan, 비활성이면 실제 곡률을 클립에 넣는다. 재활성 때
// 목표가 실제 곡률에서 한계 안으로 출발하고, 비활성 중의 잘못된 plan 값이 넘어오지 않는다.
float LateralController::requested_curvature(const LateralTarget &target, const VehicleCanState &vehicle_state,
                                             float speed_mps, float plan_age_s, bool steering, bool yaw_rate_valid,
                                             const LiveLateralParams &live, const CurvatureBounds &hold,
                                             float *plan_curvature) {
    const SteeringParams &control_params = config_.steering_params;
    float requested_curvature = steering ? lag_adjusted_curvature(target, speed_mps, plan_age_s, plan_delay_s())
                                         : torque_controller_.estimate_actual_curvature(
                                               speed_mps, vehicle_state.steering_angle_deg, control_params,
                                               vehicle_state.yaw_rate_rad_s, yaw_rate_valid, live);
    const TargetBlend &blend = target_blend_;
    if (steering && blend.weight < 1.0f) {
        const float previous =
            lag_adjusted_curvature(blend.previous, speed_mps, plan_age_s + blend.previous_extra_age_s, plan_delay_s());
        if (std::isfinite(previous)) requested_curvature = previous + blend.weight * (requested_curvature - previous);
    }
    *plan_curvature = requested_curvature;
    // 운전자가 핸들을 잡지 않았을 때는 목표를 avoid_lkas_fault_hold_angle_deg(80도) 조향각이 내는
    // 곡률 안으로 묶는다(같은 차량 모델, 학습 SR·강성·오프셋·롤). 컨트롤러가 스스로 85도를 넘겨
    // 0.89초 뒤 토크가 빠지고 핸들이 풀렸다 다시 잡는 반복 대신, 그 각도에서 토크를 끊김 없이
    // 유지한다. 운전자가 조향 중이면 묶지 않는다: 교차로에서 더 감는 운전자를 80도 쪽으로 밀지
    // 않고, 85도 위에서는 시간 규칙이 그대로 적용된다. 횡가속 한계가 더 좁은 약 35 km/h 위에서는
    // 걸리지 않는다.
    if (hold.active) requested_curvature = std::clamp(requested_curvature, hold.lo, hold.hi);
    return requested_curvature;
}

LateralController::CurvatureBounds LateralController::hold_angle_bounds(float speed_mps, bool steering,
                                                                        bool steering_pressed,
                                                                        const LiveLateralParams &live) {
    const SteeringParams &control_params = config_.steering_params;
    const float hold_angle_deg = control_params.avoid_lkas_fault_hold_angle_deg;
    CurvatureBounds bounds;
    // 조향 중이면 바퀴 속도가 유효하다(낡으면 vehicle_state_stale로 조향하지 않는다)
    if (!steering || steering_pressed || !control_params.avoid_lkas_fault_enabled || !(hold_angle_deg > 0.0f))
        return bounds;
    const float left = torque_controller_.curvature_at_angle(speed_mps, hold_angle_deg, control_params, live);
    const float right = torque_controller_.curvature_at_angle(speed_mps, -hold_angle_deg, control_params, live);
    bounds.active = true;
    bounds.lo = std::min(left, right);
    bounds.hi = std::max(left, right);
    return bounds;
}

// 토크 제어기의 진단값을 결과에 옮긴다.
void LateralController::copy_torque_state(LateralControlOutput *result) const {
    result->actual_curvature = torque_controller_.actual_curvature();
    result->actual_curvature_vm = torque_controller_.actual_curvature_vm();
    result->actual_curvature_yaw = torque_controller_.actual_curvature_yaw();
    result->curvature_error = result->desired_curvature - result->actual_curvature;
    result->normalized_output = torque_controller_.normalized_output();
    result->feedforward = torque_controller_.feedforward();
    result->friction_torque = torque_controller_.friction_torque();
    result->nn_feedforward_used = torque_controller_.nn_feedforward_used();
}

// 실행 중 다른 모델로 갈아 끼울 때(웹 콘솔 고르기)는 이력과 섞기 무게를 둔다: 이력은 모델과 상관없는 목표
// 궤적이라, 피드포워드는 선형으로 떨어졌다 다시 오르지 않고 두 모델 출력의 차이만큼만 움직인다.
void LateralController::set_lateral_nn(std::shared_ptr<const LateralNnModel> model) {
    const bool swap = lateral_nn_ && model && model->loaded();
    lateral_nn_ = model && model->loaded() ? std::move(model) : nullptr;
    if (swap) return;
    nn_inputs_.reset();
    nn_weight_ = 0.0f;
}

// 학습한 토크 모델의 입력은 목표 궤적이다(학습은 v3이 실제 궤적, v4가 이 운용 입력이다. docs/lateral-nn.md).
// - la(δ): 계획 곡률(MPC 궤적을 계획 나이만큼 앞으로 보간) × v². PID가 따르는 요청과 같은 범위로 묶는다: 운전자가
//   핸들을 잡지 않았을 때의 80도 유지 한계(hold_angle_bounds), 횡가속·곡률 상한(limit_curvature). 계획이 NaN이면 그
//   틱은 모델도 NaN이라 선형 피드포워드를 쓴다. 조향각(δ): 그 곡률을 차량 모델로 뒤집은 각도(학습 SR·강성·영점).
// - roll: 편경사 추정 / g(학습 데이터와 같은 추정기). long: ESP12 종가속.
float LateralController::lateral_nn_feedforward(const LateralTarget &target, const VehicleCanState &vehicle_state,
                                                float speed_mps, float plan_age_s, bool steering,
                                                const LiveLateralParams &live, const CurvatureBounds &hold) {
    if (!lateral_nn_) return NAN;
    const float speed = std::isfinite(speed_mps) ? std::max(speed_mps, 0.0f) : 0.0f;
    const float age = clamp_float(plan_age_s, 0.0f, kMaxPlanAgeCompS);
    const float roll_rad = live.use_vehicle ? live.roll_rad : 0.0f;
    const TargetBlend &blend = target_blend_;
    const auto curvature_at = [&](float delta_s) {
        float curvature = interp_lateral(delta_s + age, target.curvatures);
        if (blend.weight < 1.0f) {
            const float previous =
                interp_lateral(delta_s + age + blend.previous_extra_age_s, blend.previous.curvatures);
            if (std::isfinite(previous) && std::isfinite(curvature))
                curvature = previous + blend.weight * (curvature - previous);
        }
        if (!std::isfinite(curvature)) return NAN;
        return limit_curvature(speed, std::clamp(curvature, hold.lo, hold.hi), roll_rad);
    };
    const float speed_sq = speed * speed;
    const TorqueController::AngleMap angle = torque_controller_.angle_map(speed, config_.steering_params, live);
    const auto la_at = [&](float delta_s) { return curvature_at(delta_s) * speed_sq; };
    const auto angle_at = [&](float delta_s) { return angle(curvature_at(delta_s)); };
    nn_inputs_.push(la_at(0.0f), angle_at(0.0f));
    // 켜고 끌 때, 그리고 조향을 시작할 때 피드포워드가 건너뛰지 않게 1초에 걸쳐 섞는다(설정은 주행 중에도 바뀐다).
    // 학습에서 뺀 순간(desire가 있거나 깜빡이가 켜진 차선 변경·회전, nnlc csvdata)에는 선형으로 돌아간다.
    const bool trained_situation =
        target.desire == Desire::None && !vehicle_state.left_blinker && !vehicle_state.right_blinker;
    const bool wanted = steering && config_.steering_params.nn_feedforward && target.valid && speed > kNnMinSpeedMps &&
                        trained_situation;
    nn_weight_ = clamp_float(nn_weight_ + (wanted ? kNnBlendStep : -kNnBlendStep), 0.0f, 1.0f);
    if (!steering || !target.valid || nn_weight_ <= 0.0f) return NAN;
    nn_evaluated_ = true;
    return nn_inputs_.evaluate(*lateral_nn_, speed, road_bank_.lat_accel() / 9.81f, vehicle_state.long_accel_mps2,
                               la_at, angle_at);
}

// 조향 중: 토크를 계산하고 85도 고장 회피와 Panda 한계를 적용한 뒤 포화를 본다.
void LateralController::steer(const VehicleCanState &vehicle_state, const TorqueController::Input &torque_in,
                              double now_s, bool steering_pressed, bool curvature_limited, float plan_curvature,
                              float nn_feedforward, LateralControlOutput *result) {
    const SteeringParams &control_params = config_.steering_params;
    const float speed_mps = torque_in.speed_mps;
    // 85도 위에서도 컨트롤러 토크를 그대로 내되, 요청을 끄는 프레임(avoid_lkas_fault_max_frames,
    // update_large_angle_hold)에 0에 닿도록 크기를 steer_delta_down × 남은 프레임으로 묶는다.
    // 하강 레이트 한계로 내려와도 늦지 않는 만큼만 남기는 것이라, 최대 토크로 85도를 넘으면
    // 약 0.35초는 그대로 두고 0.55초에 걸쳐 내린다(2026-10-03까지는 0.69초 선형 램프로 0).
    // 요청은 토크가 0일 때 꺼지므로 어시스트가 빠지는 "탁"이 없다. 적분기는 85도 위에서 얼린다.
    const bool above_fault_angle =
        control_params.avoid_lkas_fault_enabled &&
        std::fabs(vehicle_state.steering_angle_deg) >= control_params.avoid_lkas_fault_max_angle_deg;
    // steer()는 조향 중(큰 조향각에서 요청을 끈 동안이 아닐 때)에만 불린다
    int torque_cap = kHyundaiSteerMax;
    if (fault_angle_frames_ > 0) {
        torque_cap =
            std::min(torque_cap, kHyundaiSteerDeltaDown *
                                     std::max(0, control_params.avoid_lkas_fault_max_frames - fault_angle_frames_));
    }
    TorqueController::Input in = torque_in;
    in.active = true;
    in.steering_pressed = steering_pressed;
    in.steer_limited_by_safety = steer_limited_by_safety_ || above_fault_angle;
    TorqueController::NnFeedforward nn;
    nn.torque = nn_feedforward;
    nn.weight = nn_weight_;
    nn.curve_friction = lateral_nn_ && lateral_nn_->spec().friction_free;
    nn.curve_gain = lateral_nn_ ? lateral_nn_->curve_gain() : 0.0f;
    const int raw_torque = torque_controller_.update(in, control_params, nn);
    result->desired_torque = std::clamp(raw_torque, -torque_cap, torque_cap);
    copy_torque_state(result);
    result->apply_torque =
        apply_hyundai_steer_torque_limits(result->desired_torque, last_torque_, vehicle_state.driver_torque);
    result->steer_saturated =
        update_saturation(*result, plan_curvature, speed_mps, now_s, steering_pressed, curvature_limited);
    result->lat_saturated = lac_saturated_;
}

// openpilot LatControl._check_saturation + selfdrived steerSaturated. 출력이 한계에
// 붙었거나 곡률이 횡가속 한계에 잘렸는데, 안전 한계(토크 레이트)나 운전자 때문이
// 아닐 때 0.4초 누적되면 포화다. 커브에서 plan 목표 횡가속(클립 전, 상류는 모델의
// desiredCurvature)이 실제의 1.2배를 넘고 1 m/s² 이상이며 최근 2초 안에 핸들을 잡지
// 않았으면 경고한다.
bool LateralController::update_saturation(const LateralControlOutput &result, float plan_curvature, float speed_mps,
                                          double now_s, bool steering_pressed, bool curvature_limited) {
    const bool saturated_now = std::fabs(result.normalized_output) > 1.0f - 1e-3f || curvature_limited;
    if (saturated_now && speed_mps > kSatCheckMinSpeedMps && !steer_limited_by_safety_ && !steering_pressed)
        sat_time_ += kDtCtrlF;
    else
        sat_time_ -= kDtCtrlF;
    sat_time_ = clamp_float(sat_time_, 0.0f, kSteerLimitTimerS);
    lac_saturated_ = sat_time_ > kSteerLimitTimerS - 1e-3f;
    const bool lac_saturated = lac_saturated_;
    const bool recent_steer_pressed = now_s - last_steering_pressed_s_ < 2.0;
    const float clipped_speed = std::max(speed_mps, 0.3f);
    const float desired_lat_accel = plan_curvature * clipped_speed * clipped_speed;
    const float actual_lat_accel = result.actual_curvature * clipped_speed * clipped_speed;
    const bool undershooting = std::fabs(desired_lat_accel) / std::fabs(1e-3f + actual_lat_accel) > 1.2f;
    const bool turning = std::fabs(desired_lat_accel) > 1.0f;
    return !recent_steer_pressed && undershooting && turning && lac_saturated;
}

// CLU 버튼 edge로 결합·해제 상태를 갱신한다.
void LateralController::update_button_state(int button, double now_s) {
    if (button == last_button_) return;
    if (button == kCruiseButtonCancel) {
        disengage(now_s);
    } else if (button == 0 && last_button_ == kCruiseButtonSet) {
        engaged_ = true;
    }
    last_button_ = button;
}

// avoid_lkas_fault를 끈 경우: 상류 hyundai carcontroller의 common_fault_avoidance(MAX_ANGLE, MAX_ANGLE_FRAMES,
// MAX_ANGLE_CONSECUTIVE_FRAMES). 요청을 낸 채 85도 위에 머문 프레임을 세어 89를 넘으면 2프레임 요청을 끄고
// (ToiFlt를 켜고 토크는 그대로) 다시 센다. K7은 끈 요청을 85도 위에서 다시 켜면 곧 고장을 내므로 기본값은
// 켠 쪽(update_large_angle_hold)이다.
bool LateralController::update_cut_steer_state(bool steer_request, float steering_angle_deg) {
    const SteeringParams &params = config_.steering_params;
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

// 상류 car_events: MDPS 일시 고장이 운전자가 핸들을 잡은 채 시작했으면 경고하지 않고, 운전자가 놓은 지
// 1.5초가 안 됐거나 정차면 조용한 경고(그 고장 동안 유지), 그 밖에는 steerTempUnavailable이다. 어느 쪽이든
// 고장 동안 조향은 쉰다.
bool LateralController::update_steer_fault_alert(bool steer_fault, bool steering_pressed, bool standstill) {
    steering_unpressed_frames_ =
        steering_pressed ? 0 : std::min(steering_unpressed_frames_ + 1, kSilentSteerWarningFrames);
    bool alert = false;
    if (steer_fault) {
        if (steering_pressed && (!last_steer_fault_ || no_steer_warning_)) {
            no_steer_warning_ = true;
        } else {
            no_steer_warning_ = false;
            if (silent_steer_warning_ || standstill || steering_unpressed_frames_ < kSilentSteerWarningFrames)
                silent_steer_warning_ = true;
            else
                alert = true;
        }
    } else {
        no_steer_warning_ = false;
        silent_steer_warning_ = false;
    }
    last_steer_fault_ = steer_fault;
    return alert;
}

// 큰 조향각 고장 회피. K7 MDPS는 steer 요청이 켜진 채 |조향각|이 85도 위에 0.98~1.12초 머물면
// 토크와 무관하게 ToiFlt+FailStat을 내고, 85도 아래로 와야 푼다(2026-09-18 실측). 요청을 내는
// 동안(active, 또는 정차 같은 가용성 대기) 85도 위 체류를 세어 avoid_lkas_fault_max_frames에 닿으면
// 요청을 끈다. 그 프레임까지 토크는 update가 0으로 내려 둔다. openpilot식 2프레임 컷은 85도 위에서
// 요청을 다시 켜는 순간 3~14 ms 뒤 고장을 냈고(2026-10-03, 4번 중 4번), 정차 대기 중에는 세지도
// 않아 1초 뒤 고장이 났다. 그래서 85도 위에서는 요청을 새로 켜지 않는다(결합이나 차단 해제가
// 85도 위에서 일어나도 같다).
// 끈 요청은 85도 아래로 오면 다시 켠다. 다만 그 체류 중 운전자가 핸들을 돌렸으면(넘겨받은 회전) 핸들이
// kDriverReleaseAngleDeg(15도) 아래로 오고 손을 떼거나(carrotpilot lat_suspend_control과 같은 해제 조건), 80도
// 아래에서 계획이 지금 핸들과 같은 커브를 1초 가리킬 때까지 끈 채로 둔다. 회전을 빠져나오며 핸들을 펴는 운전자를
// 시스템이 미리 밀지 않고(2026-10-03 2:54 탈출에서 최대 213), 램프에서 85도를 잠깐 넘긴 뒤 커브를 붙잡은 운전자에게는
// 커브가 끝날 때까지 손을 놓게 하지 않는다(2026-10-05 16:46 램프는 14초 동안 토크 0).
bool LateralController::update_large_angle_hold(bool steer_requested, float steering_angle_deg, bool steering_pressed,
                                                bool plan_matches_wheel) {
    const SteeringParams &params = config_.steering_params;
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
    if (!steer_req_sent_) {
        large_angle_hold_ = true;
    } else if (!large_angle_hold_ && ++fault_angle_frames_ >= params.avoid_lkas_fault_max_frames) {
        large_angle_hold_ = true;
    }
    return large_angle_hold_;
}

// 제어 내부 상태를 초기값으로 되돌린다. 토크 PID는 해제 뒤 첫 비활성 틱에 토크 제어기가 지운다.
void LateralController::reset_control_state() {
    last_torque_ = 0;
    steer_limited_by_safety_ = false;
    fault_angle_frames_ = 0;
    large_angle_hold_ = false;
    driver_took_wheel_ = false;
    held_turn_frames_ = 0;
    angle_limit_counter_ = 0;
}

bool LateralController::plan_matches_wheel(const LateralTarget &target, float speed_mps, float plan_age_s,
                                           float steering_angle_deg, const LiveLateralParams &live) {
    if (!target.valid || !std::isfinite(speed_mps)) return false;
    const float plan = lag_adjusted_curvature(target, speed_mps, plan_age_s, plan_delay_s());
    const float wheel =
        torque_controller_.curvature_at_angle(speed_mps, steering_angle_deg, config_.steering_params, live);
    return plan * wheel > 0.0f && std::fabs(plan - wheel) <= kHeldTurnPlanTolerance * std::fabs(wheel);
}

// active를 막는 현재 gate reason을 계산한다.
BlockReason LateralController::active_block_reason(const LateralPath &path, const LateralTarget &target,
                                                   const VehicleCanState &vehicle_state, bool seeds_ready,
                                                   bool vehicle_fresh, const PandaGateOutput &panda, float speed_kph,
                                                   float plan_age_s, BlockReason soft_reason) const {
    // 순서 규칙: 데이터 유효성 -> 차량 결함(즉시 해제) -> 핸드셰이크 ->
    // 가용성 대기. 결함이 뒤로 밀리면 앞선 일시적 사유가 결함을 가리고, 그
    // 사이 결합이 유예되어 톤만 울렸다가 해제된다.
    if (!config_.force_engaged && !engaged_) return BlockReason::NotEngaged;
    if (!config_.steering_params.enabled) return BlockReason::ControllerDisabled;
    if (!seeds_ready) return BlockReason::SeedsMissing;
    if (!vehicle_fresh) return BlockReason::VehicleStateStale;  // 바퀴 속도도 같은 시간 안에 왔다(유한하다)
    if (vehicle_state.door_open) return BlockReason::DoorOpen;
    if (vehicle_state.seatbelt_unlatched) return BlockReason::SeatbeltUnlatched;
    if (vehicle_state.esp_disabled) return BlockReason::EspDisabled;
    if (vehicle_state.park_brake) return BlockReason::ParkBrake;
    if (vehicle_state.brake_error) return BlockReason::BrakeError;
    if (vehicle_state.gear != kGearDrive) return BlockReason::GearNotDrive;
    // Panda 핸드셰이크는 차량 결함 뒤에 온다. 앞에 두면 시동 직후 health가
    // 도착하기 전의 결합 요청이 panda_not_ready(일시적)로 분류되어 유예되고,
    // 안전벨트/기어 같은 하드 결함이 가려진 채 결합 톤이 울린 뒤 해제된다.
    if (!panda.ready) return BlockReason::PandaNotReady;
    if (!panda.controls_allowed) return BlockReason::PandaControlsOff;
    if (!path.usable_for_steering) {
        // 모델이 낡았거나 무효면 속도와 상관없이 입력이 끊긴 것이다(해제 예고 뒤 해제).
        if (path.model_lost) return BlockReason::ModelLost;
        // 정지에서는 plan이 원래 짧아 path 무효가 정상이다. 오류가 아니라
        // 대기로 보고한다. 이 속도 밑은 min_steer_speed로 토크도 0이다.
        if (speed_kph / 3.6f < config_.steering_params.min_steer_speed_mps) return BlockReason::Stopped;
        return BlockReason::PathInvalid;
    }
    if (!target.valid || !target.mpc_solution_valid) return BlockReason::LateralPlanInvalid;
    // 모델 경로 gate는 모델 발행 시각만 본다. 플랜은 근거 프레임 캡처 시각으로도 막아, 늦게 발행된
    // 낡은 프레임의 플랜도 쓰지 않는다.
    if (target.capture_timestamp_ns != 0 && plan_age_s > static_cast<float>(kModelTimeoutS))
        return BlockReason::LateralPlanStale;
    // SoftDisable 사유는 맨 끝에 둔다.
    return soft_reason;
}

// openpilot steerTempUnavailable, calibrationIncomplete/Recalibrating/Invalid, paramsdTemporaryError. MDPS 고장
// 동안 조향을 쉬는 것은 이 사유와 따로 update가 정한다(조용한 경고일 때도 쉰다). paramsd는 쓰는 중이고 보정이
// 끝났을 때만 본다(상류 sm.seen·cal_status 검사: 보정 사유가 먼저 돌아간다).
BlockReason LateralController::soft_disable_reason(bool steer_fault_alert) const {
    if (steer_fault_alert) return BlockReason::SteerTempUnavailable;
    if (calibration_status_ == CalibrationStatus::Uncalibrated) return BlockReason::CalibrationIncomplete;
    if (calibration_status_ == CalibrationStatus::Recalibrating) return BlockReason::CalibrationRecalibrating;
    if (calibration_status_ != CalibrationStatus::Calibrated) return BlockReason::CalibrationInvalid;
    if (live_params().use_vehicle && !live_vehicle_valid_) return BlockReason::ParamsdInvalid;
    return BlockReason::None;
}

// 최종 송신 frame 묶음을 만든다.
std::vector<CanFrame> LateralController::build_frames(const VehicleCanState &vehicle_state,
                                                      const LateralControlOutput &result, bool steer_req,
                                                      bool lkas_active, int frame) {
    HyundaiLkasCommand command;
    command.apply_steer = result.apply_torque;
    command.steer_req = steer_req;
    command.cut_steer_temp = result.cut_steer_temp;
    // 클러스터 LKAS 표시(양쪽 차선 보임): 3 작동, 4 대기. 클러스터는 sys_state 천이마다 부저를
    // 울리므로 active(정차 대기 등 가용성)가 아니라 결합(engaged)을 따라 enable/disable에서만 울린다.
    // steer_req는 별도 비트로 매 프레임 정확히 나간다.
    command.sys_state = result.engaged ? 3 : 4;
    command.sys_warning = false;
    command.lkas_msg_count = next_lkas11_counter(vehicle_state);

    // should_send면 세 seed가 다 있다(seed_frames_ready).
    return build_lateral_can_frames(decode_lkas11(vehicle_state.lkas11_seed), decode_clu11(vehicle_state.clu11_seed),
                                    vehicle_state.mdps12_seed, command, config_.steering_params.mdps_speed_spoof_kph,
                                    lkas_active, vehicle_state.speed_unit_mph, frame);
}

int LateralController::next_lkas11_counter(const VehicleCanState &vehicle_state) {
    if (!lkas11_counter_valid_) {
        const HyundaiLkas11Values seed = decode_lkas11(vehicle_state.lkas11_seed);
        lkas11_counter_ = (seed.msg_count + 1) & 0xf;
        lkas11_counter_valid_ = true;
    }
    const int counter = lkas11_counter_ & 0xf;
    lkas11_counter_ = (lkas11_counter_ + 1) & 0xf;
    return counter;
}
