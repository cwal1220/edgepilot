#include "controls/lateral_torque.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "car/hyundai_can.h"
#include "common/utils_math.h"
#include "common/utils_time.h"  // kDtCtrlF

namespace {

constexpr float kFrictionThreshold = 0.2f;
// 저크 선행 마찰: 0.19s 앞의 요청 저크를 1.2Hz LP로 걸러 미리 반영한다.
constexpr int kJerkLookaheadFrames = 19;
constexpr float kJerkGain = 0.3f;
constexpr float kJerkFilterAlpha = 1.0f / (1.0f + 1.0f / (2.0f * kPiF * 1.2f * kDtCtrlF));
// openpilot INTERP_SPEEDS. 저속 이득은 이 곡선이 담당하고 LOW_SPEED_Y 곡률항은
// 쓰지 않는다(상류 PR #36364). 마지막 점만 파라미터(torque_kp)다.
constexpr float kKpSpeedsMps[] = {1.0f, 1.5f, 2.0f, 3.0f, 5.0f, 7.5f, 10.0f, 15.0f, 30.0f};
constexpr float kKpValues[] = {250.0f, 120.0f, 65.0f, 30.0f, 11.5f, 5.5f, 3.5f, 2.0f};
// 저속 떨림 대책(2026-10-15). 20~50 km/h 직선에서 핸들이 1.7~2.3 Hz, 3~7도로 떨렸다(10-10~10-15 주행 7건, 조향 중
// 직선 시간의 10~20 km/h 15.5%, 20~30 km/h 11.5%, 30~40 km/h 5.4%, 80 km/h 위 0%). 요청 곡률은 평평한데 P항과 마찰
// 보상이 조향각을 되밀고, 그 요청이 토크 변화율 한계(3/7)에 걸려 40~50도 늦게 나가 루프가 스스로 돈다. 핸들이 가벼운
// 저속일수록 루프 이득이 크다. 마찰 보상 deadzone과 직선 이득 배율을 저속에만 건다.
// 조향각 deadzone은 횡가속으로 v^2에 비례해 커진다. 그대로 두면 80 km/h 위에서 마찰 보상의 90%가 사라지므로
// 40 km/h부터 줄여 60 km/h에서 끈다.
// 학습한 토크 모델에 마찰 보상과 곡선 이득을 거는 곡선 범위(|목표 횡가속|, m/s²): 앞 값 아래(직선)는 모델만, 뒤
// 값부터 전부. 모델은 목표 궤적만 보고 추종 오차를 모른다(TorqueController::update). sunnypilot NNLC도 오차 응답을 높은
// 횡가속에서만 섞는다(error_blend_factor, 1→2 m/s²); 여기서는 진입 부족이 0.4~1.5 m/s²에 있어 더 낮게 시작한다.
constexpr float kNnCurveStartLatAccel = 0.2f;
constexpr float kNnCurveFullLatAccel = 0.6f;
// 곡선 이득(모델 파일 curve_gain)을 거는 속도: 30 km/h 아래는 없고 50 km/h부터 전부. 폐루프 재생(2026-10-18, 주행
// 여섯)에서 모델의 곡선 부족은 40~80 km/h에 있었고 20~40 km/h 진입은 선형과 같았다.
constexpr float kNnCurveGainStartSpeedMps = 30.0f / 3.6f;
constexpr float kNnCurveGainFullSpeedMps = 50.0f / 3.6f;
// 곡선 이득은 |목표 횡가속|이 커지는 동안(곡선 진입)만 건다: 그 빠르기를 τ 약 0.1 s로 걸러 0이면 없고 0.3 m/s³부터 전부.
// 유지·탈출에도 걸면 80 km/h 위 정상 곡선에서 조향각이 요구보다 7~9 % 더 나가 차가 안쪽으로 붙었다(2026-10-18 주행 둘,
// 횡가속 1 m/s² 넘는 유지 구간의 각도/요구 비 좌 1.068, 우 1.093). 폐루프 재생(2026-10-22)에서 우곡선 유지의 안쪽 치우침이
// 11.0 → 7.0 cm였고 0.5~3 Hz 횡가속은 늘지 않았다. 이득을 아예 빼면 좌곡선이 진입부터 바깥으로 갔다(탈출 -10 cm).
constexpr float kNnCurveGainFullRise = 0.3f;
constexpr float kNnCurveRiseFilterAlpha = 0.1f;  // 10 ms 틱당
constexpr float kDeadzoneFullSpeedMps = 40.0f / 3.6f;
constexpr float kDeadzoneOffSpeedMps = 60.0f / 3.6f;
// 직선 이득 배율은 10 m/s까지 다 쓰고 15 m/s에서 1로 돌린다.
constexpr float kStraightKpFullSpeedMps = 10.0f;
constexpr float kStraightKpOffSpeedMps = 15.0f;
// 요청 곡률(지금 요청과 지연 비교 대상 중 큰 쪽)이 이보다 작으면 직선, kTurnCurvature 위면 회전이라 배율이 1이다.
// 저속 회전은 P항이 토크 대부분을 낸다(10~30 km/h 회전에서 |P| 평균 312~646, FF 78~103).
constexpr float kStraightCurvature = 0.003f;
constexpr float kTurnCurvature = 0.008f;
// opendbc VehicleDynamicsParams: 강성과 관성은 이 기준 차(시빅 1326 kg + 짐 136 kg)의 질량으로 나눈 값으로 쓴다.
// 강성이 질량에 비례해 차량 모델에서 질량이 약분되므로 상류처럼 단위 질량으로 계산한다.
constexpr float kCivicMass = 1462.0f;
constexpr float kCivicWheelbase = 2.70f;
constexpr float kCivicCenterToFront = kCivicWheelbase * 0.4f;
constexpr float kCivicCenterToRear = kCivicWheelbase - kCivicCenterToFront;
constexpr float kCivicTireStiffnessFront = 192150.0f;
constexpr float kCivicTireStiffnessRear = 202500.0f;
constexpr float kGravity = 9.81f;

// openpilot KP_INTERP. 마지막 점만 파라미터라 interp를 펼쳐 쓴다.
float scheduled_kp(float speed_mps, float kp_top) {
    constexpr int n = static_cast<int>(sizeof(kKpSpeedsMps) / sizeof(kKpSpeedsMps[0]));
    if (speed_mps <= kKpSpeedsMps[0]) return kKpValues[0];
    if (speed_mps >= kKpSpeedsMps[n - 1]) return kp_top;
    for (int i = 1; i < n; ++i) {
        if (speed_mps <= kKpSpeedsMps[i]) {
            const float hi_y = i == n - 1 ? kp_top : kKpValues[i];
            const float p = (speed_mps - kKpSpeedsMps[i - 1]) / (kKpSpeedsMps[i] - kKpSpeedsMps[i - 1]);
            return kKpValues[i - 1] + p * (hi_y - kKpValues[i - 1]);
        }
    }
    return kp_top;
}

float apply_deadzone(float error, float deadzone) {
    if (error > deadzone) return error - deadzone;
    if (error < -deadzone) return error + deadzone;
    return 0.0f;
}

// opendbc scale_tire_stiffness_per_mass
std::pair<float, float> scale_tire_stiffness_per_mass(float wheelbase, float center_to_front,
                                                      float tire_stiffness_factor) {
    const float center_to_rear = wheelbase - center_to_front;
    float front = kCivicTireStiffnessFront * tire_stiffness_factor / kCivicMass;
    front *= (center_to_rear / wheelbase) / (kCivicCenterToRear / kCivicWheelbase);
    float rear = kCivicTireStiffnessRear * tire_stiffness_factor / kCivicMass;
    rear *= (center_to_front / wheelbase) / (kCivicCenterToFront / kCivicWheelbase);
    return {front, rear};
}

}  // namespace

// openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다.
int TorqueController::update(const Input &in, const SteeringParams &params, const NnFeedforward &nn) {
    const float speed_mps = in.speed_mps;
    const LiveLateralParams &live = in.live;
    nn_feedforward_used_ = false;
    const float actual_curvature =
        estimate_actual_curvature(speed_mps, in.steering_angle_deg, params, in.yaw_rate_rad_s, in.yaw_rate_valid, live);

    const float speed_sq = speed_mps * speed_mps;
    const float desired_lat_accel = in.desired_curvature * speed_sq;
    const float actual_lat_accel = actual_curvature * speed_sq;
    if (std::isfinite(desired_lat_accel)) {
        const float abs_lat_accel = std::fabs(desired_lat_accel);
        abs_lat_accel_rise_ +=
            kNnCurveRiseFilterAlpha * ((abs_lat_accel - abs_lat_accel_prev_) / kDtCtrlF - abs_lat_accel_rise_);
        abs_lat_accel_prev_ = abs_lat_accel;
    }

    // 버퍼/저크 필터는 active와 무관하게 매 프레임 갱신한다. inactive에서
    // 얼리면 다시 결합한 뒤 첫 delay 구간에 낡은 요청과 비교되어 토크가 튄다.
    request_head_ = (request_head_ + 1) % kRequestBufferLen;
    lat_accel_request_[request_head_] = std::isfinite(desired_lat_accel) ? desired_lat_accel : 0.0f;
    // 상류: delay_frames = int(clip(lat_delay / dt + 1, 1, buffer_len))
    const int delay_frames = clamp_int(static_cast<int>(in.lat_delay_s / kDtCtrlF) + 1, 1, kRequestBufferLen);
    const auto at = [this](int back) { return (request_head_ - back + 2 * kRequestBufferLen) % kRequestBufferLen; };
    const float expected_lat_accel = lat_accel_request_[at(delay_frames - 1)];

    const int lookahead_back = clamp_int(delay_frames - 1 - kJerkLookaheadFrames, 1, kRequestBufferLen - 2);
    const float raw_jerk =
        (lat_accel_request_[at(lookahead_back - 1)] - lat_accel_request_[at(lookahead_back + 1)]) / (2.0f * kDtCtrlF);
    jerk_filtered_ += kJerkFilterAlpha * (raw_jerk - jerk_filtered_);

    // 오차는 횡가속도만으로 낸다. 저속 보강은 LOW_SPEED_Y 곡률항이 아니라
    // 속도별 비례 이득이 맡는다(openpilot PR #36364).
    const float setpoint = expected_lat_accel;
    const float measurement = actual_lat_accel;
    const float error = setpoint - measurement;
    actual_curvature_ = actual_curvature;
    error_ = error;

    if (!in.active) {  // 비활성(컨트롤러를 끈 것 포함)
        // 상류(#24606)는 적분기를 지우지 않지만 여기서는 지운다. 2026-09-25~10-06 녹화 18건의 폐루프 재생에서
        // 남긴 적분(재결합 때 0.2~0.33 m/s²)이 정차 뒤 출발에서 차선 중심 오차를 키웠다(10-04 0.079 → 0.120 m,
        // 10-06 0.069 → 0.090 m). 지우면 예전과 같다.
        p_ = i_ = f_ = 0.0f;
        normalized_output_ = 0.0f;
        feedforward_ = 0.0f;
        friction_torque_ = 0.0f;
        return 0;
    }

    // 상류처럼 deadzone 조향각을 차량 모델 곡률(학습 SR·강성)로 바꿔 횡가속으로 쓴다. 저속에만 건다(속도가 NaN이면
    // interp가 0을 준다).
    const float deadzone_deg = params.steering_angle_deadzone_deg *
                               interp(speed_mps, {kDeadzoneFullSpeedMps, kDeadzoneOffSpeedMps}, {1.0f, 0.0f});
    const float curvature_deadzone =
        deadzone_deg > 0.0f ? std::fabs(vehicle_model_curvature(deg_to_rad(deadzone_deg), speed_mps,
                                                                live.use_vehicle ? live_vehicle_params_ : params))
                            : 0.0f;
    const float lat_accel_deadzone = curvature_deadzone * speed_sq;

    const TorqueTuning tuning = torque_tuning(params, live);
    float feedforward = desired_lat_accel;
    // 상수 편향(offset)은 FF에서 뺀다. bank = -g*sin(도로기울기)이므로
    // 중력의 횡가속 기여(-bank)를 빼려면 bank를 더한다 (2026-08-30 부호 수정)
    feedforward -= tuning.lat_accel_offset;
    // 학습 롤을 쓰면 상류처럼 roll·g를 빼고, 같은 몫인 편향 추정은 쓰지 않는다
    if (live.use_vehicle)
        feedforward -= live.roll_rad * kGravity;
    else if (params.live_bank_compensation)
        feedforward += in.road_bank_lat_accel;
    // opendbc get_friction: 토크 공간 마찰 계수를 latAccelFactor로 횡가속 공간에 옮긴다
    const float friction = tuning.friction * tuning.lat_accel_factor;
    const float friction_term = interp(apply_deadzone(error + kJerkGain * jerk_filtered_, lat_accel_deadzone),
                                       {-kFrictionThreshold, kFrictionThreshold}, {-friction, friction});
    feedforward += friction_term;
    friction_torque_ = friction_term / tuning.lat_accel_factor;
    const float w = clamp_float(nn.weight, 0.0f, 1.0f);
    if (std::isfinite(nn.torque) && w > 0.0f) {
        // 학습한 토크 모델: 목표 궤적에 드는 토크를 직접 낸다. 편향(직진 토크), 롤·편경사(입력)와 동특성(학습한 궤적
        // 창)이 그 안에 있어 위 선형 피드포워드를 대신한다. PID와 같은 횡가속 공간으로 옮겨 섞는다.
        // 마찰 보상은 곡선에서 남긴다: 모델은 추종 오차를 모르므로 차가 늦을 때 붙는 오차 방향 토크를 내지 못한다
        // (2026-10-16 주행에서 곡선 진입 횡가속이 요구의 92~93 %, 선형은 100~109 %). 직선에서는 모델만 쓴다(오차
        // 잡음에 토크가 흔들려 모델의 매끄러움을 잃는다). sunnypilot NNLC의 friction_override와 같은 항이다.
        // 보낸 토크를 그대로 배운 모델은 그 마찰까지 배웠으므로 더하지 않는다(nn.curve_friction).
        // 곡선 이득: 모델은 계획 궤적을 그대로 따르는 토크를 내 곡선 진입에서 늦게 덜 돈다. 선형 피드포워드는 같은
        // 곳에서 10~20 % 더 내고 PID가 깎는다. 모델 파일의 몫(nn.curve_gain)만큼 |목표 횡가속|이 커지는 동안의 모델
        // 출력을 키운다(kNnCurveGainFullRise).
        const float ramp =
            interp(std::fabs(desired_lat_accel), {kNnCurveStartLatAccel, kNnCurveFullLatAccel}, {0.0f, 1.0f});
        const float curve = nn.curve_friction ? ramp : 0.0f;
        const float rising = interp(abs_lat_accel_rise_, {0.0f, kNnCurveGainFullRise}, {0.0f, 1.0f});
        const float gain =
            1.0f + nn.curve_gain * rising * ramp *
                       interp(speed_mps, {kNnCurveGainStartSpeedMps, kNnCurveGainFullSpeedMps}, {0.0f, 1.0f});
        feedforward += w * (gain * nn.torque * tuning.lat_accel_factor + curve * friction_term - feedforward);
        nn_feedforward_used_ = true;
    }

    // 저속 직선에서만 비례 이득을 torque_straight_kp_scale배로 낮춘다. 회전 진입은 지금 요청이, 회전 탈출은 지연
    // 비교 대상이 곡률을 들고 있어 회전 내내 원래 이득이다.
    const float request_curvature =
        std::max(std::fabs(in.desired_curvature), std::fabs(expected_lat_accel) / std::max(speed_sq, 1.0f));
    const float straight = interp(request_curvature, {kStraightCurvature, kTurnCurvature}, {1.0f, 0.0f}) *
                           interp(speed_mps, {kStraightKpFullSpeedMps, kStraightKpOffSpeedMps}, {1.0f, 0.0f});
    const float kp =
        scheduled_kp(speed_mps, params.torque_kp) * (1.0f - (1.0f - params.torque_straight_kp_scale) * straight);

    const bool freeze_integrator = in.steer_limited_by_safety || in.steering_pressed || speed_mps < 5.0f;
    // 상류 controlsd처럼 유한하지 않은 값은 0 토크로 바꾼다. clamp_float는 NaN을 하한으로 돌려주므로 그대로 두면
    // 한쪽 최대 토크(kHyundaiSteerMax)가 나간다. NaN이 적분기에 남지 않게 PID 상태도 지운다.
    if (!std::isfinite(error) || !std::isfinite(feedforward) || !std::isfinite(kp) ||
        !std::isfinite(tuning.lat_accel_factor) || tuning.lat_accel_factor <= 0.0f) {
        ++nonfinite_ticks_;
        p_ = i_ = f_ = 0.0f;
        normalized_output_ = 0.0f;
        feedforward_ = 0.0f;
        friction_torque_ = 0.0f;
        return 0;
    }
    const float output_lat_accel =
        pid_update(error, feedforward, freeze_integrator, kp, params.torque_ki, tuning.lat_accel_factor);
    // opendbc torque_from_lateral_accel_linear
    const float output_torque = output_lat_accel / tuning.lat_accel_factor;

    normalized_output_ = clamp_float(static_cast<float>(kTorqueOutputSign) * output_torque, -1.0f, 1.0f);
    feedforward_ = feedforward;
    // 상류 carcontroller int(round(torque * STEER_MAX)): 파이썬 round는 반올림 짝수다
    return static_cast<int>(std::nearbyint(normalized_output_ * static_cast<float>(kHyundaiSteerMax)));
}

// 상류 VM.update_params(max(x, 0.1), max(sr, 0.1)). 강성 배율은 타이어 계수에 곱해진다.
const SteeringParams &TorqueController::vehicle_model_params(const SteeringParams &params,
                                                             const LiveLateralParams &live, float *angle_offset_deg) {
    *angle_offset_deg = live.use_vehicle ? live.angle_offset_deg : params.angle_offset_deg;
    if (!live.use_vehicle) return params;
    live_vehicle_params_ = params;
    live_vehicle_params_.steer_ratio = std::max(live.steer_ratio, 0.1f);
    live_vehicle_params_.tire_stiffness_factor = params.tire_stiffness_factor * std::max(live.stiffness_factor, 0.1f);
    return live_vehicle_params_;
}

// 조향각이 만드는 곡률(차량 모델, 제어 부호).
float TorqueController::curvature_at_angle(float speed_mps, float steering_angle_deg, const SteeringParams &params,
                                           const LiveLateralParams &live) {
    float angle_offset_deg = 0.0f;
    const SteeringParams &vm = vehicle_model_params(params, live, &angle_offset_deg);
    // vm 경로의 부호 반전은 openpilot latcontrol_torque와 동일하다.
    float curvature = vehicle_model_curvature(deg_to_rad(steering_angle_deg - angle_offset_deg), speed_mps, vm);
    if (live.use_vehicle) curvature += roll_compensation(live.roll_rad, speed_mps);
    return -curvature;
}

// curvature_at_angle의 곡률 = -(1 rad당 곡률 · (각도 - 영점) + 롤 보정)을 각도로 푼다. 롤 보정은 slip을 갱신한 뒤에 쓴다.
TorqueController::AngleMap TorqueController::angle_map(float speed_mps, const SteeringParams &params,
                                                       const LiveLateralParams &live) {
    float angle_offset_deg = 0.0f;
    const SteeringParams &vm = vehicle_model_params(params, live, &angle_offset_deg);
    const float per_rad = vehicle_model_curvature(1.0f, speed_mps, vm);
    const float roll = live.use_vehicle ? roll_compensation(live.roll_rad, speed_mps) : 0.0f;
    AngleMap map;
    map.gain = rad_to_deg(-1.0f / per_rad);
    map.offset = rad_to_deg(-roll / per_rad) + angle_offset_deg;
    return map;
}

// 현재 조향각·속도에서 차량 모델로 실제 곡률을 낸다(상류 controlsd self.curvature, 현대·기아와 같음). 상류처럼
// 속도와 무관하게 계산하므로 정차 중에도 핸들 각도의 곡률이 나온다. ESP12 요레이트 곡률은 주행 로그(curveYaw)에
// 남기려고 함께 계산한다.
float TorqueController::estimate_actual_curvature(float speed_mps, float steering_angle_deg,
                                                  const SteeringParams &params, float yaw_rate_rad_s,
                                                  bool yaw_rate_valid, const LiveLateralParams &live) {
    actual_curvature_vm_ = 0.0f;
    actual_curvature_yaw_ = 0.0f;
    if (!std::isfinite(speed_mps)) return 0.0f;  // 바퀴 속도가 끊겼다(조향각·요레이트는 정수 CAN 값이다)
    const float actual_curvature_vm = curvature_at_angle(speed_mps, steering_angle_deg, params, live);
    actual_curvature_vm_ = actual_curvature_vm;
    float actual_curvature_yaw = actual_curvature_vm;
    if (yaw_rate_valid) {
        // K7 ESP12 YAW_RATE는 제어 관례와 부호가 반대라 반전한다. 2026-08-15
        // 실차 수동 주행의 커브 53샘플에서 curveVm/curveYaw 부호가 96% 반대,
        // 크기 비율 평균 1.15로 확인했다.
        actual_curvature_yaw = -yaw_rate_rad_s / std::max(speed_mps, 1e-3f);
    }
    actual_curvature_yaw_ = actual_curvature_yaw;
    return actual_curvature_vm;
}

TorqueController::TorqueTuning TorqueController::torque_tuning(const SteeringParams &params,
                                                               const LiveLateralParams &live) {
    const bool learned = live.use_torque;  // torqued가 사전값 ±30 %로 묶고 NaN이면 사전값으로 되돌린다
    TorqueTuning tuning;
    tuning.lat_accel_factor = learned ? live.lat_accel_factor : params.torque_lat_accel_factor;
    tuning.friction = learned ? live.friction : params.torque_friction;
    tuning.lat_accel_offset = learned ? live.lat_accel_offset : params.torque_lat_accel_offset;
    return tuning;
}

// 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
void TorqueController::update_vehicle_model(const SteeringParams &params) {
    const float center_to_front = params.center_to_front_m();
    // 학습 SR·강성은 매 틱 조금씩 바뀌므로 정확히 같을 때만 건너뛴다(상류는 매번 계산)
    if (last_wheelbase_m_ == params.wheelbase_m && last_center_to_front_m_ == center_to_front &&
        last_tire_stiffness_factor_ == params.tire_stiffness_factor && last_steer_ratio_ == params.steer_ratio) {
        return;
    }
    const float center_to_rear = params.wheelbase_m - center_to_front;
    const auto stiffness =
        scale_tire_stiffness_per_mass(params.wheelbase_m, center_to_front, params.tire_stiffness_factor);
    // 앞뒤 강성이 양수라 slip factor는 늘 음수다(아래 분모들이 0에 가까워지지 않는다). 상류 calc_slip_factor(단위 질량)
    slip_factor_ = (stiffness.first * center_to_front - stiffness.second * center_to_rear) /
                   (params.wheelbase_m * params.wheelbase_m * stiffness.first * stiffness.second);
    last_wheelbase_m_ = params.wheelbase_m;
    last_center_to_front_m_ = center_to_front;
    last_tire_stiffness_factor_ = params.tire_stiffness_factor;
    last_steer_ratio_ = params.steer_ratio;
}

// 조향각과 속도에서 실제 curvature를 계산한다.
float TorqueController::vehicle_model_curvature(float steering_angle_rad, float speed_mps,
                                                const SteeringParams &params) {
    update_vehicle_model(params);
    const float denom = 1.0f - slip_factor_ * speed_mps * speed_mps;
    // 상류 curvature_factor의 (1 - steerRatioRear)는 1이다: 후륜 조향이 있는 차를 다루지 않는다(상류도 모두 0)
    const float curvature_factor = 1.0f / denom / params.wheelbase_m;
    return curvature_factor * steering_angle_rad / params.steer_ratio;
}

float TorqueController::roll_compensation(float roll_rad, float speed_mps) const {
    return kGravity * roll_rad / ((1.0f / slip_factor_) - speed_mps * speed_mps);
}

// PID 한 스텝을 계산한다.
float TorqueController::pid_update(float error, float feedforward, bool freeze_integrator, float kp, float ki,
                                   float limit) {
    p_ = kp * error;
    f_ = feedforward;
    if (!freeze_integrator) {
        const float i = i_ + ki * kDtCtrlF * error;
        // 이미 한계에 걸려 있으면 그 방향으로는 더 감지 않는다(상류 "Don't allow windup if already clipping")
        const float test_control = p_ + i + f_;
        const float i_upper = test_control > limit ? i_ : limit;
        const float i_lower = test_control < -limit ? i_ : -limit;
        i_ = std::min(std::max(i, i_lower), i_upper);  // np.clip
    }
    return clamp_float(p_ + i_ + f_, -limit, limit);
}
