#pragma once

// controlsd가 읽는 조향 런타임 파라미터. params/steering.json이 출처다. 다만 상류(opendbc
// CarControllerParams, 모듈 상수)에서도 코드에 박힌 차량·통신 상수는 여기 필드로만 두고 파일에서 읽지
// 않는다(필드마다 적었다). 테스트와 재생 도구는 이 필드를 바꿔 쓴다. 조향 토크 한계는 상류처럼 상수다
// (car/hyundai_can.h의 kHyundaiSteer*).

#include "common/utils_file.h"
#include "controls/adaptive_cruise.h"

#include <cstdint>
#include <string>

// 기본값은 params/steering.json(K7 YG HEV 실차 검증값)과 일치시킨다. 로드 실패는
// controlsd가 throw하므로 이 값은 파일 폴백이 아니라, JSON에 키가 빠졌을 때와
// 호스트 검사·재생 도구가 실제로 쓰는 값이다.
struct SteeringParams {
    bool enabled = true;

    // 저속에서도 MDPS가 LKAS 토크를 받도록 MDPS 버스 CLU11에 쓰는 속도. 상류에는 없는 기능이고, 커뮤니티
    // 포크(openpilot_c2)도 60 km/h(38 mph)를 코드에 박아 둔다. 파일에서 읽지 않는다.
    float mdps_speed_spoof_kph = 60.0f;
    int steering_pressed_threshold = 150;

    // openpilot 토크 튜닝 그대로(횡가속도 공간). 토크 = (FF + P + I) / 배율 + 마찰.
    // 배율은 MaixCAM2 torqued 학습값(2026-09-27, 6769점 TLS 2.75).
    float torque_lat_accel_factor = 2.75f;
    // KP_INTERP의 30 m/s 끝점. 나머지 점은 상류 고정값이다.
    float torque_kp = 0.8f;
    float torque_ki = 0.15f;
    float torque_friction = 0.083f;  // 토크 공간, torqued 학습값(2026-09-27)
    // 상류 steeringAngleDeadzoneDeg: 오차가 이 조향각만큼의 횡가속 안이면 마찰 보상을 넣지 않는다. 상류는 일부
    // Toyota만 0.2도를 쓰고, openpilot_c2(TorqueAngDeadZone)는 1도였다. 상류와 달리 40 km/h 위에서 줄여
    // 60 km/h에서 0으로 만든다(lateral_torque.cc).
    float steering_angle_deadzone_deg = 1.0f;
    // 저속 직선의 비례 이득 배율. 36 km/h 아래 직선(요청 곡률 0.003 미만)에서 KP_INTERP에 곱하고, 54 km/h와
    // 곡률 0.008 쪽으로 가며 1로 돌린다(lateral_torque.cc). 1이면 상류 그대로다.
    float torque_straight_kp_scale = 0.7f;

    // paramsd 학습값(2026-09-27): 조향비 14.72, 타이어 강성 0.83.
    float steer_ratio = 14.72f;
    float tire_stiffness_factor = 0.83f;
    float steer_actuator_delay = 0.42f;
    bool avoid_lkas_fault_enabled = true;
    // MDPS 고장 한계: 85도 위에 0.98~1.12초 머물면 고장(K7 실측). 상류 hyundai carcontroller의
    // MAX_ANGLE·MAX_ANGLE_FRAMES와 같은 값이고, 상류처럼 차량 상수라 파일에서 읽지 않는다.
    float avoid_lkas_fault_max_angle_deg = 85.0f;
    int avoid_lkas_fault_max_frames = 89;
    // 운전자가 핸들을 잡지 않았을 때 컨트롤러가 스스로 가는 최대 핸들 각도. 고장 각도(85도) 아래에서
    // 멈춰 토크를 끊김 없이 유지한다. 0이면 끄고, 파일 값은 84도까지로 자른다.
    float avoid_lkas_fault_hold_angle_deg = 80.0f;
    // avoid_lkas_fault_enabled를 끈 경우에만: 85도 위에서 요청을 max_frames 낸 뒤 끊는 프레임 수. 상류
    // MAX_ANGLE_CONSECUTIVE_FRAMES와 같은 값이고 파일에서 읽지 않는다. 켠 경우 85도 위에서는 짧게 끊지 않고
    // 85도 아래로 올 때까지 끈다. K7에서 2프레임 컷 뒤 다시 켜면 3~14 ms 안에 fault가 났다(2026-10-03,
    // 4번 중 4번).
    int avoid_lkas_fault_cut_frames = 2;
    float angle_offset_deg = -1.57f;  // paramsd 학습 평균(2026-09-27)
    // openpilot latAccelOffset(m/s^2). 상수 횡가속 편향을 FF에서 뺀다.
    // +y=오른쪽 관례라 양수 = 우측 쏠림 보정. torqued 학습값(녹화가 있으면 replay_lateral_learners 값)을 넣는다.
    float torque_lat_accel_offset = -0.06f;  // torqued 학습값(2026-09-27)
    // ESP12 실측으로 직선에서 추정한 도로 편경사(뱅크)를 FF에서 보정한다. 커브에서는 직전 값을
    // 유지하고 30초 넘게 갱신이 없으면 0으로 줄인다(controls/road_bank.h의 RoadBankEstimator).
    bool live_bank_compensation = true;
    // paramsd·torqued 학습값 사용. 끄면 학습기는 계산·기록만 한다. 차량 값을 켜면
    // 롤 보정이 live_bank_compensation을 대신한다.
    bool use_live_vehicle_params = true;
    bool use_live_torque_params = true;
    // lagd(locationd) 조향 지연 사용. 켜고 lagd가 확정(5블록)되면 경로 지연(목표 곡률을 읽는
    // 시점), 토크 컨트롤러의 요청 지연, torqued 지연에 추정값을 쓴다(상류 lat_delay).
    bool use_live_delay = false;
    // 학습한 토크 모델로 피드포워드를 낸다(docs/lateral-nn.md). 웹 콘솔에서 고른 모델 파일(파라미터 디렉터리의
    // lateral_nn.json이 가리키는 models/lateral_nn/<차종>_v<버전>.json, common/lateral_nn_path.h)이 있어야 하고, 없으면
    // 켜도 선형 피드포워드를 쓴다. PID는 그대로다.
    bool nn_feedforward = false;
    // paramsd·torqued 입력을 상류처럼 locationd 요레이트·롤로 받는다. 끄면 ESP12 요레이트(자체 바이어스
    // 추정)와 ESP12 횡가속 롤. locationd가 없거나 낡으면 ESP12로 돌아간다.
    bool use_locationd_learner_inputs = true;
    float wheelbase_m = 2.855f;
    float center_to_front_ratio = 0.4f;
    float path_offset_m = 0.0f;
    // Lane 모드 MPC의 경로(횡위치) 가중치. 상류 0.9.4 PATH_COST는 1이다. 2026-10-05 폐루프 재생에서
    // 3이면 고속도로 오른쪽 커브의 안쪽 치우침이 13.4 -> 10.6 cm로 줄고 횡저크는 12~19% 는다.
    // laneless 모드에는 쓰지 않는다.
    float lane_path_weight = 3.0f;
    float min_steer_speed_mps = 1.0f;
    // 경로 모드. false면 Lane 모드로 차선 확률이 높으면 차선 중심 경로를 섞고 낮으면 모델 경로만 쓴다. true면
    // Laneless 모드로 openpilot 메인처럼 모델 경로만 쓴다(get_curvature_from_plan).
    bool laneless_mode = false;
    // 이 속도 미만에서는 깜빡이를 켜도 차선 변경을 시작하지 않는다(상류 LANE_CHANGE_SPEED_MIN은 20 mph).
    float lane_change_min_speed_kph = 30.0f;

    float center_to_front_m() const;
};

// 제어 타이밍(파일에서 읽지 않는 고정값). 낡음 판정은 상류도 고정 규칙이다(메시지 주기의 10배, CAN 버스는 0.5초).
// 모델 경로는 그보다 엄격하게 250 ms, 차량 CAN 상태는 버스와 같은 500 ms를 쓴다.
constexpr double kModelTimeoutS = 0.25;
constexpr uint64_t kModelTimeoutNs = static_cast<uint64_t>(kModelTimeoutS * 1e9);
constexpr double kVehicleStateTimeoutS = 0.5;
// 해제 뒤 순정 LKAS로 넘기기 전에 0 토크 프레임을 이어 보내는 시간(openpilot_c2의 기본 인계 시간과 같다). 상류는
// LKAS11을 늘 보내므로 대응 값이 없다.
constexpr double kInactiveReleaseS = 3.0;

// params/steering.json을 읽어 SteeringParams에 반영한다.
bool load_steering_params(const std::string &path, SteeringParams *params, std::string *error);
// steering.json에서 읽는 키 가운데 a와 b가 다른 첫 키. 같으면 빈 문자열(기본값이 파일과 같은지 보는 테스트용).
std::string first_steering_param_difference(const SteeringParams &a, const SteeringParams &b);

// controlsd의 런타임 파라미터 두 파일(steering/adaptive_cruise.json). 한꺼번에 읽고 한꺼번에 바꾼다.
struct ControlParams {
    SteeringParams steering;
    AdaptiveCruiseParams cruise;
};

struct ControlParamPaths {
    std::string steering;
    std::string cruise;
};

// 둘 다 읽어야 true. 하나라도 거부되면 params는 그대로이고 error에 사유를 쓴다.
bool load_control_params(const ControlParamPaths &paths, ControlParams *params, std::string *error);
// 시작·재적용 로그의 꼬리: MDPS 속도 바꿔치기와 비전 크루즈 설정.
std::string control_params_summary(const ControlParams &params);

// 두 파일을 1초마다 stat으로 보고, 바뀌었으면 둘을 다시 읽는다(웹 콘솔이 쓴 것도 손으로 고친 것도). 하나라도
// 거부되면 둘 다 이전 값을 유지한다.
class ControlParamsWatcher {
public:
    // now_ns(monotonic_now_ns)에 시작 값을 이미 읽었다고 보고 첫 확인을 1초 뒤로 둔다.
    ControlParamsWatcher(const ControlParamPaths &paths, uint64_t now_ns);

    // 새로 적용했으면 true와 그 값. 바뀐 게 없거나 거부됐으면(로그를 남긴다) false이고 params는 그대로다.
    bool poll(uint64_t now_ns, ControlParams *params);
    unsigned generation() const { return generation_; }

private:
    ControlParamPaths paths_;
    FileWatch steering_;
    FileWatch cruise_;
    unsigned generation_ = 1;
};
