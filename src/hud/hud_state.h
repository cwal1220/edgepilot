#ifndef HUD_STATE_H
#define HUD_STATE_H

/* 공유 상태 스냅샷(ModelState·ControlState 등) → HUD 표시 상태. 그리기와 무관해 OpenCV 없이
 * 컴파일되며, overlayd와 hud_snapshot이 같은 매핑을 쓴다. */

#include "controls/departure_alert.h"
#include "common/ipc_messages.h"
#include "common/projection.h"

#include <limits>
#include <string>

/* engage 차단 사유 → HUD 라벨. HUD 상태줄과 engage 거부 토스트가 같은 표를
 * 쓰도록 여기 한 벌만 둔다. 모르는 사유면 nullptr을 돌려주고, 표시 방식은
 * 호출부가 정한다. */
const char *engage_block_label(const char *block);

/* 깜빡이 애니메이션은 켜진 순간을 0으로 하는 단계 수로 그린다. 단계 진행은
 * overlayd가 시각 기준으로 계산하므로(TurnSignalClock) 재그리기 빈도에 영향받지 않는다. */
constexpr int kTurnSignalSteps = 25;

struct HudState {
    bool panda_connected = false;
    bool panda_healthy = false;
    bool controller_enabled = false;
    bool controller_engaged = false;
    bool controller_active = false;
    bool laneless_mode = false;
    bool vehicle_fresh = false;
    bool steering_fault = false;
    bool left_blinker = false;
    bool right_blinker = false;
    int turn_signal_step = 0;
    bool cruise_active = false;
    bool brake_hold = false;
    bool soft_disabling = false;   // 3초 뒤 해제 예고(active_block이 사유)
    bool steer_saturated = false;  // 커브가 조향 한계를 넘음
    int lane_change = 0;           // 0 없음, 1 운전자가 핸들을 밀기를 기다림, 2 변경 중
    int lane_change_direction = 0; // -1 왼쪽, 1 오른쪽
    bool steer_paused = false;     // 85도 위에서 조향 요청을 끄고 쉰다
    bool steer_paused_by_driver = false;  // 넘겨받은 회전: 15도 아래에서 손을 떼야 다시 조향
    bool services_healthy = false;
    bool network_connected = false;  // 와이파이가 AP에 붙고 주소가 있다
    unsigned panda_faults = 0;
    int gear = 0;
    int wifi_signal_dbm = 0;
    float cluster_speed_kph = 0.0f;
    float ego_speed_kph = 0.0f;
    float cruise_max_speed_kph = 0.0f;
    float cruise_command_speed_kph = 0.0f;
    bool radar_lead_valid = false;
    float radar_lead_distance_m = 0.0f;
    float radar_lead_relative_speed_mps = 0.0f;
    DepartureAlertType departure_alert_type = DepartureAlertType::none;
    bool green_light_alert_armed = false;
    char engage_reject_label[48] = {};  // 비어 있지 않으면 engage 거부 알림(사유 라벨)
    bool tpms_valid = false;
    int tpms_unit = 0;
    float tpms_pressure_fl = 0.0f;
    float tpms_pressure_fr = 0.0f;
    float tpms_pressure_rl = 0.0f;
    float tpms_pressure_rr = 0.0f;
    bool tpms_warning = false;
    float steering_angle_deg = 0.0f;
    float normalized_output = 0.0f;
    int desired_torque = 0;
    int apply_torque = 0;
    // 보낸 토크 / 최대 토크(-1..1, + = 왼쪽). 토크 바가 그린다.
    float steer_torque_fraction = 0.0f;
    int driver_torque = 0;
    float driver_torque_fraction = 0.0f;  // 운전자 토크를 같은 눈금으로(토크 바의 눈금)
    float cpu_percent = 0.0f;
    float memory_percent = 0.0f;
    float storage_percent = 0.0f;
    float cpu_temp_c = 0.0f;
    float preview_fps = 0.0f;
    float model_fps = 0.0f;
    float overlay_fps = 0.0f;
    bool calibration_available = false;
    unsigned calibration_status = 0;
    int calibration_valid_blocks = 0;
    float calibration_roll_deg = 0.0f;
    float calibration_pitch_deg = 0.0f;
    float calibration_yaw_deg = 0.0f;
    char active_block[32] = {};
    char network_interface[16] = {};
    char network_ipv4[16] = {};
    char network_ssid[33] = {};
    char wired_interface[16] = {};  // 와이파이 밖의 링크(usb0 같은 USB 가상 이더넷). 연결 표시와 무관
    char wired_ipv4[16] = {};
    bool network_card = false;   // 상태 알약을 눌러 연 네트워크 카드
    bool recording = false;      // recordd가 route를 쓰는 중
    bool storage_full = false;   // 저장 공간이 모자라 녹화를 거부했거나 멈춤
    bool debug_card = false;     // 웹 기기 설정의 HUD 진단: 수치 카드를 띄운다
    // 학습값(진단 카드). learner_fresh가 아니면 "--".
    bool learner_fresh = false;
    bool params_valid = false;     // paramsd: SR·강성·평균 오프셋이 다 유효
    float steer_ratio = 0.0f;
    float stiffness = 0.0f;
    float angle_offset_deg = 0.0f;
    bool torque_valid = false;     // torqued
    float torque_factor = 0.0f;    // 학습 중인 값(raw)
    float torque_friction = 0.0f;
    float torque_offset = 0.0f;
    int torque_cal_percent = 0;
    float lateral_delay_s = 0.0f;  // 경로에 쓰는 조향 지연
    // 학습 카드: 제어가 쓰는 형태의 학습값과 지금 제어가 그 값을 쓰는지. 쓰지 않으면 제어는
    // 파라미터(사전값)를 쓴다.
    bool vehicle_learned = false;  // paramsd SR·강성·영점을 쓰는 중
    bool torque_learned = false;   // torqued 필터값을 쓰는 중
    bool delay_learned = false;    // 경로 지연에 lagd 추정을 쓰는 중(lateral_delay_s가 그 값)
    float angle_offset_fast_deg = 0.0f;  // 제어가 빼는 영점(평균 + 빠른 성분)
    float torque_factor_filtered = 0.0f; // 제어가 쓰는 횡가속 계수(필터값)
    int lag_blocks = -1;           // lagd 유효 블록(-1 = locationd 상태 없음)
    float lag_estimate_s = 0.0f;
    // 모델이 본 가까운 차선 중앙(+ = 오른쪽 = 차가 왼쪽에 있다). 모르면 NaN.
    float lane_center_offset_m = std::numeric_limits<float>::quiet_NaN();
};

/* 받은 ModelState를 렌더러가 그리는 모델 출력과 투영으로 되돌린다(overlayd·hud_snapshot). lead는
 * t=0 하나뿐이고 road_transform은 ModelState에 없다. */
ParsedModelOutput parsed_from_model_state(const ModelState &state);
ProjectionState projection_from_model_state(const ModelState &state);

/* 공유 상태 스냅샷 → HUD 표시 상태. fresh가 아니면 값을 0/false로 두어 HUD가 "--"를
 * 그린다. overlayd와 hud_snapshot이 같은 매핑을 쓴다. */
void hud_apply_panda_state(const PandaState &panda, bool fresh, HudState *hud);
void hud_apply_control_state(const ControlState &control, bool fresh, HudState *hud);
void hud_apply_model_state(const ModelState &model, bool fresh, HudState *hud);
void hud_apply_record_state(const RecordState &record, bool fresh, HudState *hud);
void hud_apply_learner_state(const LearnerState &learner, bool fresh, HudState *hud);
void hud_apply_localization_state(const LocalizationState &localization, bool fresh,
                                  HudState *hud);

/* 아래 가운데 알림 카드의 무게. 렌더러가 색으로 옮긴다(흰색, 초록, 주황, 빨강). */
enum class HudAlertLevel { notice, proceed, caution, critical };

struct HudAlertCard {
    std::string title;
    std::string detail;
    HudAlertLevel level = HudAlertLevel::notice;
    int arrow = 0;  // 제목 옆 방향 화살표: -1 왼쪽, 1 오른쪽

    bool empty() const { return title.empty(); }
};

/* 이 HUD 상태에서 띄울 알림 카드 하나. 우선순위는 engage 거부 > 해제 예고 > 조향 결함 > panda 결함 >
 * 조향 쉼(85도) > 조향 한계 > 서비스 대기 > 차선 변경 대기 > 출발 감지이고, 없으면 빈 카드다.
 * model_ok는 결함 카드의 연결 줄(MODEL/CAR/PANDA)에 쓴다. */
HudAlertCard hud_select_alert(const HudState &hud, bool model_ok);

/* 모델이 본 가까운 차선 중앙: x=0에서 왼쪽·오른쪽 자기 차선선의 가운데(+y = 오른쪽). 두 선이
 * 다 확실하지 않으면 NaN. tools/model/lane_bias.py의 offset(x=0)과 같은 값이다. */
float lane_center_offset_m(const ParsedModelOutput &output);
/* model_ok: 유효하고 신선한 모델 출력이 있는지. services_healthy의 조건 중 하나. */
void hud_apply_manager_state(const ManagerState &manager, bool fresh, bool model_ok,
                             HudState *hud);

#endif
