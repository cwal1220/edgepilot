#ifndef IPC_MESSAGES_H
#define IPC_MESSAGES_H

/* 프로세스 사이를 /dev/shm으로 건너가는 메시지 전부: 채널 매직·버전과 이름, 크기 상한, 채널
 * 헤더, 상태 스냅샷. 채널 구현은 ipc_channels.h에, ModelState를 채우고 되돌리는 변환은
 * model_state_fill.h와 hud_state.h에 있다. 메시지를 쓰기만 하는 코드는 이 헤더만 본다. */

#include "common/app_config.h"
#include "common/model_output.h"
#include "common/utils_time.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

// ---- 채널 매직·버전 ----
constexpr uint32_t kIpcMagic = 0x4b323349;
constexpr uint32_t kIpcVersion = 1;
constexpr uint32_t kFrameRingMagic = 0x4b465249;
constexpr uint32_t kFrameRingVersion = 5;
constexpr uint32_t kCanQueueMagic = 0x4b435151;
constexpr uint32_t kCanQueueVersion = 1;

// ---- 채널 이름 ----
constexpr char kRoadAiFrameRing[] = "/edgepilot_road_ai";
constexpr char kRoadAiFrameTopic[] = "/edgepilot_road_ai_frame";
constexpr char kRecordFrameTopic[] = "/edgepilot_record_frame";
constexpr char kModelStateTopic[] = "/edgepilot_model_state";
constexpr char kManagerStateTopic[] = "/edgepilot_manager_state";
constexpr char kCanTopic[] = "/edgepilot_can";
constexpr char kSendCanTopic[] = "/edgepilot_sendcan";
constexpr char kCanLogTopic[] = "/edgepilot_can_log";
constexpr char kSendCanLogTopic[] = "/edgepilot_sendcan_log";
constexpr char kPandaStateTopic[] = "/edgepilot_panda_state";
constexpr char kControlStateTopic[] = "/edgepilot_control_state";
constexpr char kLearnerStateTopic[] = "/edgepilot_learner_state";
constexpr char kImuTopic[] = "/edgepilot_imu";
constexpr char kLocalizationStateTopic[] = "/edgepilot_localization";
constexpr char kRecordStateTopic[] = "/edgepilot_record_state";

// ---- 크기 ----
constexpr unsigned kFrameSlots = 8;
constexpr unsigned kMaxProcesses = 12;
constexpr unsigned kAiWidth = kDefaultAiWidth;
constexpr unsigned kAiHeight = kDefaultAiHeight;
constexpr unsigned kAiFrameBytes = kAiWidth * kAiHeight * 3 / 2;
constexpr unsigned kCanBatchMaxFrames = 256;
constexpr unsigned kCanQueueSlots = 64;

struct IpcHeader {
    uint32_t magic = kIpcMagic;
    uint32_t version = kIpcVersion;
    uint32_t payload_capacity = 0;
    uint32_t reserved0 = 0;
    std::atomic<uint64_t> seq{0};
    std::atomic<uint64_t> timestamp_ns{0};
    std::atomic<uint32_t> payload_size{0};
    uint32_t reserved1 = 0;
};

static_assert(sizeof(IpcHeader) == 40,
              "IpcHeader layout is part of the Python manager ABI");

struct FrameRingHeader {
    uint32_t magic = kFrameRingMagic;
    uint32_t version = kFrameRingVersion;
    uint32_t slot_count = kFrameSlots;
    uint32_t width = kAiWidth;
    uint32_t height = kAiHeight;
    uint32_t frame_bytes = kAiFrameBytes;
    uint32_t reserved0 = 0;
    uint32_t reserved1 = 0;
    std::atomic<uint64_t> slot_seq[kFrameSlots]{};
    std::atomic<uint64_t> slot_frame_id[kFrameSlots]{};
    /* 슬롯 픽셀이 있는 CMM 블록의 물리 주소. 0이면 아직 붙지 않았다. */
    uint64_t slot_phys[kFrameSlots]{};
};

struct RoadAiFrame {
    uint64_t frame_id = 0;
    uint64_t timestamp_ns = 0;
    uint32_t slot = 0;
    uint32_t width = kAiWidth;
    uint32_t height = kAiHeight;
    uint32_t format = 0;
    uint32_t crop_x = 0;
    uint32_t crop_y = 0;
    uint32_t crop_width = 0;
    uint32_t crop_height = 0;
};

struct IpcPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct LeadState {
    uint32_t valid = 0;
    float probability = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
};

struct PoseState {
    uint32_t valid = 0;
    float trans[3] = {};
    float rot[3] = {};
    float trans_std[3] = {};
    float rot_std[3] = {};
};

struct CalibrationState {
    uint32_t status = 0;
    int32_t valid_blocks = 0;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float spread[3] = {};
};

struct ModelState {
    uint64_t frame_id = 0;
    uint64_t capture_timestamp_ns = 0;
    uint64_t model_timestamp_ns = 0;
    float model_execution_ms = 0.0f;
    uint32_t valid = 0;
    int32_t best_plan = 0;
    float plan_probability = 0.0f;
    float model_t[kTrajectorySize] = {};
    float lane_t[kTrajectorySize] = {};
    IpcPoint plan[kTrajectorySize] = {};
    IpcPoint lanes[4][kTrajectorySize] = {};
    float lane_probabilities[4] = {};
    float lane_stds[4] = {};
    IpcPoint road_edges[2][kTrajectorySize] = {};
    float road_edge_stds[2] = {};
    float desire_state[kDesireLen] = {};
    LeadState lead;
    PoseState pose;
    CalibrationState calibration;
    // 녹화 v6부터. plan의 yaw·yaw rate(laneless 모드가 openpilot 메인처럼 곡률을 만든다).
    float plan_yaw[kTrajectorySize] = {};
    float plan_yaw_rate[kTrajectorySize] = {};
    /* 녹화 v7부터. 이 프레임 워프에 쓴 카메라 장착(웹 기기 설정, modeld가 천천히 옮긴 값).
     * 모델 점은 이만큼 오른쪽 가상 카메라 기준이라 HUD가 같은 값으로 되돌려 그린다. */
    float camera_offset_m = 0.0f;
    float camera_height_m = 0.0f;
    /* 녹화 v8부터. 운전자가 0, 2, …, 10초 뒤 가속·브레이크 페달을 밟고 있을 확률(모델 meta). 정차
     * 출발 알림이 2초 가속 확률로 녹색 신호를 잡는다(경로보다 빨리, 경로가 안 열려도 오른다). */
    float gas_press_probs[kMetaPressHorizons] = {};
    float brake_press_probs[kMetaPressHorizons] = {};
};

/* 이 크기가 녹화 ModelState 레코드의 페이로드 크기다. 바뀌면 기존 녹화를
 * 읽는 tools/model/recording_reader.py와 어긋나므로 recording_format.h의
 * kRecordingVersion도 함께 올려야 한다. */
static_assert(sizeof(ModelState) == 3576 && offsetof(ModelState, plan_yaw) == 3256 &&
                  offsetof(ModelState, camera_offset_m) == 3520 &&
                  offsetof(ModelState, gas_press_probs) == 3528,
              "ModelState layout is shared with the recording reader");
// 웹 콘솔(web_console, MODEL_CALIBRATION_OFFSET)이 이 위치에서 보정 상태를 읽는다.
static_assert(offsetof(ModelState, calibration) == 3224 && sizeof(CalibrationState) == 32,
              "ModelState.calibration is read by the web console");
/* 웹 BEV 탭(scripts/web_console/static/bev_data.js)이 위치로 읽는 필드. 서버는 페이로드를 그대로 보내고
 * 웹 콘솔의 BEV_MODEL_FIELDS가 페이지에 위치를 알려 준다(check_web_console.py가 대조). */
#define EDGEPILOT_MODEL_STATE_AT(field, expected) \
    static_assert(offsetof(ModelState, field) == (expected), \
                  "ModelState." #field " moved: web BEV")
EDGEPILOT_MODEL_STATE_AT(model_timestamp_ns, 16);
EDGEPILOT_MODEL_STATE_AT(valid, 28);
EDGEPILOT_MODEL_STATE_AT(plan, 304);
EDGEPILOT_MODEL_STATE_AT(lanes, 700);
EDGEPILOT_MODEL_STATE_AT(lane_probabilities, 2284);
EDGEPILOT_MODEL_STATE_AT(road_edges, 2316);
EDGEPILOT_MODEL_STATE_AT(road_edge_stds, 3108);
EDGEPILOT_MODEL_STATE_AT(lead, 3148);
EDGEPILOT_MODEL_STATE_AT(gas_press_probs, 3528);
#undef EDGEPILOT_MODEL_STATE_AT
static_assert(sizeof(IpcPoint) == 12 && sizeof(LeadState) == 24 && offsetof(LeadState, probability) == 4 &&
                  offsetof(LeadState, x) == 8 && offsetof(LeadState, velocity) == 16 &&
                  offsetof(LeadState, acceleration) == 20,
              "web BEV reads IpcPoint and LeadState by position");

struct ProcessState {
    char name[16] = {};
    uint32_t running = 0;
};

static_assert(sizeof(ProcessState) == 20,
              "ProcessState layout is shared with the Python manager");

struct ManagerState {
    uint64_t timestamp_ns = 0;
    uint32_t process_count = 0;
    uint32_t reserved = 0;
    ProcessState processes[kMaxProcesses] = {};
};

static_assert(sizeof(ManagerState) == 256,
              "ManagerState layout is shared with the Python manager");

struct IpcCanFrame {
    uint32_t address = 0;
    uint32_t src = 0;
    uint32_t bus_time = 0;
    uint32_t data_len = 0;
    uint32_t flags = 0;
    uint8_t data[64] = {};
};

struct CanBatch {
    uint64_t timestamp_ns = 0;
    uint32_t valid = 0;
    uint32_t count = 0;
    uint32_t dropped = 0;
    uint32_t reserved = 0;
    IpcCanFrame frames[kCanBatchMaxFrames] = {};
};

inline bool can_batch_is_fresh(const CanBatch &batch, uint64_t now_ns,
                                    uint64_t max_age_ns)
{
    return batch.valid && timestamp_fresh_ns(batch.timestamp_ns, now_ns, max_age_ns);
}

/* 프레임 목록을 배치 하나로 담는다. 256개를 넘는 꼬리는 dropped로 센다.
 * fill(dst, src)이 프레임별 필드를 옮긴다. controlsd(CanFrame)와 pandad
 * (PandaCanFrame)가 같은 껍데기를 쓴다. */
template <class Frame, class Fill>
CanBatch make_can_batch(const std::vector<Frame> &frames, Fill fill)
{
    CanBatch batch;
    batch.timestamp_ns = monotonic_now_ns();
    batch.valid = 1;
    batch.count = static_cast<uint32_t>(
        std::min<size_t>(frames.size(), kCanBatchMaxFrames));
    batch.dropped = static_cast<uint32_t>(frames.size() - batch.count);
    for (uint32_t i = 0; i < batch.count; ++i) fill(&batch.frames[i], frames[i]);
    return batch;
}

struct CanQueueHeader {
    uint32_t magic = kCanQueueMagic;
    uint32_t version = kCanQueueVersion;
    uint32_t slot_count = kCanQueueSlots;
    uint32_t reserved0 = 0;
    std::atomic<uint64_t> write_seq{0};
    std::atomic<uint64_t> read_seq{0};
    uint64_t reserved1 = 0;
    uint64_t reserved2 = 0;
};

struct PandaState {
    uint64_t timestamp_ns = 0;
    uint32_t connected = 0;
    uint32_t comms_healthy = 0;
    uint32_t tx_enabled = 0;
    uint32_t controls_allowed = 0;
    uint32_t ignition_line = 0;
    uint32_t ignition_can = 0;
    uint32_t safety_mode = 0;
    uint32_t safety_param = 0;
    uint32_t panda_type = 0;
    uint32_t can_rx_errs = 0;
    uint32_t can_send_errs = 0;
    uint32_t can_fwd_errs = 0;
    uint32_t blocked_msg_cnt = 0;
    uint32_t heartbeat_lost = 0;
    uint32_t usb_tx_timeouts = 0;
    uint32_t usb_tx_retries = 0;
    uint32_t malformed_rx_batches = 0;
    uint32_t faults = 0;
    uint32_t fault_status = 0;
    uint32_t voltage = 0;
    uint32_t current = 0;
};

// ---- ControlState::hud_flags 비트 ----
constexpr uint32_t kHudFlagLaneless = 1U << 0;
constexpr uint32_t kHudFlagBrakeHold = 1U << 1;
constexpr uint32_t kHudFlagSoftDisabling = 1U << 2;   // 3초 뒤 해제 예고(active_block이 사유)
constexpr uint32_t kHudFlagSteerSaturated = 1U << 3;  // 커브가 조향 한계를 넘음
constexpr uint32_t kHudFlagLaneChangePending = 1U << 4;  // 차선 변경 대기: 운전자가 그쪽으로 핸들을 밀어야 시작
constexpr uint32_t kHudFlagLaneChanging = 1U << 5;       // 차선 변경 중(시작·마무리)
constexpr uint32_t kHudFlagLaneChangeRight = 1U << 6;    // 차선 변경 방향(없으면 왼쪽)
constexpr uint32_t kHudFlagSteerPaused = 1U << 7;        // 85도 위에서 조향 요청을 끄고 쉰다
constexpr uint32_t kHudFlagSteerPausedByDriver = 1U << 8;  // 운전자가 넘겨받아 15도 아래에서 손을 떼야 다시 조향
constexpr uint32_t kHudFlagTurnLeft = 1U << 9;           // 회전 desire(교차로 좌회전)
constexpr uint32_t kHudFlagTurnRight = 1U << 10;         // 회전 desire(교차로 우회전)
constexpr uint32_t kHudFlagBrakeLights = 1U << 11;       // 자차 브레이크등(페달 스트로크 또는 AUTO HOLD, brake_lights_on)

struct ControlState {
    uint64_t timestamp_ns = 0;
    uint32_t enabled = 0;
    uint32_t engaged = 0;
    uint32_t active = 0;
    uint32_t should_send = 0;
    uint32_t path_usable = 0;
    uint32_t seeds_ready = 0;
    uint32_t vehicle_fresh = 0;
    uint32_t steering_fault = 0;
    uint32_t left_blinker = 0;
    uint32_t right_blinker = 0;
    uint32_t cruise_active = 0;
    int32_t gear = 0;
    float cluster_speed_kph = 0.0f;
    float cruise_max_speed_kph = 0.0f;
    float cruise_command_speed_kph = 0.0f;
    float steering_angle_deg = 0.0f;
    float desired_curvature = 0.0f;
    float actual_curvature = 0.0f;
    float normalized_output = 0.0f;
    int32_t desired_torque = 0;
    int32_t apply_torque = 0;
    int32_t driver_torque = 0;
    uint32_t desire = 0;
    char active_block[32] = {};
    uint32_t radar_lead_valid = 0;
    float radar_lead_distance_m = 0.0f;
    float radar_lead_relative_speed_mps = 0.0f;
    uint32_t departure_alert_type = 0;
    uint32_t departure_alert_event_id = 0;
    uint32_t green_light_alert_armed = 0;
    uint32_t tpms_valid = 0;
    uint32_t tpms_unit = 0;
    float tpms_pressure_fl = 0.0f;
    float tpms_pressure_fr = 0.0f;
    float tpms_pressure_rl = 0.0f;
    float tpms_pressure_rr = 0.0f;
    uint32_t tpms_warning = 0;
    uint32_t hud_flags = 0;
    uint32_t engage_event_id = 0;
    uint32_t disengage_event_id = 0;
    uint32_t engage_reject_event_id = 0;
    char engage_reject_block[32] = {};
    float ego_speed_kph = 0.0f;
    uint32_t reserved = 0;  // 정렬 꼬리를 0으로 채워 발행·기록 바이트가 매번 같다
};

/* paramsd·torqued 출력(상류 vehicleParameters·lateralTorqueParameters). controlsd가
 * paramsd 출력마다(20 Hz) 발행하고 recordd가 LearnerState로 저장한다. */
constexpr uint32_t kLearnerVehicleInputsOk = 1U << 0;
constexpr uint32_t kLearnerVehicleValid = 1U << 1;
constexpr uint32_t kLearnerSensorValid = 1U << 2;
constexpr uint32_t kLearnerSteerRatioValid = 1U << 3;
constexpr uint32_t kLearnerStiffnessValid = 1U << 4;
constexpr uint32_t kLearnerOffsetAverageValid = 1U << 5;
constexpr uint32_t kLearnerOffsetValid = 1U << 6;
constexpr uint32_t kLearnerTorqueInputsOk = 1U << 7;
constexpr uint32_t kLearnerTorqueValid = 1U << 8;
constexpr uint32_t kLearnerUseVehicle = 1U << 9;   // 스위치 적용 후 실제 사용
constexpr uint32_t kLearnerUseTorque = 1U << 10;
constexpr uint32_t kLearnerVehicleRestored = 1U << 11;
constexpr uint32_t kLearnerTorqueRestored = 1U << 12;
constexpr uint32_t kLearnerUseDelay = 1U << 13;    // 경로 지연에 lagd 추정 사용 중
constexpr uint32_t kLearnerLocalizerInputs = 1U << 14;  // paramsd·torqued 입력이 locationd

struct LearnerState {
    uint64_t timestamp_ns = 0;
    uint32_t flags = 0;
    float steer_ratio = 0.0f;
    float stiffness_factor = 0.0f;
    float roll_rad = 0.0f;
    float angle_offset_average_deg = 0.0f;
    float angle_offset_deg = 0.0f;
    float steer_ratio_std = 0.0f;
    float stiffness_factor_std = 0.0f;
    float angle_offset_average_std = 0.0f;
    float angle_offset_fast_std = 0.0f;
    float yaw_bias_rad_s = 0.0f;
    float lat_accel_factor_raw = 0.0f;
    float lat_accel_offset_raw = 0.0f;
    float friction_raw = 0.0f;
    float lat_accel_factor = 0.0f;
    float lat_accel_offset = 0.0f;
    float friction = 0.0f;
    float decay = 0.0f;
    float max_resets = 0.0f;
    int32_t total_bucket_points = 0;
    int32_t cal_perc = 0;
    float road_bank_lat_accel = 0.0f;  // 기존 편경사 추정(비교용)
    // 학습기 사전값. controlsd 시작 때 파라미터에서 고정된다.
    float prior_steer_ratio = 0.0f;
    float prior_lat_accel_factor = 0.0f;
    float prior_friction = 0.0f;
    int16_t bucket_points[8] = {};
    float plan_delay_s = 0.0f;  // 실제 쓴 경로 지연(lagd 사용 중이면 추정값). 예전 기록은 0
};
/* recordd가 그대로 저장하고 tools/model/recording_reader.py LEARNER_STATE와
 * 웹 콘솔의 LEARNER_FIELDS가 위치로 읽는다(check_web_console.py가 대조). */
#define EDGEPILOT_LEARNER_STATE_AT(field, expected) \
    static_assert(offsetof(LearnerState, field) == (expected), \
                  "LearnerState." #field " moved")
EDGEPILOT_LEARNER_STATE_AT(flags, 8);
EDGEPILOT_LEARNER_STATE_AT(steer_ratio, 12);
EDGEPILOT_LEARNER_STATE_AT(yaw_bias_rad_s, 48);
EDGEPILOT_LEARNER_STATE_AT(lat_accel_factor_raw, 52);
EDGEPILOT_LEARNER_STATE_AT(max_resets, 80);
EDGEPILOT_LEARNER_STATE_AT(total_bucket_points, 84);
EDGEPILOT_LEARNER_STATE_AT(road_bank_lat_accel, 92);
EDGEPILOT_LEARNER_STATE_AT(prior_steer_ratio, 96);
EDGEPILOT_LEARNER_STATE_AT(bucket_points, 108);
EDGEPILOT_LEARNER_STATE_AT(plan_delay_s, 124);
#undef EDGEPILOT_LEARNER_STATE_AT
static_assert(sizeof(LearnerState) == 128, "LearnerState size");

/* 보드 IMU(LSM6DSOW, i2c-1 0x6B) 원시 샘플. imud가 104 Hz로 읽어 100 ms마다 묶어 발행하고
 * recordd가 RecordType::Imu로 그대로 남긴다. 축은 칩 좌표(보정·회전 없음)이고 자이로
 * 바이어스도 빼지 않는다: 차량/카메라 축과의 관계와 바이어스는 분석에서 구한다. */
struct ImuSample {
    uint64_t timestamp_ns = 0;   // CLOCK_BOOTTIME, 데이터 준비를 본 시각
    float accel_mps2[3] = {};
    float gyro_rad_s[3] = {};
    float temperature_c = 0.0f;
    uint32_t reserved = 0;
};

static_assert(sizeof(ImuSample) == 40, "ImuSample layout is shared with the recording reader");

constexpr unsigned kImuBatchMaxSamples = 16;

struct ImuBatch {
    uint64_t timestamp_ns = 0;   // 발행 시각
    uint32_t count = 0;
    uint32_t dropped = 0;        // 누적: 읽기 전에 덮어쓰인 샘플(데이터 준비 두 번 이상 놓침)
    ImuSample samples[kImuBatchMaxSamples] = {};
};

static_assert(sizeof(ImuBatch) == 16 + 40 * kImuBatchMaxSamples,
              "ImuBatch layout is shared with the recording reader");

/* locationd 출력: 자세 칼만 필터(상류 livePose)와 조향 지연 추정(상류 liveDelay).
 * IMU 묶음마다(약 20 Hz) 발행하고 recordd가 RecordType::Localization으로 그대로 남긴다.
 * 보정(차량) 좌표계는 x 앞, y 오른쪽, z 아래라 요레이트는 오른쪽 회전이 양수(곡률 관례와 같다). */
constexpr uint32_t kLocalizationFilterValid = 1U << 0;
constexpr uint32_t kLocalizationInputsOk = 1U << 1;
constexpr uint32_t kLocalizationSensorsOk = 1U << 2;
constexpr uint32_t kLocalizationPosenetOk = 1U << 3;
constexpr uint32_t kLocalizationCalibValid = 1U << 4;
constexpr uint32_t kLocalizationLagRestored = 1U << 5;
// input_flags: 거부 누적이 한도를 넘은 입력(inputs_ok를 떨어뜨린 것)과 차속 가드
constexpr uint32_t kLocalizationInvalidAccel = 1U << 0;
constexpr uint32_t kLocalizationInvalidGyro = 1U << 1;
constexpr uint32_t kLocalizationInvalidCamera = 1U << 2;
constexpr uint32_t kLocalizationCameraGuarded = 1U << 3;

struct LocalizationState {
    uint64_t timestamp_ns = 0;             // 추정 시각 = 마지막 IMU 샘플 시각(CLOCK_BOOTTIME)
    uint32_t flags = 0;
    uint32_t lag_status = 0;               // LateralLagStatus: 0 미추정, 1 추정, 2 무효
    float orientation_calib[3] = {};       // roll(오른쪽 아래 +), pitch, yaw
    float orientation_std[3] = {};         // 기기 좌표계 표준편차
    float angular_velocity_calib[3] = {};  // rad/s
    float angular_velocity_calib_std[3] = {};
    float velocity_device[3] = {};         // m/s
    float velocity_device_std[3] = {};
    float acceleration_calib[3] = {};      // m/s², 중력 제외
    float lateral_delay_s = 0.0f;          // 쓸 지연: 추정되면 추정, 아니면 초기값
    float lag_estimate_s = 0.0f;           // 진행 중 블록 포함 평균
    float lag_estimate_std_s = 0.0f;
    int32_t lag_valid_blocks = 0;
    int32_t lag_cal_perc = 0;
    uint32_t lag_points = 0;               // 창 안의 조건 만족 점 수
    uint32_t input_flags = 0;              // kLocalizationInvalid*, kLocalizationCameraGuarded
};
static_assert(sizeof(LocalizationState) == 128, "LocalizationState layout is shared with the recording reader");

/* recordd가 0.5초마다 발행하고 overlayd가 녹화 표시(REC)에 쓴다. */
struct RecordState {
    uint64_t timestamp_ns = 0;
    uint32_t active = 0;           // route를 쓰는 중
    uint32_t storage_blocked = 0;  // 저장 공간 여유가 모자라 녹화를 거부했거나 멈춤
};

/* controlsd가 발행하고 overlayd/recordd가 읽는 공유 레이아웃이다. 기록 v5는 이
 * 구조체를 그대로 저장하고 tools/model/recording_reader.py가 위치로 디코드하므로 필드
 * 순서까지 전부 고정한다. 같은 크기 필드 둘을 맞바꿔도 여기서 걸린다. */
#define EDGEPILOT_CONTROL_STATE_AT(field, expected) \
    static_assert(offsetof(ControlState, field) == (expected), \
                  "ControlState." #field " moved: recording v5 layout")
EDGEPILOT_CONTROL_STATE_AT(timestamp_ns, 0);
EDGEPILOT_CONTROL_STATE_AT(enabled, 8);
EDGEPILOT_CONTROL_STATE_AT(engaged, 12);
EDGEPILOT_CONTROL_STATE_AT(active, 16);
EDGEPILOT_CONTROL_STATE_AT(should_send, 20);
EDGEPILOT_CONTROL_STATE_AT(path_usable, 24);
EDGEPILOT_CONTROL_STATE_AT(seeds_ready, 28);
EDGEPILOT_CONTROL_STATE_AT(vehicle_fresh, 32);
EDGEPILOT_CONTROL_STATE_AT(steering_fault, 36);
EDGEPILOT_CONTROL_STATE_AT(left_blinker, 40);
EDGEPILOT_CONTROL_STATE_AT(right_blinker, 44);
EDGEPILOT_CONTROL_STATE_AT(cruise_active, 48);
EDGEPILOT_CONTROL_STATE_AT(gear, 52);
EDGEPILOT_CONTROL_STATE_AT(cluster_speed_kph, 56);
EDGEPILOT_CONTROL_STATE_AT(cruise_max_speed_kph, 60);
EDGEPILOT_CONTROL_STATE_AT(cruise_command_speed_kph, 64);
EDGEPILOT_CONTROL_STATE_AT(steering_angle_deg, 68);
EDGEPILOT_CONTROL_STATE_AT(desired_curvature, 72);
EDGEPILOT_CONTROL_STATE_AT(actual_curvature, 76);
EDGEPILOT_CONTROL_STATE_AT(normalized_output, 80);
EDGEPILOT_CONTROL_STATE_AT(desired_torque, 84);
EDGEPILOT_CONTROL_STATE_AT(apply_torque, 88);
EDGEPILOT_CONTROL_STATE_AT(driver_torque, 92);
EDGEPILOT_CONTROL_STATE_AT(desire, 96);
EDGEPILOT_CONTROL_STATE_AT(active_block, 100);
EDGEPILOT_CONTROL_STATE_AT(radar_lead_valid, 132);
EDGEPILOT_CONTROL_STATE_AT(radar_lead_distance_m, 136);
EDGEPILOT_CONTROL_STATE_AT(radar_lead_relative_speed_mps, 140);
EDGEPILOT_CONTROL_STATE_AT(departure_alert_type, 144);
EDGEPILOT_CONTROL_STATE_AT(departure_alert_event_id, 148);
EDGEPILOT_CONTROL_STATE_AT(green_light_alert_armed, 152);
EDGEPILOT_CONTROL_STATE_AT(tpms_valid, 156);
EDGEPILOT_CONTROL_STATE_AT(tpms_unit, 160);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_fl, 164);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_fr, 168);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_rl, 172);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_rr, 176);
EDGEPILOT_CONTROL_STATE_AT(tpms_warning, 180);
EDGEPILOT_CONTROL_STATE_AT(hud_flags, 184);
EDGEPILOT_CONTROL_STATE_AT(engage_event_id, 188);
EDGEPILOT_CONTROL_STATE_AT(disengage_event_id, 192);
EDGEPILOT_CONTROL_STATE_AT(engage_reject_event_id, 196);
EDGEPILOT_CONTROL_STATE_AT(engage_reject_block, 200);
EDGEPILOT_CONTROL_STATE_AT(ego_speed_kph, 232);
#undef EDGEPILOT_CONTROL_STATE_AT
static_assert(sizeof(ControlState) == 240,
              "ControlState layout is shared by controlsd, overlayd and the recording");
/* recordd가 RecordType::PandaState로 그대로 저장한다. */
static_assert(sizeof(PandaState) == 96,
              "PandaState is recorded as-is: bump kRecordingVersion");

#endif
