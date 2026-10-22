// controlsd 한 틱(ControlsTick)을 controlsd main처럼 10 ms마다 돌린다. K7 수신 CAN(신호 배치대로 채운
// 바이트), 20 Hz 직선 도로 모델, Panda 상태를 넣고 ControlState와 보낼 CAN을 본다: SET으로 결합해
// LKAS11을 보내고 문이 열리면 해제한다, Panda가 허가하지 않으면 1초 유예 뒤 거부한다, 브레이크
// 페달(AHB1)이 고정형 크루즈 추정을 끈다, locationd를 못 읽은 틱은 마지막 lagd 지연을 쓴다, 확률 문턱 밑의
// 모델 앞차는 쓰지 않는다, 새 모델의 플랜은 그 모델을 받은 틱에 쓴다.
#include "car/can_frame.h"
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "common/utils_file.h"
#include "controls/control_holds.h"
#include "controls/controls_tick.h"
#include "localization/lateral_lag.h"

#include <gtest/gtest.h>
#include <stdlib.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>

namespace {

struct K7Inputs {
    float speed_kph = 60.0f;
    int cruise_button = 0;  // CLU11 CF_Clu_CruiseSwState(kCruiseButton*)
    bool door_open = false;
    float brake_stroke_mm = 0.0f;
    bool wheel_speed = true;  // WHL_SPD11을 보낼지
    int driver_torque = 0;    // MDPS12 CR_Mdps_StrColTq
};

void add_frame(CanBatch *batch, uint32_t address, uint8_t bus, const std::array<uint8_t, 8> &data,
               uint32_t length = 8) {
    IpcCanFrame &frame = batch->frames[batch->count++];
    frame.address = address;
    frame.src = bus;
    frame.data_len = length;
    std::copy(data.begin(), data.begin() + length, frame.data);
}

// 한 틱의 K7 수신 프레임: 컨트롤러가 신선해야 하는 메시지 전부와 AHB1.
CanBatch k7_batch(const K7Inputs &in, uint64_t now_ns) {
    CanBatch batch;
    batch.valid = 1;
    batch.timestamp_ns = now_ns;
    std::array<uint8_t, 8> d{};
    add_frame(&batch, kHyundaiLkas11Address, kCameraBus, d);
    d = {};
    set_signal_le(&d, 0, 3, static_cast<uint32_t>(in.cruise_button));
    set_signal_le(&d, 8, 9, static_cast<uint32_t>(std::lround(in.speed_kph * 2.0f)));
    add_frame(&batch, kHyundaiClu11Address, kPowertrainBus, d, 4);
    d = {};
    add_frame(&batch, kHyundaiSas11Address, kMdpsBus, d);
    d = {};
    set_signal_le(&d, 0, 11, static_cast<uint32_t>(1024 + in.driver_torque));
    add_frame(&batch, kHyundaiMdps12Address, kMdpsBus, d);
    d = {};
    set_signal_le(&d, 0, 11, 1023);   // 횡가속 0
    set_signal_le(&d, 40, 13, 4095);  // 요레이트 0
    add_frame(&batch, kHyundaiEsp12Address, kPowertrainBus, d);
    if (in.wheel_speed) {
        d = {};
        for (const int start : {0, 16, 32, 48})
            set_signal_le(&d, start, 14, static_cast<uint32_t>(std::lround(in.speed_kph / 0.03125f)));
        add_frame(&batch, kHyundaiWhlSpd11Address, kPowertrainBus, d);
    }
    d = {};
    add_frame(&batch, kHyundaiTcs13Address, kPowertrainBus, d);
    add_frame(&batch, kHyundaiTcs15Address, kPowertrainBus, d);
    add_frame(&batch, kHyundaiEEms11Address, kPowertrainBus, d);
    add_frame(&batch, kHyundaiCgw2Address, kPowertrainBus, d);
    set_signal_le(&d, 16, 4, kGearDrive);
    add_frame(&batch, kHyundaiElectGearAddress, kPowertrainBus, d);
    d = {};
    set_signal_le(&d, 10, 2, 1);  // 안전벨트 착용
    if (in.door_open) set_signal_le(&d, 8, 2, 1);
    add_frame(&batch, kHyundaiCgw1Address, kPowertrainBus, d);
    d = {};
    set_signal_le(&d, 8, 16, static_cast<uint32_t>(std::lround(in.brake_stroke_mm * 10.0f)));
    add_frame(&batch, kHyundaiAhb1Address, kPowertrainBus, d);
    return batch;
}

// 66 m 앞까지 뻗은 직선 plan, 보정 완료.
ModelState straight_model(uint64_t now_ns) {
    ModelState model;
    model.valid = 1;
    model.capture_timestamp_ns = now_ns;
    model.model_timestamp_ns = now_ns;
    model.calibration.status = 1;
    for (int i = 0; i < kTrajectorySize; ++i) model.plan[i].x = 2.0f * static_cast<float>(i + 1);
    return model;
}

PandaState panda_state(uint64_t now_ns, bool controls_allowed) {
    PandaState state;
    state.timestamp_ns = now_ns;
    state.connected = state.comms_healthy = state.tx_enabled = 1;
    state.controls_allowed = controls_allowed ? 1U : 0U;
    state.safety_mode = kExpectedPandaSafetyModel;
    state.safety_param = kExpectedPandaSafetyParam;
    return state;
}

// controlsd main의 순서대로 틱을 돈다(Laneless: 차선 없이 plan으로 경로를 만든다).
class Drive {
public:
    explicit Drive(const ControlParams &params = laneless_params(), const std::string &vehicle_learn_json = {},
                   const std::string &torque_learn_cache = {})
        : tick_(params, false, vehicle_learn_json, torque_learn_cache) {}

    static ControlParams laneless_params() {
        ControlParams params;
        params.steering.laneless_mode = true;
        return params;
    }

    // seconds 동안 틱을 돌리고 마지막 ControlState를 돌려준다.
    ControlState run(double seconds, const K7Inputs &in, bool panda_allows = true) {
        ControlState state;
        for (int i = 0; i < static_cast<int>(std::lround(seconds * 100.0)); ++i) {
            now_ns_ += 10'000'000ULL;
            const double now_s = static_cast<double>(now_ns_) * 1e-9;
            tick_.on_can_batch(k7_batch(in, now_ns_), now_ns_, now_s);
            if (ticks_++ % 5 == 0) tick_.on_model(straight_model(now_ns_));
            tick_.on_panda(panda_state(now_ns_, panda_allows));
            tick_.on_localization(LocalizationRead{}, now_s);
            state = tick_.update(now_s, now_ns_);
            torque_writes_ += tick_.update_learners(now_s).torque_persist_data ? 1 : 0;
        }
        return state;
    }

    // 버튼을 0.05 s 누르고 뗀다.
    ControlState press(int button, K7Inputs in, bool panda_allows = true) {
        in.cruise_button = button;
        run(0.05, in, panda_allows);
        in.cruise_button = 0;
        return run(0.01, in, panda_allows);
    }

    const ControlsTick &tick() const { return tick_; }
    // update_learners가 내준 torqued 저장 수
    int torque_writes() const { return torque_writes_; }

private:
    uint64_t now_ns_ = 1'000'000'000ULL;
    unsigned ticks_ = 0;
    ControlsTick tick_;
    int torque_writes_ = 0;
};

bool sends(const ControlsTick &tick, uint32_t address, uint8_t bus) {
    const LateralControlOutput &result = tick.result();
    if (!result.should_send) return false;
    return std::any_of(result.frames.begin(), result.frames.end(),
                       [&](const CanFrame &frame) { return frame.address == address && frame.bus == bus; });
}

// 학습 파일은 내용이 바뀔 때만 쓴다: 배울 것이 없으면 처음 한 번 뒤로는 다시 쓰지 않는다. CAN이 없으면(차가
// 꺼짐) torqued는 쉬어 처음 한 번도 쓰지 않는다.
TEST(ControlsTick, LearnerFilesAreWrittenOnlyWhenTheyChange) {
    const ControlParams params;
    ControlsTick off(params, false, std::string(), std::string());
    int vehicle_writes = 0, torque_writes = 0;
    for (int i = 0; i < 13000; ++i) {  // 100 Hz로 130 s: paramsd 저장 주기 2번
        const LearnerOutput out = off.update_learners(1.0 + i * 0.01);
        vehicle_writes += out.vehicle_persist_data ? 1 : 0;
        torque_writes += out.torque_persist_data ? 1 : 0;
    }
    EXPECT_EQ(vehicle_writes, 1);
    EXPECT_EQ(torque_writes, 0);

    // 시동을 켜고 서 있으면 새 점이 없어 torqued는 처음 한 번만 쓴다(저장 주기 10번)
    K7Inputs parked;
    parked.speed_kph = 0.0f;
    Drive drive(params);
    drive.run(130.0, parked);
    EXPECT_EQ(drive.torque_writes(), 1);
    // 시작 때 읽은 내용과 같으면 첫 저장도 하지 않는다
    Drive restarted(params, std::string(), drive.tick().learners().torque_persist_data());
    restarted.run(30.0, parked);
    EXPECT_EQ(restarted.torque_writes(), 0);
}

TEST(ControlsTick, EngagesOnSetAndDisengagesWhenADoorOpens) {
    Drive drive;
    K7Inputs in;
    ControlState state = drive.run(1.0, in);
    ASSERT_EQ(state.engaged, 0);
    ASSERT_STREQ(state.active_block, "not_engaged");
    ASSERT_EQ(state.vehicle_fresh, 1) << "수신 프레임이 컨트롤러가 보는 메시지를 다 채운다";

    state = drive.press(kCruiseButtonSet, in);
    ASSERT_EQ(state.engaged, 1);
    ASSERT_EQ(state.engage_event_id, 1);
    state = drive.run(1.0, in);  // 경로 유효 0.5 s 디바운스를 넘긴다
    ASSERT_EQ(state.active, 1) << state.active_block;
    ASSERT_TRUE(sends(drive.tick(), kHyundaiLkas11Address, kPowertrainBus));
    ASSERT_TRUE(sends(drive.tick(), kHyundaiLkas11Address, kMdpsBus)) << "MDPS 버스에도 LKAS11을 보낸다";

    in.door_open = true;
    state = drive.run(0.05, in);
    ASSERT_EQ(state.engaged, 0) << "문 열림은 즉시 해제한다";
    ASSERT_EQ(state.active, 0);
    ASSERT_EQ(state.disengage_event_id, 1);
}

// lagd(locationd)는 controlsd의 핸들 조작·포화 판정을 hud_flags로 받는다(상류 lagd의 입력). 비트 15는 늘 켜서 옛
// 녹화와 가린다.
TEST(ControlsTick, ControlStateCarriesTheLagdInputs) {
    Drive drive;
    K7Inputs in;
    ControlState state = drive.run(0.5, in);
    EXPECT_TRUE(state.hud_flags & kHudFlagLagInputs);
    EXPECT_FALSE(state.hud_flags & (kHudFlagSteeringPressed | kHudFlagLatSaturated));
    in.driver_torque = 300;
    state = drive.run(0.1, in);
    EXPECT_TRUE(state.hud_flags & kHudFlagSteeringPressed) << "설정 문턱을 넘는 운전자 토크";
    EXPECT_EQ(state.driver_torque, 300);
    in.driver_torque = 0;
    state = drive.run(0.1, in);
    EXPECT_FALSE(state.hud_flags & kHudFlagSteeringPressed);
}

// 끝낼 때는 마지막 저장 뒤 바뀐 학습값만 내놓고, 그대로면 아무것도 내놓지 않는다
TEST(ControlsTick, FinalLearnerOutputsOnlyWhatChanged) {
    ControlsTick tick(Drive::laneless_params(), false, std::string(), std::string());
    const LearnerOutput first = tick.final_learner_outputs();
    ASSERT_TRUE(first.vehicle_persist_data.has_value()) << "시작 때 읽은 파일이 없으니 처음 것은 새 내용이다";
    const LearnerOutput again = tick.final_learner_outputs();
    EXPECT_FALSE(again.vehicle_persist_data.has_value()) << "방금 낸 것과 같다";
    EXPECT_FALSE(again.torque_persist_data.has_value());
}

TEST(ControlsTick, PandaRefusalRejectsEngageAfterTheGrace) {
    Drive drive;
    K7Inputs in;
    drive.run(1.0, in, false);
    ControlState state = drive.press(kCruiseButtonSet, in, false);
    ASSERT_EQ(state.engage_reject_event_id, 0) << "Panda 허가를 1초 기다린다";
    state = drive.run(1.1, in, false);
    ASSERT_EQ(state.engaged, 0);
    ASSERT_EQ(state.engage_reject_event_id, 1);
    ASSERT_STREQ(state.engage_reject_block, "panda_controls_off");
}

TEST(ControlsTick, BrakePedalCancelsTheCruiseEstimate) {
    Drive drive;
    K7Inputs in;
    drive.run(1.0, in);
    ControlState state = drive.press(kCruiseButtonSet, in);
    ASSERT_EQ(state.cruise_active, 1) << "SET은 고정형 크루즈 추정을 켠다";
    in.brake_stroke_mm = 15.0f;
    state = drive.run(0.05, in);
    ASSERT_EQ(state.cruise_active, 0) << "AHB1 페달 15 mm는 크루즈 추정을 끈다";
    ASSERT_EQ(state.engaged, 1) << "브레이크는 횡제어를 해제하지 않는다";
}

// locationd 상태를 읽다가 쓰기와 겹친 틱(read 거짓, state는 읽다 만 값)은 상류 SubMaster처럼 마지막으로
// 온전히 읽은 값을 쓴다. 그 틱에 lagd 지연을 버리면 토크 요청 버퍼를 읽는 위치가 바뀌어 토크가 튄다.
// CAN 묶음의 신선도는 꺼낸 시각으로 잰다: 그 시각보다 늦은 시각의 묶음은 버리므로, controlsd는 틱 시작 시각이
// 아니라 꺼낸 직후의 시각을 넘긴다(틱 안에 들어온 묶음을 버리던 문제)
TEST(ControlsTick, CanBatchFreshnessUsesThePopTime) {
    ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string());
    const uint64_t stamp = 5'000'000'000ULL;
    const CanBatch batch = k7_batch(K7Inputs{}, stamp);
    EXPECT_FALSE(tick.on_can_batch(batch, stamp - 1'000'000ULL, 5.0)) << "꺼내기 전 시각에는 미래 묶음이다";
    EXPECT_TRUE(tick.on_can_batch(batch, stamp + 2'000'000ULL, 5.0)) << "꺼낸 직후의 시각이면 받는다";
    EXPECT_FALSE(tick.on_can_batch(batch, stamp + 101'000'000ULL, 5.0)) << "100 ms가 지나면 낡았다";
}

TEST(ControlsTick, TornLocalizationReadKeepsTheLastLagEstimate) {
    ControlParams params;
    params.steering.use_live_delay = true;
    ControlsTick tick(params, false, std::string(), std::string());
    LocalizationRead good;
    good.open = good.read = true;
    good.read_ns = 10'000'000'000ULL;
    good.state.timestamp_ns = good.read_ns - 50'000'000ULL;
    good.state.lateral_delay_s = 0.3f;
    good.state.lag_status = static_cast<uint32_t>(LateralLagStatus::Estimated);
    tick.on_localization(good, 10.0);
    ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), 0.3f);

    LocalizationRead torn;
    torn.open = true;
    torn.read_ns = good.read_ns + 10'000'000ULL;
    torn.state.lateral_delay_s = 0.6f;  // 읽다 만 값
    tick.on_localization(torn, 10.01);
    ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), 0.3f) << "못 읽은 틱은 마지막 값을 쓴다";

    torn.read_ns = good.state.timestamp_ns + 10'000'000'000ULL;
    tick.on_localization(torn, 20.0);
    ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), 0.3f) << "상류처럼 locationd가 멈춰도 마지막 값을 쓴다";
}

// 모델 앞차는 openpilot radard처럼 고정한 확률(0.5) 밑이면 쓰지 않는다. 출발 알림과 비전 크루즈가 같은
// 판정(observe_vision_lead)을 받는다.
TEST(ControlsTick, VisionLeadNeedsTheFixedProbability) {
    ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string());
    const uint64_t now_ns = 2'000'000'000ULL;
    ModelState model = straight_model(now_ns);
    model.lead.valid = 1;
    model.lead.x = 30.0f;
    model.lead.velocity = 15.0f;
    model.lead.probability = kLeadProbabilityThreshold - 0.05f;
    tick.on_model(model);
    tick.update(2.0, now_ns);
    ASSERT_FALSE(tick.alert_input().lead_valid);
    model.lead.probability = kLeadProbabilityThreshold + 0.05f;
    tick.on_model(model);
    tick.update(2.0, now_ns);
    ASSERT_TRUE(tick.alert_input().lead_valid);
}

// 비전 크루즈는 modeld의 앞차 추적(모델 거리와 광류를 섞은 값)이 있으면 그 거리와 상대 속도를 받고, 출발
// 알림은 모델 앞차를 그대로 받는다(늦으면 안 된다). 추적이 없으면 크루즈도 모델 앞차다.
TEST(ControlsTick, CruiseTakesTheLeadTrackAndTheAlertTheModelLead) {
    ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string());
    const uint64_t now_ns = 2'000'000'000ULL;
    ModelState model = straight_model(now_ns);
    model.lead.valid = 1;
    model.lead.x = 30.0f;
    model.lead.velocity = 15.0f;
    model.lead.probability = 0.9f;
    model.lead_track.valid = 1;
    model.lead_track.distance_m = 32.0f;
    model.lead_track.relative_speed_mps = -2.0f;
    tick.on_model(model);
    tick.update(2.0, now_ns);
    ASSERT_TRUE(tick.cruise_input().vision_lead_valid);
    EXPECT_FLOAT_EQ(tick.cruise_input().vision_lead_distance_m, 32.0f - kRadarToCameraDistanceM);
    EXPECT_FLOAT_EQ(tick.cruise_input().vision_lead_relative_speed_mps, -2.0f);
    ASSERT_TRUE(tick.alert_input().lead_valid);
    EXPECT_FLOAT_EQ(tick.alert_input().lead_distance_m, 30.0f - kRadarToCameraDistanceM) << "출발 알림은 모델 앞차";

    model.lead_track.valid = 0;  // 추적이 없으면 크루즈도 모델 앞차
    tick.on_model(model);
    tick.update(2.0, now_ns);
    EXPECT_FLOAT_EQ(tick.cruise_input().vision_lead_distance_m, 30.0f - kRadarToCameraDistanceM);

    model.lead_track.valid = 1;  // 모델 앞차가 확률 문턱 밑이면 추적이 있어도 쓰지 않는다
    model.lead.probability = kLeadProbabilityThreshold - 0.05f;
    tick.on_model(model);
    tick.update(2.0, now_ns);
    EXPECT_FALSE(tick.cruise_input().vision_lead_valid);
}

// Lane 모드 모델: 차 양옆 1.75 m의 차선 둘과 직진 plan.
ModelState lane_model(uint64_t now_ns) {
    const float v = 20.0f;
    ModelState model;
    model.valid = 1;
    model.capture_timestamp_ns = model.model_timestamp_ns = now_ns;
    model.calibration.status = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        model.plan[i] = {v * t, 0.0f, 0.0f};
        model.lanes[1][i] = {-1.75f, 0.0f};
        model.lanes[2][i] = {1.75f, 0.0f};
    }
    model.lane_probabilities[1] = model.lane_probabilities[2] = 0.99f;
    model.lane_stds[1] = model.lane_stds[2] = 0.05f;
    model.desire_state[0] = 1.0f;
    return model;
}

// 웹 콘솔에서 바꿔 다시 읽은 파라미터는 다음 모델 프레임부터 플래너에 들어간다(update_params → 플래너).
TEST(ControlsTick, ReloadedParamsReachThePlanner) {
    ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string());
    uint64_t now_ns = 1'000'000'000ULL;
    for (int frame = 0; frame < 40; ++frame, now_ns += 50'000'000ULL) {
        tick.on_model(lane_model(now_ns));
        tick.update(static_cast<double>(now_ns) * 1e-9, now_ns);
    }
    ASSERT_FALSE(tick.target().laneless_mode) << "차선이 자리 잡으면 Lane 모드로 계획한다";
    params.steering.laneless_mode = true;
    tick.update_params(params);
    tick.on_model(lane_model(now_ns));
    tick.update(static_cast<double>(now_ns) * 1e-9, now_ns);
    EXPECT_TRUE(tick.target().laneless_mode) << "다시 읽은 뒤 첫 프레임부터 Laneless로 계획한다";
}

// 파라미터 파일은 감시자가 1초마다 보는 것으로만 다시 읽힌다: 웹 콘솔은 임시 파일 + rename으로 쓰고 알리지 않는다.
// 1초가 되기 전에는 보지 않고, 바뀐 뒤 처음 확인에서 읽으며(빠진 키는 지금 값 그대로), 그다음 확인은 바뀐 게 없다.
TEST(ControlsTick, ParamWatcherPicksUpAnAtomicRewriteWithinASecond) {
    char dir_template[] = "/tmp/edgepilot_params_XXXXXX";
    ASSERT_NE(mkdtemp(dir_template), nullptr);
    const std::string dir = dir_template;
    const ControlParamPaths paths{dir + "/steering.json", dir + "/adaptive_cruise.json"};
    ASSERT_TRUE(write_file_atomic(paths.steering, read_text_file("params/steering.json")));
    ASSERT_TRUE(write_file_atomic(paths.cruise, read_text_file("params/adaptive_cruise.json")));
    ControlParams params;
    std::string error;
    ASSERT_TRUE(load_control_params(paths, &params, &error)) << error;
    const uint64_t start_ns = 10'000'000'000ULL;
    ControlParamsWatcher watcher(paths, start_ns);

    ASSERT_TRUE(write_file_atomic(paths.steering, "{\"torque_kp\": 1.25}\n"));
    ControlParams reloaded = params;
    EXPECT_FALSE(watcher.poll(start_ns + 500'000'000ULL, &reloaded)) << "1초가 되기 전에는 보지 않는다";
    ASSERT_TRUE(watcher.poll(start_ns + 1'000'000'000ULL, &reloaded)) << "rename으로 바뀐 파일을 1초 확인에서 읽는다";
    EXPECT_FLOAT_EQ(reloaded.steering.torque_kp, 1.25f);
    EXPECT_FLOAT_EQ(reloaded.steering.steer_ratio, params.steering.steer_ratio) << "빠진 키는 지금 값 그대로";
    EXPECT_EQ(watcher.generation(), 2U);
    EXPECT_FALSE(watcher.poll(start_ns + 2'000'000'000ULL, &reloaded)) << "그다음 확인은 바뀐 게 없다";
    std::filesystem::remove_all(dir);
}

// 새 모델의 플랜은 그 모델을 받은 틱의 step이 바로 쓴다(예전 작업 스레드는 한 틱 늦어, 이 틱에는 직전
// 프레임의 플랜이 나왔다).
TEST(ControlsTick, UsesTheNewPlanInTheSameTick) {
    const ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string());
    uint64_t now_ns = 1'000'000'000ULL;
    for (int frame = 0; frame < 20; ++frame, now_ns += 50'000'000ULL) {
        tick.on_model(lane_model(now_ns));
        tick.update(static_cast<double>(now_ns) * 1e-9, now_ns);
        ASSERT_TRUE(tick.target().valid) << frame;
        ASSERT_EQ(tick.target().capture_timestamp_ns, now_ns) << "프레임 " << frame << "의 플랜을 같은 틱에 써야 한다";
    }
}

}  // namespace
