/* controlsd 한 틱(ControlsTick)을 controlsd main처럼 10 ms마다 돌린다. K7 수신 CAN(신호 배치대로 채운
 * 바이트), 20 Hz 직선 도로 모델, Panda 상태를 넣고 ControlState와 보낼 CAN을 본다: SET으로 결합해
 * LKAS11을 보내고 문이 열리면 해제한다, Panda가 허가하지 않으면 1초 유예 뒤 거부한다, 브레이크
 * 페달(AHB1)이 고정형 크루즈 추정을 끈다, 휠 속도가 끊기면 크루즈·발행 속도는 0.5초 뒤 비운다,
 * locationd를 못 읽은 틱은 마지막 lagd 지연을 쓴다, 확률 문턱 밑의 모델 앞차는 쓰지 않는다. 플래너는 그
 * 자리에서 계산한다(SyncPlanner).
 * 보드의 작업 스레드 플래너(LateralPlannerWorker)는 SyncPlanner와 같은 결과를 내는지 따로 본다. */
#include "car/can_frame.h"
#include "controls/control_holds.h"
#include "controls/controls_tick.h"
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "localization/lateral_lag.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace {

struct K7Inputs {
  float speed_kph = 60.0f;
  int cruise_button = 0;  // CLU11 CF_Clu_CruiseSwState(kCruiseButton*)
  bool door_open = false;
  float brake_stroke_mm = 0.0f;
  bool wheel_speed = true;  // WHL_SPD11을 보낼지
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
  set_signal_le(&d, 0, 11, 1024);  // 운전자 토크 0
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
  model.plan_probability = 0.9f;
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

/* controlsd main의 순서대로 틱을 돈다(Laneless: 차선 없이 plan으로 경로를 만든다). */
class Drive {
public:
  explicit Drive(const ControlParams &params = laneless_params())
      : planner_(params.steering),
        tick_(params, false, planner_, std::string(), std::string(), 1) {
    tick_.controller().set_clock([this] { return now_ns_; });
  }

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
      tick_.on_localization(LocalizationRead{}, now_ns_, now_s);
      state = tick_.step(now_s, now_ns_);
      tick_.update_learners(now_s);
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

private:
  uint64_t now_ns_ = 1'000'000'000ULL;
  unsigned ticks_ = 0;
  SyncPlanner planner_;
  ControlsTick tick_;
};

bool sends(const ControlsTick &tick, uint32_t address, uint8_t bus) {
  const LateralControlResult &result = tick.result();
  if (!result.should_send) return false;
  return std::any_of(result.frames.begin(), result.frames.end(),
                     [&](const CanFrame &frame) { return frame.address == address && frame.bus == bus; });
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

TEST(ControlsTick, CruiseSpeedGoesStaleAfterHalfASecond) {
  ControlParams params = Drive::laneless_params();
  params.timing.vehicle_state_timeout_ms = 1000;
  Drive drive(params);
  K7Inputs in;
  ControlState state = drive.run(1.0, in);
  ASSERT_FLOAT_EQ(state.ego_speed_kph, 60.0f);
  in.wheel_speed = false;
  state = drive.run(0.7, in);
  ASSERT_TRUE(std::isfinite(drive.tick().result().control_speed_kph))
      << "조향은 설정한 1 s까지 마지막 휠 속도를 쓴다";
  ASSERT_TRUE(std::isnan(state.ego_speed_kph)) << "크루즈·경보·발행 속도는 0.5 s가 지나면 쓰지 않는다";
}

/* locationd 상태를 읽다가 쓰기와 겹친 틱(read 거짓, state는 읽다 만 값)은 상류 SubMaster처럼 마지막으로
 * 온전히 읽은 값을 쓴다. 그 틱에 lagd 지연을 버리면 토크 요청 버퍼를 읽는 위치가 바뀌어 토크가 튄다. */
TEST(ControlsTick, TornLocalizationReadKeepsTheLastLagEstimate) {
  ControlParams params;
  params.steering.use_live_delay = true;
  SyncPlanner planner(params.steering);
  ControlsTick tick(params, false, planner, std::string(), std::string(), 1);
  LocalizationRead good;
  good.open = good.read = true;
  good.read_ns = 10'000'000'000ULL;
  good.state.timestamp_ns = good.read_ns - 50'000'000ULL;
  good.state.lateral_delay_s = 0.3f;
  good.state.lag_status = static_cast<uint32_t>(LateralLagStatus::Estimated);
  tick.on_localization(good, good.read_ns, 10.0);
  ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), 0.3f);

  LocalizationRead torn;
  torn.open = true;
  torn.read_ns = good.read_ns + 10'000'000ULL;
  torn.state.lateral_delay_s = 0.6f;  // 읽다 만 값
  tick.on_localization(torn, torn.read_ns, 10.01);
  ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), 0.3f) << "못 읽은 틱은 마지막 값을 쓴다";

  torn.read_ns = good.state.timestamp_ns + 2'100'000'000ULL;
  tick.on_localization(torn, torn.read_ns, 12.1);
  ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), params.steering.steer_actuator_delay)
      << "마지막 값도 2초가 지나면 낡았다";
}

/* 모델 앞차는 openpilot radard처럼 고정한 확률(0.5) 밑이면 쓰지 않는다. 출발 알림과 비전 크루즈가 같은
 * 판정(observe_vision_lead)을 받는다. */
TEST(ControlsTick, VisionLeadNeedsTheFixedProbability) {
  ControlParams params;
  SyncPlanner planner(params.steering);
  ControlsTick tick(params, false, planner, std::string(), std::string(), 1);
  const uint64_t now_ns = 2'000'000'000ULL;
  ModelState model = straight_model(now_ns);
  model.lead.valid = 1;
  model.lead.x = 30.0f;
  model.lead.velocity = 15.0f;
  model.lead.probability = kLeadProbabilityThreshold - 0.05f;
  tick.on_model(model);
  tick.step(2.0, now_ns);
  ASSERT_FALSE(tick.alert_input().lead_valid);
  model.lead.probability = kLeadProbabilityThreshold + 0.05f;
  tick.on_model(model);
  tick.step(2.0, now_ns);
  ASSERT_TRUE(tick.alert_input().lead_valid);
}

// Lane 모드 모델: 차선 둘과 직진 plan. 차선 중심이 차에서 offset_m만큼 떨어져 있다.
ModelState lane_model(uint64_t now_ns, float offset_m) {
  const float v = 20.0f;
  ModelState model;
  model.valid = 1;
  model.capture_timestamp_ns = model.model_timestamp_ns = now_ns;
  model.plan_probability = 0.9f;
  model.calibration.status = 1;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float t = model_t_idx(i);
    model.model_t[i] = model.lane_t[i] = t;
    model.plan[i] = {v * t, 0.0f, 0.0f};
    model.lanes[1][i] = {v * t, offset_m - 1.75f, 0.0f};
    model.lanes[2][i] = {v * t, offset_m + 1.75f, 0.0f};
  }
  model.lane_probabilities[1] = model.lane_probabilities[2] = 0.99f;
  model.lane_stds[1] = model.lane_stds[2] = 0.05f;
  model.desire_state[0] = 1.0f;
  return model;
}

// 작업 스레드가 이 캡처 시각의 결과를 낼 때까지 기다린다(최대 5초).
LateralTarget wait_for(const LateralPlannerWorker &worker, uint64_t capture_ns) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  LateralTarget target = worker.latest();
  while (target.capture_timestamp_ns != capture_ns && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::microseconds(200));
    target = worker.latest();
  }
  return target;
}

void expect_same_target(const LateralTarget &a, const LateralTarget &b, int frame) {
  EXPECT_EQ(a.valid, b.valid) << frame;
  EXPECT_EQ(a.mpc_solution_valid, b.mpc_solution_valid) << frame;
  EXPECT_EQ(a.laneless_mode, b.laneless_mode) << frame;
  EXPECT_EQ(a.target_y_m, b.target_y_m) << frame;
  EXPECT_EQ(a.heading_rad, b.heading_rad) << frame;
  EXPECT_EQ(a.curvature, b.curvature) << frame;
  for (int i = 0; i < kLateralControlN; ++i) {
    EXPECT_EQ(a.psis[i], b.psis[i]) << frame << " psi " << i;
    EXPECT_EQ(a.curvatures[i], b.curvatures[i]) << frame << " curvature " << i;
  }
}

/* 보드의 작업 스레드 플래너는 같은 입력 순서면 그 자리에서 계산한 것(SyncPlanner)과 비트까지 같다.
 * MPC와 차선 플래너가 프레임 사이에 상태를 이어 가므로 차선 중심을 프레임마다 옮긴다. */
TEST(PlannerWorker, MatchesTheSyncPlanner) {
  SteeringParams steering;
  LateralPlannerWorker worker(steering);
  SyncPlanner sync(steering);
  VehicleCanState vehicle{};
  for (int frame = 0; frame < 30; ++frame) {
    const uint64_t now_ns = 1'000'000'000ULL + static_cast<uint64_t>(frame) * 50'000'000ULL;
    const ModelState model = lane_model(now_ns, 0.3f * std::sin(0.2f * static_cast<float>(frame)));
    worker.submit(model, vehicle, 20.0f, 0.0f, true);
    sync.submit(model, vehicle, 20.0f, 0.0f, true);
    const LateralTarget target = wait_for(worker, now_ns);
    ASSERT_EQ(target.capture_timestamp_ns, now_ns) << "작업 스레드가 프레임 " << frame << "을 끝내지 못했다";
    expect_same_target(target, sync.latest(), frame);
  }
}

// 바꾼 파라미터는 바로 다음 프레임부터 쓴다(작업 스레드가 요청보다 먼저 적용한다).
TEST(PlannerWorker, ParamsApplyFromTheNextFrame) {
  SteeringParams steering;
  LateralPlannerWorker worker(steering);
  VehicleCanState vehicle{};
  uint64_t now_ns = 1'000'000'000ULL;
  LateralTarget target;
  for (int frame = 0; frame < 40; ++frame, now_ns += 50'000'000ULL) {
    worker.submit(lane_model(now_ns, 0.0f), vehicle, 20.0f, 0.0f, true);
    target = wait_for(worker, now_ns);
  }
  ASSERT_FALSE(target.laneless_mode) << "차선이 자리 잡으면 Lane 모드 경로";
  steering.laneless_mode = true;
  worker.update_params(steering);
  worker.submit(lane_model(now_ns, 0.0f), vehicle, 20.0f, 0.0f, true);
  target = wait_for(worker, now_ns);
  ASSERT_EQ(target.capture_timestamp_ns, now_ns);
  ASSERT_TRUE(target.laneless_mode) << "Laneless 모드로 바꾼 뒤의 첫 프레임";
}

}  // namespace
