/* 녹화를 controlsd 한 틱의 로직(ControlsTick)에 다시 흘린다. 10 ms 틱마다 그 시각까지 기록된 CAN·
 * 모델·Panda·locationd 상태를 넣고 step/update_learners를 돌려, 보낼 CAN과 ControlState를 쓴다.
 * 플래너는 그 자리에서 계산하고(SyncPlanner) 시계는 틱 시각이라, 같은 입력이면 출력이 비트 단위로
 * 같다. controlsd 리팩토링 전후 비교와, 녹화된 실제 출력과의 대조에 쓴다.
 * 사용: replay_controls [--steering s.json] [--cruise c.json] [--vehicle-json j]
 *                       [--force-engaged] [--dump out.txt] <events.bin...>
 *   --force-engaged는 EDGEPILOT_FORCE_ENGAGED처럼 버튼 없이 결합한다(녹화 전에 결합해 둔 route용).
 *   마지막 줄에 출력 전체의 FNV-1a 다이제스트와, 녹화된 ControlState와의 일치율을 쓴다. */
#include "controls/adaptive_cruise.h"
#include "controls/control_params.h"
#include "controls/controls_tick.h"
#include "recording/event_log_reader.h"
#include "common/ipc_messages.h"
#include "recording/recorded_can.h"
#include "recording/recorded_model_state.h"
#include "recording/recording_format.h"
#include "common/utils_file.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace {

struct Record {
  uint64_t timestamp_ns = 0;
  RecordType type = RecordType::CanRx;
  uint32_t version = kRecordingVersion;  // 레코드가 든 파일의 기록 버전
  std::vector<char> payload;
};

struct Digest {
  uint64_t value = 1469598103934665603ULL;
  void add(const void *data, size_t size) {
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
      value ^= bytes[i];
      value *= 1099511628211ULL;
    }
  }
};

}  // namespace

int main(int argc, char **argv) {
  std::string steering_path, cruise_path, vehicle_json_path, dump_path;
  bool force_engaged = false;
  std::vector<std::string> events;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
    if (arg == "--steering") steering_path = next();
    else if (arg == "--cruise") cruise_path = next();
    else if (arg == "--vehicle-json") vehicle_json_path = next();
    else if (arg == "--dump") dump_path = next();
    else if (arg == "--force-engaged") force_engaged = true;
    else if (arg.rfind("--", 0) == 0) events.clear(), i = argc;
    else events.push_back(arg);
  }
  if (events.empty()) {
    std::fprintf(stderr, "usage: %s [--steering s.json] [--cruise c.json] "
                         "[--vehicle-json live_parameters.json] [--force-engaged] [--dump out.txt] <events.bin...>\n", argv[0]);
    return 2;
  }

  ControlParams params;
  std::string error;
  if ((!steering_path.empty() && !load_steering_params_json(steering_path, &params.steering, &error)) ||
      (!cruise_path.empty() && !load_adaptive_cruise_params_json(cruise_path, &params.cruise, &error))) {
    std::fprintf(stderr, "params: %s\n", error.c_str());
    return 1;
  }
  const std::string vehicle_json = vehicle_json_path.empty() ? std::string() : read_text_file(vehicle_json_path);

  std::deque<Record> records;
  for (const std::string &path : events) {
    EventLogReader reader(path);
    if (!reader.ok()) continue;
    EventRecordHeader rh{};
    std::vector<char> buf;
    while (reader.next(&rh, &buf)) {
      const auto type = static_cast<RecordType>(rh.type);
      if (type != RecordType::CanRx && type != RecordType::ModelState && type != RecordType::PandaState &&
          type != RecordType::Localization && type != RecordType::ControlState)
        continue;
      records.push_back({rh.timestamp_ns, type, reader.version(), buf});
    }
  }
  if (records.empty()) {
    std::fprintf(stderr, "no records\n");
    return 1;
  }
  /* recordd는 쌓인 CAN을 먼저 쓰고 상태 스냅샷을 그 뒤에 쓰므로 파일 순서는 시각 순이 아니다. 시각 순으로
   * 늘어놓되 첫 틱은 파일의 첫 레코드 시각에 둔다(route 첫 PandaState가 1초쯤 앞서 있어도 틱을 늘리지 않는다). */
  const uint64_t first_record_ns = records.front().timestamp_ns;
  std::stable_sort(records.begin(), records.end(),
                   [](const Record &a, const Record &b) { return a.timestamp_ns < b.timestamp_ns; });

  SyncPlanner planner(params.steering);
  ControlsTick tick(params, force_engaged, planner, vehicle_json, std::string(), 1);
  uint64_t tick_ns = first_record_ns;
  tick.controller().set_clock([&tick_ns] { return tick_ns; });
  const uint64_t start_ns = tick_ns;

  std::FILE *dump = dump_path.empty() ? nullptr : std::fopen(dump_path.c_str(), "w");
  Digest digest;
  LocalizationRead localization;
  unsigned long ticks = 0, sent = 0, compared = 0, same_active = 0, same_engaged = 0, same_desire = 0;
  double torque_error = 0.0;
  ControlState last_control{};
  bool have_control = false;

  while (!records.empty()) {
    tick_ns += 10'000'000ULL;
    const double now_s = static_cast<double>(tick_ns - start_ns) * 1e-9;
    while (!records.empty() && records.front().timestamp_ns <= tick_ns) {
      const Record &r = records.front();
      switch (r.type) {
        case RecordType::CanRx:
          tick.on_can_batch(decode_recorded_can(r.payload.data(), r.payload.size(), tick_ns), tick_ns, now_s);
          break;
        case RecordType::ModelState: {
          ModelState model{};
          if (decode_recorded_model_state(r.payload.data(), static_cast<uint32_t>(r.payload.size()), r.version,
                                          &model))
            tick.on_model(model, now_s);
          break;
        }
        case RecordType::PandaState: {
          PandaState panda{};
          std::memcpy(&panda, r.payload.data(), std::min(sizeof(panda), r.payload.size()));
          tick.on_panda(panda);
          break;
        }
        case RecordType::Localization:
          localization.open = true;
          localization.read = true;
          std::memcpy(&localization.state, r.payload.data(), std::min(sizeof(localization.state), r.payload.size()));
          break;
        case RecordType::ControlState:
          if (r.payload.size() >= sizeof(ControlState)) {
            std::memcpy(&last_control, r.payload.data(), sizeof(last_control));
            have_control = true;
          }
          break;
        default:
          break;
      }
      records.pop_front();
    }
    localization.read_ns = tick_ns;
    tick.on_localization(localization, tick_ns, now_s);
    ControlState state = tick.step(now_s, tick_ns);
    const LateralControlResult &result = tick.result();
    const bool send = result.should_send && !result.frames.empty();
    digest.add(&state, sizeof(state));
    if (send) {
      ++sent;
      for (const CanFrame &frame : result.frames) {
        digest.add(&frame.address, sizeof(frame.address));
        digest.add(&frame.bus, sizeof(frame.bus));
        digest.add(frame.data.data(), frame.length);
      }
    }
    LearnerOutputs learned = tick.update_learners(now_s);
    if (learned.learner_state) digest.add(&*learned.learner_state, sizeof(LearnerState));
    if (learned.vehicle_json) digest.add(learned.vehicle_json->data(), learned.vehicle_json->size());
    if (dump) {
      std::fprintf(dump, "%lu eng=%u act=%u block=%s torque=%d/%d curv=%a/%a desire=%u cruise=%u/%a/%a",
                   ticks, state.engaged, state.active, state.active_block, state.desired_torque, state.apply_torque,
                   state.desired_curvature, state.actual_curvature, state.desire, state.cruise_active,
                   state.cruise_max_speed_kph, state.cruise_command_speed_kph);
      if (send)
        for (const CanFrame &frame : result.frames) {
          std::fprintf(dump, " %x@%u:", frame.address, frame.bus);
          for (size_t b = 0; b < frame.length; ++b) std::fprintf(dump, "%02x", frame.data[b]);
        }
      std::fputc('\n', dump);
    }
    // 녹화된 ControlState(controlsd가 그 틱에 실제로 낸 값)와 대조
    if (have_control && last_control.timestamp_ns + 10'000'000ULL >= tick_ns) {
      ++compared;
      same_engaged += state.engaged == last_control.engaged;
      same_active += state.active == last_control.active;
      same_desire += state.desire == last_control.desire;
      torque_error += std::abs(state.apply_torque - last_control.apply_torque);
    }
    ++ticks;
  }
  if (dump) std::fclose(dump);
  std::printf("ticks %lu sent %lu digest %016llx\n", ticks, sent, static_cast<unsigned long long>(digest.value));
  if (compared > 0)
    std::printf("vs recorded ControlState (%lu ticks): engaged %.1f%% active %.1f%% desire %.1f%% |apply torque| %.2f\n",
                compared, 100.0 * same_engaged / compared, 100.0 * same_active / compared,
                100.0 * same_desire / compared, torque_error / compared);
  return 0;
}
