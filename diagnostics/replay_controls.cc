// 녹화를 controlsd 한 틱의 로직(ControlsTick)에 다시 흘린다. 10 ms 틱마다 그 시각까지 기록된 CAN·
// 모델·Panda·locationd 상태를 넣고 step/update_learners를 돌려, 보낼 CAN과 ControlState를 쓴다.
// 한 틱 안에서는 controlsd main처럼 CAN을 먼저, 그다음 모델·Panda·locationd를 넣고, 플래너는 controlsd처럼 그
// 자리에서 계산하며 시계는 틱 시각이라, 같은 입력이면 출력이 비트 단위로 같다. controlsd 리팩토링 전후 비교와, 녹화된
// 실제 출력과의 대조에 쓴다. 보드와 틱 경계(10 ms 격자의 위상)가 다르므로 녹화 출력과 틱마다 같지는 않다. 2026-10-13
// 전 녹화에서는 보드가 새 플랜을 한 틱 늦게 썼으므로(플래너 작업 스레드) 모델 프레임의 첫 틱이 그만큼 더 다르다.
// 사용: replay_controls [--steering s.json] [--cruise c.json] [--vehicle-json j] [--torque-cache c.bin]
//                       [--force-engaged] [--lane-filter W] [--lateral-nn m.json]
//                       [--nn-inputs out.csv [--nn-inputs-every N]] [--dump out.txt] <events.bin...>
//   --vehicle-json과 --torque-cache는 controlsd가 시작할 때 복원하는 paramsd·torqued 저장값이다(route/params의
//   live_parameters.json, live_torque_parameters.bin). 없으면 학습기는 사전값에서 새로 시작한다(보드와 다르다).
//   --force-engaged는 EDGEPILOT_FORCE_ENGAGED처럼 버튼 없이 결합한다(녹화 전에 결합해 둔 route용).
//   --lane-filter는 차선선 시간 필터의 새 측정 가중이다(planning/lane_line_filter, 1이면 끈다).
//   --lateral-nn은 학습한 토크 모델 파일이다(docs/lateral-nn.md, 녹화 route의 models/lateral_nn.json이 그 주행의
//   모델이다). 쓰려면 --steering의 nn_feedforward도 켠다. 켜져 있는데 모델을 주지 않으면 경고하고 선형으로 돈다.
//   --nn-inputs는 모델을 계산한 틱마다(N틱에 한 번, 기본 1) 그 입력을 CSV로 쓴다: t(가장 이른 기록부터 초,
//   extract_lateral_dataset의 t와 같은 기준)와 모델 파일의 features 이름 열. 계획 조건 학습(nnlc)의 입력이다.
//   --lateral-nn이 있어야 하고 nn_feedforward를 켠다(열린 루프라 피드포워드가 입력을 바꾸지 않는다).
//   마지막 줄에 출력 전체의 FNV-1a 다이제스트와, 녹화된 ControlState와의 일치율을 쓴다.
#include "cli_args.h"
#include "common/ipc_messages.h"
#include "common/utils_file.h"
#include "controls/adaptive_cruise.h"
#include "controls/control_params.h"
#include "controls/controls_tick.h"
#include "recording/event_log_reader.h"
#include "recording/recorded_can.h"
#include "recording/recording_format.h"
#include "route_events.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace {

using Record = TimeOrderedEvents::Record;

// 틱에 넣는 레코드 종류
bool replayed(const Record &r) {
    const auto type = record_type(r.header);
    return type == RecordType::CanRx || type == RecordType::ModelState || type == RecordType::PandaState ||
           type == RecordType::Localization || type == RecordType::ControlState;
}

// 파일 순서로 첫 번째인 틱 레코드의 시각(첫 틱의 자리). 없으면 0.
uint64_t first_record_in_file_order(const std::vector<std::string> &paths) {
    Record r;
    for (const std::string &path : paths) {
        EventLogReader reader(path);
        while (reader.next(&r.header, &r.payload))
            if (replayed(r)) return r.header.timestamp_ns;
    }
    return 0;
}

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
    std::string steering_path, cruise_path, vehicle_json_path, torque_cache_path, dump_path;
    bool force_engaged = false;
    std::string lateral_nn_path, nn_inputs_path;
    long nn_inputs_every = 1;
    double lane_filter = 0.0;  // 0이면 코드 기본값
    std::vector<std::string> events;
    ArgReader args(argc, argv);
    while (args.next()) {
        if (args.is("--steering"))
            steering_path = args.value();
        else if (args.is("--cruise"))
            cruise_path = args.value();
        else if (args.is("--vehicle-json"))
            vehicle_json_path = args.value();
        else if (args.is("--torque-cache"))
            torque_cache_path = args.value();
        else if (args.is("--dump"))
            dump_path = args.value();
        else if (args.is("--force-engaged"))
            force_engaged = true;
        else if (args.is("--lane-filter"))
            args.read(&lane_filter, 0.0, 1.0);
        else if (args.is("--lateral-nn"))
            lateral_nn_path = args.value();
        else if (args.is("--nn-inputs"))
            nn_inputs_path = args.value();
        else if (args.is("--nn-inputs-every"))
            args.read(&nn_inputs_every, 1);
        else if (args.is_option())
            args.fail();
        else
            events.push_back(args.arg());
    }
    if (args.error() || events.empty() || (!nn_inputs_path.empty() && lateral_nn_path.empty())) {
        std::fprintf(stderr,
                     "usage: %s [--steering s.json] [--cruise c.json] [--vehicle-json live_parameters.json] "
                     "[--torque-cache live_torque_parameters.bin] "
                     "[--force-engaged] [--lane-filter W] [--lateral-nn m.json] "
                     "[--nn-inputs out.csv [--nn-inputs-every N]] [--dump out.txt] <events.bin...>\n",
                     argv[0]);
        return 2;
    }

    ControlParams params;
    std::string error;
    if ((!steering_path.empty() && !load_steering_params(steering_path, &params.steering, &error)) ||
        (!cruise_path.empty() && !load_adaptive_cruise_params(cruise_path, &params.cruise, &error))) {
        std::fprintf(stderr, "params: %s\n", error.c_str());
        return 1;
    }
    if (!nn_inputs_path.empty()) params.steering.nn_feedforward = true;
    if (params.steering.nn_feedforward && lateral_nn_path.empty())
        std::fprintf(stderr,
                     "warning: nn_feedforward is on but no --lateral-nn model is given; the replay uses the "
                     "linear feed-forward\n");
    const std::string vehicle_json = vehicle_json_path.empty() ? std::string() : read_text_file(vehicle_json_path);
    const std::string torque_cache = torque_cache_path.empty() ? std::string() : read_text_file(torque_cache_path);
    if (!torque_cache_path.empty() && torque_cache.empty()) {
        std::fprintf(stderr, "cannot read %s\n", torque_cache_path.c_str());
        return 1;
    }

    // 레코드는 기록 시각 순으로 흘려 읽는다(RouteEvents: recordd는 쌓인 CAN을 먼저 쓰고 상태 스냅샷을 그 뒤에 쓴다). 첫
    // 틱은 파일 순서로 첫 레코드의 시각에 둔다(route 첫 PandaState가 1초쯤 앞서 있어도 틱을 늘리지 않는다).
    // --nn-inputs의 t 기준은 가장 이른 기록이다(extract_lateral_dataset의 route 시작과 같다).
    const uint64_t first_record_ns = first_record_in_file_order(events);
    RouteEvents route_events(events);
    Record pending;
    bool have_pending = route_events.next(&pending);
    const uint64_t origin_ns = have_pending ? pending.header.timestamp_ns : 0;
    const auto advance = [&] {
        do {
            have_pending = route_events.next(&pending);
        } while (have_pending && !replayed(pending));
    };
    if (have_pending && !replayed(pending)) advance();
    if (!have_pending || first_record_ns == 0) {
        std::fprintf(stderr, "no records\n");
        return 1;
    }

    ControlsTick tick(params, force_engaged, vehicle_json, torque_cache);
    if (!torque_cache.empty() && tick.learners().torque_restore_status() != TorqueRestore::Restored)
        std::fprintf(stderr, "warning: torqued cache not restored (corrupt, other tuning or other source)\n");
    if (lane_filter > 0.0) tick.planner().set_lane_filter_weight(lane_filter);
    std::FILE *nn_inputs = nullptr;
    if (!lateral_nn_path.empty()) {
        auto nn = std::make_shared<LateralNnModel>();
        if (!nn->load(lateral_nn_path, &error)) {
            std::fprintf(stderr, "lateral nn: %s\n", error.c_str());
            return 1;
        }
        tick.set_lateral_nn(nn);
        if (!nn_inputs_path.empty()) {
            nn_inputs = std::fopen(nn_inputs_path.c_str(), "w");
            if (!nn_inputs) {
                std::fprintf(stderr, "cannot write %s\n", nn_inputs_path.c_str());
                return 1;
            }
            std::fputc('t', nn_inputs);
            for (const std::string &name : nn->spec().names()) std::fprintf(nn_inputs, ",%s", name.c_str());
            std::fputc('\n', nn_inputs);
        }
    }
    uint64_t tick_ns = first_record_ns;
    const uint64_t start_ns = tick_ns;

    std::FILE *dump = dump_path.empty() ? nullptr : std::fopen(dump_path.c_str(), "w");
    Digest digest;
    LocalizationRead localization;
    std::vector<Record> tick_records;
    CanBatch can_batch;  // 21.5 KB라 틱마다 새로 만들지 않는다
    unsigned long ticks = 0, sent = 0, compared = 0, same_active = 0, same_engaged = 0, same_desire = 0;
    // 둘 다 조향 중인 틱에서 경로 출처(Laneless 비트)와 학습 토크 모델 사용이 같은지. route params·모델 사본이 보드가
    // 그때 쓴 것과 다르면(주행 중 웹 콘솔로 바꿈 등) 여기서 드러난다.
    unsigned long both_active = 0, same_laneless = 0, same_nn = 0;
    double torque_error = 0.0;
    ControlState last_control{};
    bool have_control = false;

    while (have_pending) {
        tick_ns += 10'000'000ULL;
        const double now_s = static_cast<double>(tick_ns - start_ns) * 1e-9;
        // controlsd main처럼 이번 틱까지 쌓인 CAN을 먼저 모두 넣고 그 뒤에 모델·Panda·locationd를 넣는다. 시각 순으로만
        // 넣으면 같은 틱에서 CAN보다 먼저 찍힌 모델이 그 CAN이 반영되기 전 차량 상태로 플래너를 돌린다.
        tick_records.clear();
        while (have_pending && pending.header.timestamp_ns <= tick_ns) {
            tick_records.push_back(std::move(pending));
            advance();
        }
        std::stable_partition(tick_records.begin(), tick_records.end(),
                              [](const Record &r) { return record_type(r.header) == RecordType::CanRx; });
        for (const Record &r : tick_records) {
            switch (record_type(r.header)) {
                case RecordType::CanRx:
                    if (decode_recorded_can(r.payload.data(), r.payload.size(), tick_ns, &can_batch))
                        tick.on_can_batch(can_batch, tick_ns, now_s);
                    break;
                case RecordType::ModelState: {
                    ModelState model{};
                    if (read_record(r.payload, &model)) tick.on_model(model);
                    break;
                }
                case RecordType::PandaState: {
                    PandaState panda;
                    if (read_record(r.payload, &panda)) tick.on_panda(panda);
                    break;
                }
                case RecordType::Localization:
                    localization.open = true;
                    localization.read = read_record(r.payload, &localization.state);
                    break;
                case RecordType::ControlState:
                    if (read_record(r.payload, &last_control)) have_control = true;
                    break;
                default:
                    break;
            }
        }
        localization.read_ns = tick_ns;
        tick.on_localization(localization, now_s);
        ControlState state = tick.update(now_s, tick_ns);
        const LateralControlOutput &result = tick.result();
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
        if (nn_inputs && ticks % static_cast<unsigned long>(nn_inputs_every) == 0 &&
            tick.controller().lateral_nn_evaluated()) {
            std::fprintf(nn_inputs, "%.3f", (static_cast<double>(tick_ns) - static_cast<double>(origin_ns)) * 1e-9);
            for (float value : tick.controller().lateral_nn_inputs()) std::fprintf(nn_inputs, ",%.6g", value);
            std::fputc('\n', nn_inputs);
        }
        LearnerOutput learned = tick.update_learners(now_s);
        if (learned.learner_state) digest.add(&*learned.learner_state, sizeof(LearnerState));
        if (learned.vehicle_persist_data)
            digest.add(learned.vehicle_persist_data->data(), learned.vehicle_persist_data->size());
        if (dump) {
            std::fprintf(dump,
                         "%lu eng=%u act=%u block=%s torque=%d/%d curv=%a/%a desire=%u cruise=%u/%a/%a hud=%x "
                         "angle=%.1f driver=%d ff=%.4f fr=%.5f ts=%llu",
                         ticks, state.engaged, state.active, state.active_block, state.desired_torque,
                         state.apply_torque, state.desired_curvature, state.actual_curvature, state.desire,
                         state.cruise_active, state.cruise_max_speed_kph, state.cruise_command_speed_kph,
                         state.hud_flags, state.steering_angle_deg, state.driver_torque, result.feedforward,
                         result.friction_torque, static_cast<unsigned long long>(tick_ns));
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
            if (state.active && last_control.active) {
                ++both_active;
                const auto bit = [](const ControlState &c, uint32_t flag) { return (c.hud_flags & flag) != 0; };
                same_laneless += bit(state, kHudFlagLaneless) == bit(last_control, kHudFlagLaneless);
                same_nn += bit(state, kHudFlagNnFeedforward) == bit(last_control, kHudFlagNnFeedforward);
            }
        }
        ++ticks;
    }
    if (dump) std::fclose(dump);
    if (nn_inputs) std::fclose(nn_inputs);
    std::printf("ticks %lu sent %lu digest %016llx\n", ticks, sent, static_cast<unsigned long long>(digest.value));
    if (compared > 0)
        std::printf(
            "vs recorded ControlState (%lu ticks): engaged %.1f%% active %.1f%% desire %.1f%% |apply torque| %.2f\n",
            compared, 100.0 * same_engaged / compared, 100.0 * same_active / compared, 100.0 * same_desire / compared,
            torque_error / compared);
    if (both_active > 0) {
        const double laneless = 100.0 * same_laneless / both_active, nn = 100.0 * same_nn / both_active;
        std::printf("while both steer (%lu ticks): path source (Laneless) %.1f%% learned torque model %.1f%%\n",
                    both_active, laneless, nn);
        if (both_active >= 1000 && (laneless < 90.0 || nn < 90.0))
            std::fprintf(stderr,
                         "warning: the path source or the learned torque model differs from the recording in more "
                         "than 10%% of the steering ticks: the route's params/models copy may not be what the board "
                         "used (changed while driving?)\n");
    }
    return route_events.exit_code();
}
