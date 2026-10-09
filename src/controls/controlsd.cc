/* controlsd: K7 횡제어 프로세스(100 Hz). 로직은 ControlsTick(controls_tick.h)에 있고, 이 파일은
 * 공유 메모리 입출력, 파라미터 파일 감시, 학습 상태 파일 쓰기, 1초 통계만 맡는다. */
#include "common/background_writer.h"
#include "controls/controls_tick.h"
#include "common/ipc_channels.h"
#include "common/utils_file.h"
#include "common/utils_process.h"
#include "common/utils_time.h"

#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace {

volatile sig_atomic_t g_stop = 0;
volatile sig_atomic_t g_reload_params = 0;

void reload_signal_handler(int) {
  g_reload_params = 1;
}

bool open_when_ready(LatestChannel *channel, const char *topic,
                     size_t size, bool create) {
  while (!g_stop) {
    if (channel->open(topic, size, create)) return true;
    std::fprintf(stderr, "controlsd: waiting for %s\n", topic);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

bool open_when_ready(CanQueue *queue, const char *topic, bool create) {
  while (!g_stop) {
    if (queue->open(topic, kCanQueueSlots, create)) return true;
    std::fprintf(stderr, "controlsd: waiting for %s\n", topic);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

CanBatch make_send_batch(const std::vector<CanFrame> &frames) {
  return make_can_batch(frames, [](IpcCanFrame *dst, const CanFrame &src) {
    dst->address = src.address;
    dst->src = src.bus;
    dst->data_len = src.length;
    std::copy_n(src.data.begin(), src.length, dst->data);
  });
}

/* 1초 창의 루프 통계. 창이 끝나면 한 줄로 찍고 비운다. */
struct TickStats {
  unsigned can_frames = 0;
  unsigned generated_frames = 0;
  unsigned publish_errors = 0;
  unsigned send_queue_full = 0;
  unsigned stale_can_batches = 0;
  unsigned ticks = 0;
  unsigned misses = 0;
  double work_sum_us = 0.0;
  double work_max_us = 0.0;

  void account(double work_us, bool missed) {
    work_sum_us += work_us;
    work_max_us = std::max(work_max_us, work_us);
    ++ticks;
    if (missed) ++misses;
  }

  void log(double window_s, unsigned long long tx_depth, unsigned long long rx_depth,
           unsigned param_generation, const ControlsTick &tick) {
    const LateralControlResult &result = tick.result();
    const PandaGateOutput &panda = tick.panda();
    const LateralTarget &target = tick.target();
    const VehicleCanState &vehicle = tick.vehicle();
    const float road_bank_lat_accel = tick.controller().road_bank_lat_accel();
    const AdaptiveCruiseOutput &adaptive_cruise = tick.adaptive_cruise();
    const DepartureAlertInput &alert_input = tick.alert_input();
    std::fprintf(stderr,
                 "controlsd: hz=%.3f work_avg_us=%.1f work_max_us=%.1f "
                 "misses=%u can=%u generated=%u errors=%u txFull=%u rxStale=%u "
                 "queue=%llu/%llu params=%u "
                 "engaged=%u active=%u "
                 "panda=%u/%u plan=%u mpc=%u desire=%d "
                 "torque=%d/%d driver=%d angle=%.2f "
                 "curve=%.6f/%.6f curveVm=%.6f curveYaw=%.6f lat=%.2f bank=%.2f long=%.2f "
                 "error=%.6f pathY=%.3f "
                 "laneC=%.3f laneW=%.2f lane=%.2f/%.2f "
                 "lprob=%.2f/%.2f/%.2f lstd=%.2f/%.2f "
                 "cluster=%.1f wheel=%.1f cruise=%u max=%.1f cmd=%.1f target=%.1f "
                 "lead=%u/%.1f/%.1f button=%d pedal=%d/%d block=%s\n",
                 ticks / window_s, work_sum_us / std::max(1U, ticks), work_max_us,
                 misses, can_frames, generated_frames, publish_errors,
                 send_queue_full, stale_can_batches, tx_depth, rx_depth,
                 param_generation,
                 result.engaged ? 1 : 0, result.active ? 1 : 0,
                 panda.ready ? 1 : 0, panda.controls_allowed ? 1 : 0,
                 target.valid ? 1 : 0, target.mpc_solution_valid ? 1 : 0, target.desire,
                 result.desired_torque, result.apply_torque,
                 vehicle.driver_torque, vehicle.steering_angle_deg,
                 result.desired_curvature, result.actual_curvature,
                 result.actual_curvature_vm, result.actual_curvature_yaw,
                 vehicle.lat_accel_mps2, road_bank_lat_accel, vehicle.long_accel_mps2,
                 result.curvature_error, target.target_y_m,
                 0.5 * (target.lane_left_y_m + target.lane_right_y_m),
                 target.lane_width_m, target.lane_left_y_m, target.lane_right_y_m,
                 target.lane_left_prob, target.lane_right_prob, target.lane_d_prob,
                 target.lane_left_std, target.lane_right_std,
                 result.cluster_speed_kph, result.control_speed_kph,
                 adaptive_cruise.active ? 1U : 0U,
                 adaptive_cruise.maximum_speed_kph, adaptive_cruise.commanded_speed_kph,
                 adaptive_cruise.target_speed_kph,
                 adaptive_cruise.lead_valid ? 1U : 0U,
                 alert_input.lead_distance_m, alert_input.lead_relative_speed_mps,
                 adaptive_cruise.command_button,
                 vehicle.gas, vehicle.driver_override,
                 block_reason_name(result.active_block));
    *this = TickStats{};
  }
};

}  // namespace

int main() {
  install_stop_signal_handlers(&g_stop);
  signal(SIGHUP, reload_signal_handler);

  try {
    CanQueue can_sub;
    LatestChannel model_sub;
    LatestChannel panda_state_sub;
    CanQueue sendcan_pub;
    LatestChannel control_state_pub;
    LatestChannel learner_state_pub;
    LatestChannel localization_sub;
    bool localization_open = false;
    double next_localization_open_s = 0.0;
    if (!open_when_ready(&can_sub, kCanTopic, true) ||
        !open_when_ready(&model_sub, kModelStateTopic, sizeof(ModelState), false) ||
        !open_when_ready(&panda_state_sub, kPandaStateTopic,
                         sizeof(PandaState), true) ||
        !open_when_ready(&sendcan_pub, kSendCanTopic, true) ||
        !open_when_ready(&control_state_pub, kControlStateTopic,
                         sizeof(ControlState), true) ||
        !open_when_ready(&learner_state_pub, kLearnerStateTopic,
                         sizeof(LearnerState), true)) {
      return 0;
    }
    sendcan_pub.reset();

    const bool force_engaged = env_flag("EDGEPILOT_FORCE_ENGAGED", false);
    const ControlParamPaths param_paths{param_path("steering.json"), param_path("adaptive_cruise.json")};
    ControlParams params;
    std::string error;
    if (!load_control_params(param_paths, &params, &error)) throw std::runtime_error(error);
    std::fprintf(stderr, "controlsd: params steering=%s adaptive=%s %s\n", param_paths.steering.c_str(),
                 param_paths.cruise.c_str(), control_params_summary(params).c_str());
    /* paramsd·torqued. 사전값은 시작 때 파라미터로 고정한다. 복원이 거부된 저장은 상류처럼
     * 지운다(torqued는 깨진 캐시만, 튜닝이 바뀐 캐시는 둔다). */
    BackgroundWriter learner_store("controlsd: learner write");
    const std::string vehicle_learn_path = param_path("live_parameters.json");
    const std::string torque_learn_path = param_path("live_torque_parameters.bin");
    const std::string vehicle_learn_json = read_text_file(vehicle_learn_path);
    const std::string torque_learn_cache = read_text_file(torque_learn_path);
    LateralPlannerWorker lateral_planner(params.steering);
    ControlsTick tick(params, force_engaged, lateral_planner, vehicle_learn_json, torque_learn_cache,
                      static_cast<uint64_t>(monotonic_now_ns()));
    const LateralLearners &learners = tick.learners();
    if (learners.vehicle_restore_rejected()) learner_store.remove(vehicle_learn_path);
    if (learners.torque_restore_status() == TorqueRestore::Corrupt)
      learner_store.remove(torque_learn_path);
    std::fprintf(stderr,
                 "controlsd: learners paramsd=%s torqued=%s use_vehicle=%u use_torque=%u\n",
                 learners.vehicle_restored() ? "restored"
                 : vehicle_learn_json.empty() ? "fresh" : "rejected",
                 learners.torque_restore_status() == TorqueRestore::Restored        ? "restored"
                 : learners.torque_restore_status() == TorqueRestore::KeyMismatch   ? "key_mismatch"
                 : learners.torque_restore_status() == TorqueRestore::Corrupt       ? "corrupt"
                 : learners.torque_restore_status() == TorqueRestore::SourceChanged ? "source_changed"
                                                                                    : "fresh",
                 params.steering.use_live_vehicle_params ? 1U : 0U,
                 params.steering.use_live_torque_params ? 1U : 0U);
    TickStats stats;
    uint64_t model_seq = 0;
    uint64_t panda_state_seq = 0;

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto next_tick = start;
    auto log_start = start;
    ControlParamsWatcher param_watcher(param_paths, start);

    while (!g_stop) {
      next_tick += std::chrono::milliseconds(10);
      const auto work_start = Clock::now();
      const double now_s = std::chrono::duration<double>(work_start - start).count();
      const uint64_t can_now_ns = monotonic_now_ns();

      const bool reload_requested = g_reload_params != 0;
      if (reload_requested) g_reload_params = 0;
      if (const auto reloaded = param_watcher.poll(work_start, reload_requested, params)) {
        params = *reloaded;
        tick.apply_params(params);
      }

      CanBatch can_batch;
      while (can_sub.pop(&can_batch)) {
        if (!tick.on_can_batch(can_batch, can_now_ns, now_s)) {
          ++stats.stale_can_batches;
          continue;
        }
        stats.can_frames += std::min<uint32_t>(can_batch.count, kCanBatchMaxFrames);
      }
      ModelState model;
      uint64_t next_model_seq = model_seq;
      if (model_sub.read(&model, sizeof(model), &next_model_seq) &&
          next_model_seq != model_seq) {
        model_seq = next_model_seq;
        tick.on_model(model);
      }
      PandaState panda_state;
      uint64_t next_panda_state_seq = panda_state_seq;
      if (panda_state_sub.read(&panda_state, sizeof(panda_state),
                               &next_panda_state_seq) &&
          next_panda_state_seq != panda_state_seq) {
        panda_state_seq = next_panda_state_seq;
        tick.on_panda(panda_state);
      }
      /* locationd가 없어도 제어는 그대로라 붙을 때까지 1초마다 다시 열어 본다. */
      if (!localization_open && now_s >= next_localization_open_s) {
        localization_open = localization_sub.open(kLocalizationStateTopic, sizeof(LocalizationState), false);
        next_localization_open_s = now_s + 1.0;
      }
      LocalizationRead localization;
      localization.open = localization_open;
      if (localization_open) {
        localization.read = localization_sub.read(&localization.state, sizeof(localization.state));
        localization.read_ns = monotonic_now_ns();
      }
      tick.on_localization(localization, can_now_ns, now_s);

      /* IPC를 읽는 동안 새 모델/Panda 상태가 발행될 수 있으므로 freshness
       * 판정에는 공유 상태를 읽은 직후의 시간을 사용한다. */
      ControlState control_state = tick.step(now_s, monotonic_now_ns());
      control_state.timestamp_ns = monotonic_now_ns();
      if (!control_state_pub.publish(&control_state, sizeof(control_state))) {
        ++stats.publish_errors;
      }

      const LateralControlResult &result = tick.result();
      if (result.should_send && !result.frames.empty()) {
        const CanBatch send_batch = make_send_batch(result.frames);
        if (!sendcan_pub.push(send_batch)) {
          ++stats.publish_errors;
          ++stats.send_queue_full;
        } else {
          stats.generated_frames += static_cast<unsigned>(result.frames.size());
        }
      }

      /* 송신 뒤에 둬서 적합·직렬화 틱이 CAN 송신을 늦추지 않게 한다. */
      LearnerOutputs learned = tick.update_learners(now_s);
      if (learned.vehicle_json) learner_store.write(vehicle_learn_path, std::move(*learned.vehicle_json));
      if (learned.torque_cache) learner_store.write(torque_learn_path, std::move(*learned.torque_cache));
      if (learned.learner_state) {
        learned.learner_state->timestamp_ns = monotonic_now_ns();
        if (!learner_state_pub.publish(&*learned.learner_state, sizeof(LearnerState)))
          ++stats.publish_errors;
      }

      const auto work_end = Clock::now();
      stats.account(
          std::chrono::duration<double, std::micro>(work_end - work_start).count(),
          work_end > next_tick);
      if (work_end - log_start >= std::chrono::seconds(1)) {
        stats.log(std::chrono::duration<double>(work_end - log_start).count(),
                  static_cast<unsigned long long>(sendcan_pub.depth()),
                  static_cast<unsigned long long>(can_sub.depth()),
                  param_watcher.generation(), tick);
        log_start = work_end;
      }

      /* tick을 넘겼으면 밀린 만큼 따라잡지 않고 현재 시각으로 재동기화한다.
       * next_tick을 과거에 둔 채로 두면 다음 몇 번의 반복이 sleep 없이 연속
       * 실행되어 LKAS frame과 counter가 한꺼번에 몰려 나간다. */
      const auto tick_end = Clock::now();
      if (next_tick > tick_end) {
        std::this_thread::sleep_until(next_tick);
      } else {
        next_tick = tick_end;
      }
    }
    std::fprintf(stderr, "controlsd: stopping\n");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "controlsd error: %s\n", error.what());
    return 1;
  }
}
