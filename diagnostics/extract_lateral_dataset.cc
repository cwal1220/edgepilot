/* v2/v3 녹화에서 횡제어 분석/학습용 CSV를 뽑는다. CAN 디코드와 부호 규약은
 * 런타임과 같은 vehicle_can을 그대로 쓴다.
 * 사용: extract_lateral_dataset <out.csv> <events...> */
#include "common/utils_math.h"
#include "common/utils_time.h"
#include "common/ipc_messages.h"
#include "recording/recorded_model_state.h"
#include "recorded_vehicle_can.h"
#include "recording/recording_format.h"
#include "controls/control_params.h"
#include "controls/lateral_torque.h"
#include "recording/event_log_reader.h"
#include "car/vehicle_can.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

constexpr float kGravity = 9.8f;
constexpr double kVehicleTimeoutS = 0.5;  // ControlTiming 기본값
// openpilot NNFF future_times. 플랜에서 이 시점의 값을 뽑는다.
constexpr float kFutureTimes[] = {0.3f, 0.6f, 1.0f, 1.5f};
constexpr int kFutureCount = 4;
// 요각 미분 폭(초). 플랜 격자가 근거리에서 촘촘해 이 정도면 안정적이다.
constexpr float kYawDiffHalfWindowS = 0.15f;
// 편경사 필터 시정수. lateral_controller.cc와 같다.
constexpr float kBankRcS = 2.0f;
constexpr float kBankMinSpeedMps = 8.0f;

// 플랜 격자(model_t)에서 t 시점 값을 선형 보간한다.
float interp_plan(float t, const float *grid, const float *values) {
  if (t <= grid[0]) return values[0];
  for (int i = 1; i < kTrajectorySize; ++i) {
    if (t <= grid[i]) {
      const float span = grid[i] - grid[i - 1];
      if (span < 1e-6f) return values[i];
      return values[i - 1] + (t - grid[i - 1]) / span * (values[i] - values[i - 1]);
    }
  }
  return values[kTrajectorySize - 1];
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <out.csv> <events.bin...>\n", argv[0]);
    return 2;
  }

  std::FILE *out = std::fopen(argv[1], "w");
  if (!out) {
    std::fprintf(stderr, "cannot open %s\n", argv[1]);
    return 1;
  }
  std::fprintf(out,
               "t,v,active,desire,block,angle,driver_tq,apply_tq,tq_norm,des_tq_norm,"
               "lat,lat_valid,yaw,long,bank,roll,"
               "des_curv,act_curv,des_la,"
               "la_p03,la_p06,la_p10,la_p15,"
               "roll_p03,roll_p06,roll_p10,roll_p15,model_age\n");

  const SteeringParams params;  // steer_max 384 (실차 설정과 동일)
  const float torque_scale = static_cast<float>(kTorqueOutputSign) / static_cast<float>(params.steer_max);

  VehicleCanState vehicle{};
  ModelState model{};
  bool have_model = false;
  double model_time_s = -1.0;
  float bank = 0.0f;
  bool bank_init = false;
  double last_row_s = -1.0;
  double route_start_s = -1.0;
  uint64_t rows = 0;

  std::vector<char> buf;
  for (int arg = 2; arg < argc; ++arg) {
    EventLogReader reader(argv[arg]);
    if (!reader.ok()) {
      std::fprintf(stderr, "skip %s: not an event file\n", argv[arg]);
      continue;
    }

    EventRecordHeader rh{};
    while (reader.next(&rh, &buf)) {

      const double now_s = static_cast<double>(rh.timestamp_ns) * 1e-9;
      if (route_start_s < 0.0) route_start_s = now_s;

      if (rh.type == static_cast<uint16_t>(RecordType::CanRx)) {
        apply_recorded_can(buf.data(), rh.payload_size, now_s, &vehicle);
        continue;
      }

      if (rh.type == static_cast<uint16_t>(RecordType::ModelState) &&
          decode_recorded_model_state(buf.data(), rh.payload_size, reader.version(), &model)) {
        have_model = model.valid != 0;
        model_time_s = now_s;
        continue;
      }

      if (rh.type != static_cast<uint16_t>(RecordType::ControlState) ||
          rh.payload_size < sizeof(ControlState)) {
        continue;
      }
      ControlState cs{};
      std::memcpy(&cs, buf.data(), sizeof(cs));

      const float v = vehicle_speed_kph(vehicle, now_s, kVehicleTimeoutS) / 3.6f;
      const bool yaw_fresh =
          signal_time_fresh(vehicle.esp12_time_s, now_s, kVehicleTimeoutS) &&
          vehicle.yaw_rate_valid;

      /* 편경사: lateral_controller와 같은 식/게이트. 다만 이 도구는 100 Hz가
       * 아니라 ControlState 주기로 돌아 alpha를 dt로 계산한다. */
      const double dt = last_row_s < 0.0 ? 0.0 : now_s - last_row_s;
      last_row_s = now_s;
      if (yaw_fresh && vehicle.lat_accel_valid &&
          std::isfinite(vehicle.lat_accel_mps2) && v > kBankMinSpeedMps &&
          dt > 0.0 && dt < 0.5) {
        const float sample = clamp_float(
            vehicle.lat_accel_mps2 + vehicle.yaw_rate_rad_s * v, -2.0f, 2.0f);
        if (!bank_init) {
          bank = sample;
          bank_init = true;
        } else {
          const float alpha = static_cast<float>(dt / (kBankRcS + dt));
          bank += alpha * (sample - bank);
        }
      }
      const float roll = bank / kGravity;

      float future_la[kFutureCount] = {};
      float future_roll[kFutureCount] = {};
      float model_age = -1.0f;
      if (have_model && model_time_s >= 0.0) {
        model_age = static_cast<float>(now_s - model_time_s);
        float grid[kTrajectorySize];
        float yaw[kTrajectorySize];
        float plan_roll[kTrajectorySize];
        const bool grid_valid = model.model_t[kTrajectorySize - 1] > 0.0f;
        /* v5 녹화에는 plan orientation이 없다. 요각은 plan 위치의 중앙차분으로,
         * 롤은 0으로 대신한다(전진 0.5 m 미만이면 heading이 무의미해 0). */
        for (int i = 0; i < kTrajectorySize; ++i) {
          grid[i] = grid_valid ? model.model_t[i] : model_t_idx(i);
          const int prev = std::max(0, i - 1), next = std::min(kTrajectorySize - 1, i + 1);
          const float dx = model.plan[next].x - model.plan[prev].x;
          const float dy = model.plan[next].y - model.plan[prev].y;
          yaw[i] = std::fabs(dx) >= 0.5f ? std::atan2(dy, dx) : 0.0f;
          plan_roll[i] = 0.0f;
        }
        for (int i = 0; i < kFutureCount; ++i) {
          const float t = kFutureTimes[i];
          /* 미래 횡가속: openpilot은 플랜 acceleration.y를 쓰지만 녹화에는
           * 없다. 요각 미분으로 대체한다. a_y = v * dψ/dt. */
          const float ahead = interp_plan(t + kYawDiffHalfWindowS, grid, yaw);
          const float behind = interp_plan(
              std::max(0.0f, t - kYawDiffHalfWindowS), grid, yaw);
          const float span = (t + kYawDiffHalfWindowS) -
                             std::max(0.0f, t - kYawDiffHalfWindowS);
          future_la[i] = span > 1e-6f ? v * (ahead - behind) / span : 0.0f;
          future_roll[i] = interp_plan(t, grid, plan_roll) + roll;
        }
      }

      const float des_la = cs.desired_curvature * v * v;
      std::fprintf(out,
                   "%.3f,%.3f,%u,%u,%s,%.2f,%d,%d,%.5f,%.5f,"
                   "%.3f,%d,%.5f,%.3f,%.4f,%.5f,"
                   "%.7f,%.7f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.5f,%.5f,%.5f,%.5f,%.3f\n",
                   now_s - route_start_s, v, cs.active, cs.desire,
                   cs.active_block[0] ? cs.active_block : "-",
                   cs.steering_angle_deg, cs.driver_torque, cs.apply_torque,
                   static_cast<float>(cs.apply_torque) * torque_scale,
                   static_cast<float>(cs.desired_torque) * torque_scale,
                   vehicle.lat_accel_mps2, vehicle.lat_accel_valid ? 1 : 0,
                   vehicle.yaw_rate_rad_s, vehicle.long_accel_mps2, bank, roll,
                   cs.desired_curvature, cs.actual_curvature, des_la,
                   future_la[0], future_la[1], future_la[2], future_la[3],
                   future_roll[0], future_roll[1], future_roll[2], future_roll[3],
                   model_age);
      ++rows;
    }
    /* 8-19 route처럼 tmpfs가 차서 끊긴 파일은 0으로 채워진 구간이 남는다.
     * 리더는 재동기화를 시도하지 않고 그 파일을 거기서 끝낸다. */
    if (reader.truncated())
      std::fprintf(stderr, "%s: truncated at %lld bytes (type=%u len=%u)\n", argv[arg],
                   reader.truncated_at(), rh.type, rh.payload_size);
  }

  std::fclose(out);
  std::fprintf(stderr, "%s: %llu rows\n", argv[1],
               static_cast<unsigned long long>(rows));
  return 0;
}
