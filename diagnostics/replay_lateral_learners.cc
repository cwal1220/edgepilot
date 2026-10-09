/* 녹화를 paramsd·torqued 학습기로 재생한다. CAN은 런타임과 같은 vehicle_can으로 풀고
 * controlsd와 같은 LateralLearners를 부른다(활성·보낸 토크는 ControlState, 운전자 개입은
 * 기록된 운전자 토크를 컨트롤러와 같게 디바운스). 참조 구현 대조용으로 입력·출력을 남길 수 있다.
 * 사용: replay_lateral_learners [옵션] <events.bin...>
 * --upstream-schedule: 조향각·속도를 상류처럼 20 Hz로만 관측(기본은 런타임과 같은 매 틱).
 * --fit-all: torqued 적합에 점 전부(기본은 상류처럼 2000점 무작위).
 * --inputs in.bin / --outputs out.csv / --torque-inputs tin.bin / --torque-outputs tout.csv:
 *   참조 구현 대조용 입력·출력.
 * --torque-cache c.bin: 있으면 복원하고 저장 틱마다 덮어쓴다(상류 LiveTorqueParameters).
 * --steering route/params/steering.json: 녹화 당시 파라미터(사전값·지연·토크 튜닝·입력 출처).
 *   없으면 코드 기본값이라 보드와 다를 수 있다.
 * --vehicle-json route/params/live_parameters.json: paramsd 저장값으로 시작한다(보드와 같게).
 * --source steering|locationd|esp: paramsd·torqued 요레이트·롤 출처. 기본 steering은
 *   use_locationd_learner_inputs를 따른다(controlsd와 같이 locationd가 0.5초 넘게 낡거나 필터가
 *   무효면 ESP12).
 * --metric-from N: 곡률 대조를 0부터 센 N번 이벤트 파일부터 센다(앞 파일로 학습을 수렴시킬 때).
 * 끝에 결합 직진 구간의 조향각 기반 곡률(학습값 차량 모델) − 요레이트 곡률 평균을 출처별로 출력한다. */
#include "controls/control_params.h"
#include "recording/event_log_reader.h"
#include "common/ipc_messages.h"
#include "learners/lateral_learners.h"
#include "learners/localizer_inputs.h"
#include "controls/lateral_torque.h"
#include "recorded_vehicle_can.h"
#include "recording/recording_format.h"
#include "common/utils_file.h"
#include "car/vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

struct InputRow {  // 참조 구현이 읽는 고정 레이아웃
  double t, angle_deg, speed, yaw, lat;
  int32_t fresh, gear, yaw_valid, lat_valid;
};
struct TorqueInputRow {
  double t, steer, speed, yaw, roll;
  int32_t fresh, lat_active, steer_override, pose_valid;
};

constexpr int kSteeringPressedMinCount = 5;  // lateral_controller.cc와 같다

}  // namespace

int main(int argc, char **argv) {
  std::string inputs_path, outputs_path, torque_inputs_path, torque_outputs_path, torque_cache_path;
  std::string steering_path, vehicle_json_path;
  bool fit_all = false, usage_error = false;
  std::string source = "steering";
  size_t metric_from_file = 0;  // 곡률 대조를 이 번호 파일부터만 센다(앞 파일은 학습 수렴용)
  VehicleParamsOptions options;
  std::vector<std::string> events;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--metric-from" && i + 1 < argc) {
      const char *text = argv[++i];
      char *end = nullptr;
      const unsigned long n = std::strtoul(text, &end, 10);
      if (*text < '0' || *text > '9' || *end != '\0') usage_error = true;
      metric_from_file = n;
      continue;
    }
    if (arg == "--upstream-schedule") options.car_state_every_tick = false;
    else if (arg == "--inputs" && i + 1 < argc) inputs_path = argv[++i];
    else if (arg == "--outputs" && i + 1 < argc) outputs_path = argv[++i];
    else if (arg == "--fit-all") fit_all = true;
    else if (arg == "--torque-inputs" && i + 1 < argc) torque_inputs_path = argv[++i];
    else if (arg == "--torque-outputs" && i + 1 < argc) torque_outputs_path = argv[++i];
    else if (arg == "--torque-cache" && i + 1 < argc) torque_cache_path = argv[++i];
    else if (arg == "--steering" && i + 1 < argc) steering_path = argv[++i];
    else if (arg == "--vehicle-json" && i + 1 < argc) vehicle_json_path = argv[++i];
    else if (arg == "--source" && i + 1 < argc) source = argv[++i];
    else if (arg.rfind("--", 0) == 0) usage_error = true;
    else events.push_back(arg);
  }
  if (source != "steering" && source != "locationd" && source != "esp") usage_error = true;
  if (events.empty() || usage_error) {
    std::fprintf(stderr, "usage: %s [--upstream-schedule] [--fit-all] [--inputs in.bin] [--outputs out.csv] "
                         "[--torque-inputs tin.bin] [--torque-outputs tout.csv] [--torque-cache c.bin] "
                         "[--steering steering.json] [--vehicle-json live_parameters.json] "
                         "[--source steering|locationd|esp] [--metric-from N] <events.bin...>\n", argv[0]);
    return 2;
  }

  SteeringParams sp;
  if (!steering_path.empty()) {
    std::string error;
    if (!load_steering_params_json(steering_path, &sp, &error)) {
      std::fprintf(stderr, "%s: %s\n", steering_path.c_str(), error.c_str());
      return 1;
    }
  }
  const ControlTiming timing;
  const double timeout_s = timing.vehicle_state_timeout_ms / 1000.0;
  const std::string cache = torque_cache_path.empty() ? std::string() : read_text_file(torque_cache_path);
  const std::string vehicle_json = vehicle_json_path.empty() ? std::string() : read_text_file(vehicle_json_path);
  const bool use_locationd = source == "steering" ? sp.use_locationd_learner_inputs : source == "locationd";
  std::printf("요레이트·롤 출처: %s\n", use_locationd ? "locationd(무효·낡으면 ESP12)" : "ESP12");
  LateralLearners learners(sp, vehicle_json, cache, 1, options);
  if (!vehicle_json.empty())
    std::printf("paramsd 저장값: %s\n", learners.vehicle_restored() ? "복원" : "거부");
  // 결합 직진 구간의 곡률 대조: 학습값 차량 모델(조향각) − 요레이트(바이어스 제거)
  TorqueController curvature_model;
  struct CurvatureSums {  // 출처(ESP12·locationd)별
    double diff = 0.0, angle = 0.0, yaw = 0.0, roll = 0.0;
    long n = 0;
  } curv[2];
  LocalizationState loc{};
  bool have_loc = false;
  if (!cache.empty())
    std::printf("torqued 캐시 %zu B: %s\n", cache.size(),
                learners.torque_restore_status() == TorqueRestore::Restored      ? "복원"
                : learners.torque_restore_status() == TorqueRestore::KeyMismatch ? "키 불일치"
                                                                                 : "손상");
  learners.torque_estimator().set_fit_all_points(fit_all);
  int pressed_counter = 0;

  std::FILE *in_file = inputs_path.empty() ? nullptr : std::fopen(inputs_path.c_str(), "wb");
  std::FILE *out_file = outputs_path.empty() ? nullptr : std::fopen(outputs_path.c_str(), "w");
  if (out_file)
    std::fprintf(out_file, "t,inputs_ok,valid,sensor_valid,steer_ratio,stiffness,roll_deg,"
                           "offset_avg_deg,offset_deg,sr_std,sf_std,yaw_bias\n");
  std::FILE *tin_file = torque_inputs_path.empty() ? nullptr : std::fopen(torque_inputs_path.c_str(), "wb");
  std::FILE *tout_file = torque_outputs_path.empty() ? nullptr : std::fopen(torque_outputs_path.c_str(), "w");
  if (tout_file)
    std::fprintf(tout_file, "t,inputs_ok,valid,factor_raw,offset_raw,friction_raw,factor,offset,"
                            "friction,points,cal_perc,decay\n");

  VehicleCanState vehicle{};
  std::vector<char> buf;
  /* ControlState 레코드 시각은 controlsd가 만든 시각이라 앞에 기록된 CAN 배치보다 이를 수
   * 있다. 실시간이면 now가 항상 수신 시각 뒤이므로, 재생에서도 본 CAN 중 최신을 하한으로 둔다. */
  double latest_can_s = 0.0;
  long publishes = 0, active_ticks = 0, ticks = 0;
  double first_t = -1.0, last_t = 0.0;
  size_t file_index = 0;
  for (const std::string &path : events) {
    const bool count_metric = file_index++ >= metric_from_file;
    EventLogReader reader(path);
    if (!reader.ok()) continue;
    EventRecordHeader rh{};
    while (reader.next(&rh, &buf)) {
      const double record_s = static_cast<double>(rh.timestamp_ns) * 1e-9;
      const double now_s = std::max(record_s, latest_can_s);
      if (rh.type == static_cast<uint16_t>(RecordType::CanRx)) {
        latest_can_s = std::max(latest_can_s, record_s);
        apply_recorded_can(buf.data(), rh.payload_size, now_s, &vehicle);
        continue;
      }
      if (rh.type == static_cast<uint16_t>(RecordType::Localization)) {
        std::memcpy(&loc, buf.data(), std::min(sizeof(loc), buf.size()));
        have_loc = true;
        continue;
      }
      // 보드가 실제로 쓴 조향 지연(torqued lag도 이 값을 따른다). 예전 기록은 0이라 건너뛴다
      if (rh.type == static_cast<uint16_t>(RecordType::LearnerState) && rh.payload_size >= sizeof(LearnerState)) {
        LearnerState ls{};
        std::memcpy(&ls, buf.data(), sizeof(ls));
        if (ls.plan_delay_s > 0.0f) learners.set_lateral_delay(ls.plan_delay_s);
        continue;
      }
      // 제어 틱(ControlState, 100 Hz)마다 한 번 넣는다. controlsd가 부를 자리와 같다.
      if (rh.type != static_cast<uint16_t>(RecordType::ControlState)) continue;
      ControlState cs{};
      std::memcpy(&cs, buf.data(), std::min(sizeof(cs), buf.size()));
      const bool pressed = std::abs(cs.driver_torque) > sp.steering_pressed_threshold;
      pressed_counter = std::clamp(pressed_counter + (pressed ? 1 : -1), 0,
                                   kSteeringPressedMinCount * 2 + 1);
      // controlsd처럼 매 틱 최신 locationd 상태로 판단한다(기록 시각 = 학습기 시계)
      if (have_loc) {
        const double sample_t_s = static_cast<double>(loc.timestamp_ns) * 1e-9;
        LocalizerSample sample;
        const bool use = localizer_sample_from(loc, use_locationd, now_s - sample_t_s, sample_t_s, &sample);
        learners.set_localizer(use, sample);
      }
      learners.update(vehicle, now_s, timeout_s, cs.active != 0, cs.apply_torque,
                      pressed_counter > kSteeringPressedMinCount);

      const VehicleParamsInput &in = learners.last_vehicle_input();
      const LiveLateralParams live = learners.live();
      if (count_metric && cs.active && in.inputs_fresh && in.yaw_rate_valid && in.speed_mps > 12.0 &&
          std::fabs(cs.desired_curvature) < 3e-4f && live.use_vehicle) {
        const double angle_curv = curvature_model.estimate_actual_curvature(
            static_cast<float>(in.speed_mps), static_cast<float>(in.steering_angle_deg), sp, 0.0f, false,
            live);
        const double yaw_curv = -in.yaw_rate_rad_s / in.speed_mps;  // 좌측 양수 → 우측 양수
        CurvatureSums &c = curv[in.localizer_roll_given ? 1 : 0];  // 이 틱에 넣은 요레이트의 출처
        c.diff += angle_curv - yaw_curv;
        c.angle += angle_curv;
        c.yaw += yaw_curv;
        c.roll += live.roll_rad;
        ++c.n;
      }
      if (in_file) {
        const InputRow row{in.t_s, in.steering_angle_deg, in.speed_mps, in.yaw_rate_rad_s,
                           in.lat_accel_mps2, in.inputs_fresh ? 1 : 0, in.gear,
                           in.yaw_rate_valid ? 1 : 0, in.lat_accel_valid ? 1 : 0};
        std::fwrite(&row, sizeof(row), 1, in_file);
      }
      if (first_t < 0.0) first_t = now_s;
      last_t = now_s;
      ++ticks;
      if (in.inputs_fresh && in.speed_mps > 1.0 && std::fabs(in.steering_angle_deg) < 45.0 &&
          in.gear != 7)
        ++active_ticks;
      if (learners.vehicle_published()) {
        ++publishes;
        const VehicleParams &p = learners.vehicle_params();
        if (out_file)
          std::fprintf(out_file, "%.4f,%d,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                       now_s - first_t, p.inputs_ok, p.valid, p.sensor_valid, p.steer_ratio,
                       p.stiffness_factor, p.roll_rad * 180.0 / 3.14159265358979323846,
                       p.angle_offset_average_deg, p.angle_offset_deg, p.steer_ratio_std,
                       p.stiffness_factor_std, learners.yaw_bias_rad_s());
      }

      const TorqueEstimatorInput &tin = learners.last_torque_input();
      if (tin_file) {
        const TorqueInputRow row{tin.t_s, tin.steer_torque, tin.speed_mps, tin.yaw_rate_rad_s,
                                 tin.roll_rad, tin.inputs_fresh ? 1 : 0, tin.lat_active ? 1 : 0,
                                 tin.steer_override ? 1 : 0, tin.pose_valid ? 1 : 0};
        std::fwrite(&row, sizeof(row), 1, tin_file);
      }
      if (learners.torque_persist_due() && !torque_cache_path.empty()) {
        std::ofstream f(torque_cache_path, std::ios::binary | std::ios::trunc);
        f.write(learners.torque_cache().data(),
                static_cast<std::streamsize>(learners.torque_cache().size()));
      }
      if (learners.torque_published() && tout_file) {
        const TorqueParams &q = learners.torque_params();
        std::fprintf(tout_file, "%.4f,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g\n",
                     now_s - first_t, q.inputs_ok, q.valid, q.lat_accel_factor_raw,
                     q.lat_accel_offset_raw, q.friction_raw, q.lat_accel_factor,
                     q.lat_accel_offset, q.friction, q.total_bucket_points, q.cal_perc, q.decay);
      }
    }
  }
  if (in_file) std::fclose(in_file);
  if (out_file) std::fclose(out_file);
  if (tin_file) std::fclose(tin_file);
  if (tout_file) std::fclose(tout_file);
  const VehicleParams &p = learners.vehicle_params();
  std::printf("%.0fs, 틱 %ld, 활성 %.0fs, 출력 %ld | SR %.3f (±%.3f) 강성 %.3f 영점 %+.3f도 "
              "(합계 %+.3f) 롤 %+.3f도 | valid %d | 자이로 바이어스 %+.4f deg/s\n",
              last_t - first_t, ticks, active_ticks / 100.0, publishes, p.steer_ratio,
              p.steer_ratio_std, p.stiffness_factor, p.angle_offset_average_deg,
              p.angle_offset_deg, p.roll_rad * 180.0 / 3.14159265358979323846, p.valid,
              learners.yaw_bias_rad_s() * 180.0 / 3.14159265358979323846);
  for (int k = 0; k < 2; ++k) {
    const CurvatureSums &c = curv[k];
    if (c.n == 0) continue;
    std::printf("결합 직진 %ld틱(%s 요레이트): 조향각 곡률 %+.3fe-4, 요레이트 곡률 %+.3fe-4, 차이 %+.3fe-4 1/m | "
                "평균 롤 %+.3f도\n",
                c.n, k ? "locationd" : "ESP12", c.angle / c.n * 1e4, c.yaw / c.n * 1e4, c.diff / c.n * 1e4,
                c.roll / c.n * 180.0 / 3.14159265358979323846);
  }
  const TorqueParams &q = learners.torque_params();
  std::printf("torqued: 점 %d (진행 %d%%) valid %d | 원시 배율 %.3f 절편 %+.3f 마찰 %.3f | "
              "필터 배율 %.3f 절편 %+.3f 마찰 %.3f (사전 %.3f/%.3f) decay %.1f\n",
              q.total_bucket_points, q.cal_perc, q.valid, q.lat_accel_factor_raw,
              q.lat_accel_offset_raw, q.friction_raw, q.lat_accel_factor, q.lat_accel_offset,
              q.friction, sp.torque_lat_accel_factor, sp.torque_friction, q.decay);
  for (int b = 0; b < TorqueEstimator::kBuckets; ++b)
    std::printf("%s%d", b ? " " : "  버킷 ", learners.torque_estimator().bucket_size(b));
  std::printf("\n");
  return 0;
}
