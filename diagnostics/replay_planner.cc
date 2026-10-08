/* 녹화된 ModelState/ControlState로 LateralPlanner를 재실행한다.
 * 녹화된 인지 결과에 대해 플래너가 무엇을 요구했는지 오프라인으로 재현한다.
 * 사용: replay_planner [--laneless|--lane] [--exact] [--vehicle] [--steering s.json]
 *                      <out.csv> <events.bin...>
 *   --exact     LateralTarget의 모든 필드와 지연 보정 곡률을 %a(비트 그대로)로 쓴다. 리팩토링 전후
 *               출력이 비트 단위로 같은지 비교할 때 쓴다.
 *   --vehicle   녹화한 CAN(깜빡이·운전자 토크·사각지대)과 ControlState의 active를 플래너에 준다.
 *               없으면 깜빡이·개입 없음, active로 돌려 차선 변경이 돌지 않는다.
 *   --steering  조향 파라미터 파일(예: route의 params 스냅샷). 없으면 기본값.
 *   --laneless, --lane     파라미터 파일과 상관없이 Laneless 또는 Lane 모드로 돌린다. */
#include "controls/control_params.h"
#include "recording/event_log_reader.h"
#include "common/ipc_messages.h"
#include "controls/lateral_controller.h"
#include "planning/lateral_planner.h"
#include "recording/recorded_model_state.h"
#include "recorded_vehicle_can.h"
#include "recording/recording_format.h"
#include "car/vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

void write_exact_header(std::FILE *out) {
  std::fprintf(out, "t,valid,capture_ns,mpc_valid,laneless,target_y,heading,curvature,desire,"
                    "lc_state,lc_dir");
  for (int i = 0; i < kLateralControlN; ++i) std::fprintf(out, ",psi%d", i);
  for (int i = 0; i < kLateralControlN; ++i) std::fprintf(out, ",curv%d", i);
  std::fprintf(out, ",lane_l,lane_r,lane_w,prob_l,prob_r,std_l,std_r,d_prob,des\n");
}

void write_exact_row(std::FILE *out, uint64_t timestamp_ns, const LateralTarget &t, float des) {
  std::fprintf(out, "%llu,%d,%llu,%d,%d,%a,%a,%a,%d,%d,%d", static_cast<unsigned long long>(timestamp_ns),
               t.valid ? 1 : 0, static_cast<unsigned long long>(t.capture_timestamp_ns),
               t.mpc_solution_valid ? 1 : 0, t.laneless_mode ? 1 : 0, t.target_y_m, t.heading_rad, t.curvature,
               t.desire, t.lane_change_state, t.lane_change_direction);
  for (float v : t.psis) std::fprintf(out, ",%a", v);
  for (float v : t.curvatures) std::fprintf(out, ",%a", v);
  std::fprintf(out, ",%a,%a,%a,%a,%a,%a,%a,%a,%a\n", t.lane_left_y_m, t.lane_right_y_m, t.lane_width_m,
               t.lane_left_prob, t.lane_right_prob, t.lane_left_std, t.lane_right_std, t.lane_d_prob, des);
}

}  // namespace

int main(int argc, char **argv) {
  SteeringParams steering;
  bool force_laneless = false, force_lane = false, exact = false, use_vehicle = false, usage_error = false;
  std::string steering_path;
  std::vector<const char *> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--laneless") force_laneless = true;
    else if (arg == "--lane") force_lane = true;
    else if (arg == "--exact") exact = true;
    else if (arg == "--vehicle") use_vehicle = true;
    else if (arg == "--steering" && i + 1 < argc) steering_path = argv[++i];
    else if (arg.rfind("--", 0) == 0) usage_error = true;
    else positional.push_back(argv[i]);
  }
  if (usage_error || (force_laneless && force_lane) || positional.size() < 2) {
    std::fprintf(stderr, "usage: %s [--laneless|--lane] [--exact] [--vehicle] [--steering s.json] "
                         "<out.csv> <events.bin...>\n", argv[0]);
    return 2;
  }
  std::string error;
  if (!steering_path.empty() && !load_steering_params_json(steering_path, &steering, &error)) {
    std::fprintf(stderr, "%s: %s\n", steering_path.c_str(), error.c_str());
    return 1;
  }
  if (force_laneless) steering.laneless_mode = true;
  if (force_lane) steering.laneless_mode = false;
  LateralPlanner planner(steering);

  VehicleCanState vehicle{};   // --vehicle이 아니면 깜빡이·개입 없음
  std::FILE *out = std::fopen(positional[0], "w");
  if (out == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", positional[0]);
    return 1;
  }
  if (exact) {
    write_exact_header(out);
  } else {
    std::fprintf(out, "t,v_kph,measured,des_rec,des_replay,target_curv,target_y,"
                      "lane_l,lane_r,prob_l,prob_r,d_prob,laneless,lane_w,mpc_valid,heading0,heading_target\n");
  }

  float v_kph = 0.0f, measured = 0.0f, des_rec = 0.0f, prev_des = 0.0f;
  bool have_cs = false, active = true;
  for (size_t a = 1; a < positional.size(); ++a) {
    EventLogReader reader(positional[a]);
    if (!reader.ok()) continue;
    EventRecordHeader rh{};
    std::vector<char> buf;
    while (reader.next(&rh, &buf)) {
      if (use_vehicle && rh.type == static_cast<uint16_t>(RecordType::CanRx)) {
        apply_recorded_can(buf.data(), rh.payload_size, static_cast<double>(rh.timestamp_ns) * 1e-9, &vehicle,
                           true);
      } else if (rh.type == static_cast<uint16_t>(RecordType::ControlState) &&
                 rh.payload_size >= sizeof(ControlState)) {
        ControlState cs{};
        std::memcpy(&cs, buf.data(), sizeof(cs));
        v_kph = cs.ego_speed_kph > 0.0f ? cs.ego_speed_kph : cs.cluster_speed_kph;
        measured = cs.actual_curvature;
        des_rec = cs.desired_curvature;
        if (use_vehicle) active = cs.active != 0;
        have_cs = true;
      } else if (rh.type == static_cast<uint16_t>(RecordType::ModelState)) {
        if (!have_cs) continue;
        ModelState ms{};
        if (!decode_recorded_model_state(buf.data(), rh.payload_size, reader.version(), &ms)) continue;
        const float v = v_kph / 3.6f;
        LateralTarget t = planner.update(ms, vehicle, v, measured, active);
        /* 곡률 보정은 컨트롤러와 같은 100 Hz 틱으로 돌린다. 틱당 변화율 제한이
         * 있어 모델 주기로 한 번만 부르면 5배 과하게 걸린다. plan 나이는 틱마다
         * 늘어난다. */
        float des = prev_des;
        for (int tick = 0; tick < 5; ++tick) {
          des = lag_adjusted_desired_curvature(t, v, 0.01f * tick,
                                               steering.steer_actuator_delay, prev_des);
          if (t.valid) prev_des = des;
        }
        if (exact) {
          write_exact_row(out, rh.timestamp_ns, t, des);
          continue;
        }
        std::fprintf(out, "%.3f,%.1f,%.6f,%.6f,%.6f,%.6f,%.3f,"
                          "%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.2f,%d,%.4f,%.4f\n",
                     rh.timestamp_ns * 1e-9, v_kph, measured, des_rec, des,
                     t.curvature, t.target_y_m,
                     t.lane_left_y_m, t.lane_right_y_m,
                     t.lane_left_prob, t.lane_right_prob, t.lane_d_prob,
                     t.laneless_mode ? 1 : 0, t.lane_width_m,
                     t.mpc_solution_valid ? 1 : 0, t.heading_rad, t.psis[kLateralControlN - 1]);
      }
    }
  }
  std::fclose(out);
  return 0;
}
