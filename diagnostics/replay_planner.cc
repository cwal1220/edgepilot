// 녹화된 ModelState/ControlState로 LateralPlanner를 재실행한다(기록 시각 순, RouteEvents).
// 녹화된 인지 결과에 대해 플래너가 무엇을 요구했는지 오프라인으로 재현한다.
// 사용: replay_planner [--laneless|--lane] [--exact] [--vehicle] [--steering s.json] [--lane-filter W]
//                      <out.csv> <events.bin...>
//   --exact     LateralTarget의 모든 필드와 지연 보정 곡률을 %a(비트 그대로)로 쓴다. 리팩토링 전후
//               출력이 비트 단위로 같은지 비교할 때 쓴다.
// 지연 보정 곡률(des_replay)은 controlsd처럼 plan 나이를 촬영 시각부터 재고(modeld 지연 포함), 녹화된
// LearnerState의 경로 지연(lagd를 쓰고 있었으면 그 추정)과 롤을 쓴다. 경로 지연이 없는 예전 녹화는
// steer_actuator_delay를 쓴다.
//   --vehicle   녹화한 CAN(깜빡이·운전자 토크·사각지대)과 ControlState의 active를 플래너에 준다.
//               없으면 깜빡이·개입 없음, active로 돌려 차선 변경이 돌지 않는다.
//   --steering  조향 파라미터 파일(예: route의 params 스냅샷). 없으면 기본값.
//   --laneless, --lane     파라미터 파일과 상관없이 Laneless 또는 Lane 모드로 돌린다.
//   --lane-filter  차선선 시간 필터의 새 측정 가중(planning/lane_line_filter, 1이면 끈다).
#include "car/vehicle_can.h"
#include "cli_args.h"
#include "common/ipc_messages.h"
#include "controls/control_params.h"
#include "controls/drive_helpers.h"
#include "controls/steering_pressed.h"
#include "planning/lateral_planner.h"
#include "recorded_vehicle_can.h"
#include "recording/event_log_reader.h"
#include "recording/recording_format.h"
#include "route_events.h"

#include <cstdio>
#include <string>
#include <vector>

namespace {

// 지연을 반영한 plan 곡률을 클립한다. plan이 무효면 0. 컨트롤러는 그 사이에 목표 혼합(50 ms)과 85도 묶음도
// 하므로 이 값은 그 둘을 뺀 근사다.
float lag_adjusted_desired_curvature(const LateralTarget &target, float speed_mps, float plan_age_s,
                                     float steer_actuator_delay_s, float prev_curvature, float roll_rad) {
    if (!target.valid) return 0.0f;
    return clip_curvature(speed_mps, prev_curvature,
                          lag_adjusted_curvature(target, speed_mps, plan_age_s, steer_actuator_delay_s), roll_rad);
}

void write_exact_header(std::FILE *out) {
    std::fprintf(out,
                 "t,valid,capture_ns,mpc_valid,laneless,target_y,heading,curvature,desire,"
                 "lc_state,lc_dir");
    for (int i = 0; i < kLateralControlN; ++i) std::fprintf(out, ",psi%d", i);
    for (int i = 0; i < kLateralControlN; ++i) std::fprintf(out, ",curv%d", i);
    std::fprintf(out, ",lane_l,lane_r,lane_w,prob_l,prob_r,std_l,std_r,d_prob,des\n");
}

void write_exact_row(std::FILE *out, uint64_t timestamp_ns, const LateralTarget &t, float des) {
    std::fprintf(out, "%llu,%d,%llu,%d,%d,%a,%a,%a,%d,%d,%d", static_cast<unsigned long long>(timestamp_ns),
                 t.valid ? 1 : 0, static_cast<unsigned long long>(t.capture_timestamp_ns), t.mpc_solution_valid ? 1 : 0,
                 t.laneless_mode ? 1 : 0, t.target_y_m, t.psis[0], t.curvatures[0], static_cast<int>(t.desire),
                 static_cast<int>(t.lane_change_state), t.lane_change_direction);
    for (float v : t.psis) std::fprintf(out, ",%a", v);
    for (float v : t.curvatures) std::fprintf(out, ",%a", v);
    std::fprintf(out, ",%a,%a,%a,%a,%a,%a,%a,%a,%a\n", t.lane_left_y_m, t.lane_right_y_m, t.lane_width_m,
                 t.lane_left_prob, t.lane_right_prob, t.lane_left_std, t.lane_right_std, t.lane_d_prob, des);
}

}  // namespace

int main(int argc, char **argv) {
    SteeringParams steering;
    bool force_laneless = false, force_lane = false, exact = false, use_vehicle = false;
    std::string steering_path;
    float lane_filter = 0.0f;  // 0이면 코드 기본값
    std::vector<const char *> positional;
    ArgReader args(argc, argv);
    while (args.next()) {
        if (args.is("--laneless"))
            force_laneless = true;
        else if (args.is("--lane"))
            force_lane = true;
        else if (args.is("--exact"))
            exact = true;
        else if (args.is("--vehicle"))
            use_vehicle = true;
        else if (args.is("--steering"))
            steering_path = args.value();
        else if (args.is("--lane-filter"))
            args.read(&lane_filter, 0.0, 1.0);
        else if (args.is_option())
            args.fail();
        else
            positional.push_back(args.arg());
    }
    if (args.error() || (force_laneless && force_lane) || positional.size() < 2) {
        std::fprintf(stderr,
                     "usage: %s [--laneless|--lane] [--exact] [--vehicle] [--steering s.json] "
                     "[--lane-filter W] <out.csv> <events.bin...>\n",
                     argv[0]);
        return 2;
    }
    std::string error;
    if (!steering_path.empty() && !load_steering_params(steering_path, &steering, &error)) {
        std::fprintf(stderr, "%s: %s\n", steering_path.c_str(), error.c_str());
        return 1;
    }
    if (force_laneless) steering.laneless_mode = true;
    if (force_lane) steering.laneless_mode = false;
    LateralPlanner planner(steering);
    if (lane_filter > 0.0f) planner.set_lane_filter_weight(lane_filter);

    VehicleCanState vehicle{};  // --vehicle이 아니면 깜빡이·개입 없음
    std::FILE *out = std::fopen(positional[0], "w");
    if (out == nullptr) {
        std::fprintf(stderr, "cannot open %s\n", positional[0]);
        return 1;
    }
    if (exact) {
        write_exact_header(out);
    } else {
        std::fprintf(out,
                     "t,v_kph,measured,des_rec,des_replay,target_curv,target_y,"
                     "lane_l,lane_r,prob_l,prob_r,d_prob,laneless,lane_w,mpc_valid,heading0,heading_target\n");
    }

    float v_kph = 0.0f, measured = 0.0f, des_rec = 0.0f, prev_des = 0.0f;
    float plan_delay_s = 0.0f, roll_rad = 0.0f;  // 마지막 LearnerState: controlsd가 실제로 쓴 경로 지연과 롤
    bool have_cs = false, active = true, steering_pressed = false;
    SteeringPressedFilter pressed_filter;  // controlsd처럼 녹화된 운전자 토크를 거른다
    RouteEvents events(std::vector<std::string>(positional.begin() + 1, positional.end()));
    TimeOrderedEvents::Record record;
    while (events.next(&record)) {
        const EventRecordHeader &rh = record.header;
        const RecordType type = record_type(rh);
        const std::vector<char> &buf = record.payload;
        ControlState cs;
        LearnerState ls;
        if (use_vehicle && type == RecordType::CanRx) {
            apply_recorded_can(buf.data(), rh.payload_size, record_time_s(rh), &vehicle);
        } else if (type == RecordType::ControlState && read_record(buf, &cs)) {
            v_kph = cs.ego_speed_kph > 0.0f ? cs.ego_speed_kph : cs.cluster_speed_kph;
            measured = cs.actual_curvature;
            des_rec = cs.desired_curvature;
            if (use_vehicle) {
                active = recorded_lat_active(cs);
                steering_pressed = pressed_filter.update(cs.driver_torque, steering.steering_pressed_threshold);
            }
            have_cs = true;
        } else if (type == RecordType::LearnerState && read_record(buf, &ls)) {
            plan_delay_s = ls.plan_delay_s;
            roll_rad = (ls.flags & kLearnerUseVehicle) != 0 ? ls.roll_rad : 0.0f;
        } else if (type == RecordType::ModelState) {
            if (!have_cs) continue;
            ModelState ms{};
            if (!read_record(buf, &ms)) continue;
            LateralPlannerInput planner_in;
            planner_in.v_ego = v_kph / 3.6f;
            planner_in.measured_curvature = measured;
            planner_in.lat_active = active;
            planner_in.steering_pressed = steering_pressed;
            const float v = planner_in.v_ego;
            LateralTarget t = planner.update(ms, vehicle, planner_in);
            // 곡률 보정은 컨트롤러와 같은 100 Hz 틱으로 돌린다. 틱당 변화율 제한이
            // 있어 모델 주기로 한 번만 부르면 5배 과하게 걸린다. plan 나이는 촬영 시각부터
            // 재어 틱마다 늘어난다.
            const float capture_age_s =
                ms.model_timestamp_ns > ms.capture_timestamp_ns && ms.capture_timestamp_ns != 0
                    ? static_cast<float>(static_cast<double>(ms.model_timestamp_ns - ms.capture_timestamp_ns) * 1e-9)
                    : 0.0f;
            const float delay_s = plan_delay_s > 0.0f ? plan_delay_s : steering.steer_actuator_delay;
            float des = prev_des;
            for (int tick = 0; tick < 5; ++tick) {
                des = lag_adjusted_desired_curvature(t, v, capture_age_s + 0.01f * tick, delay_s, prev_des, roll_rad);
                if (t.valid) prev_des = des;
            }
            if (exact) {
                write_exact_row(out, rh.timestamp_ns, t, des);
                continue;
            }
            std::fprintf(out,
                         "%.3f,%.1f,%.6f,%.6f,%.6f,%.6f,%.3f,"
                         "%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.2f,%d,%.4f,%.4f\n",
                         record_time_s(rh), v_kph, measured, des_rec, des, t.curvatures[0], t.target_y_m,
                         t.lane_left_y_m, t.lane_right_y_m, t.lane_left_prob, t.lane_right_prob, t.lane_d_prob,
                         t.laneless_mode ? 1 : 0, t.lane_width_m, t.mpc_solution_valid ? 1 : 0, t.psis[0],
                         t.psis[kLateralControlN - 1]);
        }
    }
    std::fclose(out);
    return events.exit_code();
}
