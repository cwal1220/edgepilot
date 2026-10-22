// 녹화를 paramsd·torqued 학습기로 재생한다. CAN은 런타임과 같은 vehicle_can으로 풀고
// controlsd와 같은 LateralLearners를 부른다(활성·보낸 토크는 ControlState, 운전자 개입은
// 기록된 운전자 토크를 컨트롤러와 같게 디바운스). 게시한 학습값을 CSV로 남길 수 있다.
// 사용: replay_lateral_learners [옵션] <events.bin...>
// --upstream-schedule: 조향각·속도를 상류처럼 20 Hz로만 관측(기본은 런타임과 같은 매 틱).
// --outputs out.csv / --torque-outputs tout.csv: paramsd·torqued가 게시한 값.
// --torque-cache c.bin: 있으면 복원하고 저장 틱마다 덮어쓴다(상류 LiveTorqueParameters).
// --steering route/params/steering.json: 녹화 당시 파라미터(사전값·지연·토크 튜닝·입력 출처).
//   없으면 코드 기본값이라 보드와 다를 수 있다.
// --vehicle-json route/params/live_parameters.json: paramsd 저장값으로 시작한다(보드와 같게).
// --source steering|locationd|esp: paramsd·torqued 요레이트·롤 출처. 기본 steering은
//   use_locationd_learner_inputs를 따른다(controlsd와 같이 locationd가 0.5초 넘게 낡거나 필터가
//   무효면 ESP12).
// --metric-from N: 곡률 대조를 0부터 센 N번 이벤트 파일부터 센다(앞 파일로 학습을 수렴시킬 때).
// 끝에 결합 직진 구간의 조향각 기반 곡률(학습값 차량 모델) − 요레이트 곡률 평균을 출처별로 출력한다.
#include "car/speed_filter.h"
#include "car/vehicle_can.h"
#include "cli_args.h"
#include "common/ipc_messages.h"
#include "common/utils_file.h"
#include "common/utils_math.h"  // kPi
#include "common/utils_time.h"
#include "controls/control_params.h"
#include "controls/lateral_torque.h"
#include "controls/steering_pressed.h"
#include "learners/lateral_learners.h"
#include "learners/localizer_inputs.h"
#include "recorded_vehicle_can.h"
#include "recording/event_log_reader.h"
#include "recording/recording_format.h"
#include "route_events.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    std::string outputs_path, torque_outputs_path, torque_cache_path;
    std::string steering_path, vehicle_json_path;
    std::string source = "steering";
    long metric_from_file = 0;  // 곡률 대조를 이 번호 파일부터만 센다(앞 파일은 학습 수렴용)
    VehicleParamsOptions options;
    std::vector<std::string> events;
    ArgReader args(argc, argv);
    while (args.next()) {
        if (args.is("--metric-from"))
            args.read(&metric_from_file, 0);
        else if (args.is("--upstream-schedule"))
            options.car_state_every_tick = false;
        else if (args.is("--outputs"))
            outputs_path = args.value();
        else if (args.is("--torque-outputs"))
            torque_outputs_path = args.value();
        else if (args.is("--torque-cache"))
            torque_cache_path = args.value();
        else if (args.is("--steering"))
            steering_path = args.value();
        else if (args.is("--vehicle-json"))
            vehicle_json_path = args.value();
        else if (args.is("--source"))
            source = args.value();
        else if (args.is_option())
            args.fail();
        else
            events.push_back(args.arg());
    }
    if (source != "steering" && source != "locationd" && source != "esp") args.fail();
    if (events.empty() || args.error()) {
        std::fprintf(stderr,
                     "usage: %s [--upstream-schedule] [--outputs out.csv] [--torque-outputs tout.csv] "
                     "[--torque-cache c.bin] "
                     "[--steering steering.json] [--vehicle-json live_parameters.json] "
                     "[--source steering|locationd|esp] [--metric-from N] <events.bin...>\n",
                     argv[0]);
        return 2;
    }

    SteeringParams sp;
    if (!steering_path.empty()) {
        std::string error;
        if (!load_steering_params(steering_path, &sp, &error)) {
            std::fprintf(stderr, "%s: %s\n", steering_path.c_str(), error.c_str());
            return 1;
        }
    }
    const std::string cache = torque_cache_path.empty() ? std::string() : read_text_file(torque_cache_path);
    const std::string vehicle_json = vehicle_json_path.empty() ? std::string() : read_text_file(vehicle_json_path);
    const bool use_locationd = source == "steering" ? sp.use_locationd_learner_inputs : source == "locationd";
    std::printf("yaw rate and roll source: %s\n", use_locationd ? "locationd (ESP12 when invalid or stale)" : "ESP12");
    LateralLearners learners(sp, vehicle_json, cache, options);
    if (!vehicle_json.empty())
        std::printf("paramsd saved values: %s\n", learners.vehicle_restored() ? "restored" : "rejected");
    // 결합 직진 구간의 곡률 대조: 학습값 차량 모델(조향각) − 요레이트(바이어스 제거)
    TorqueController curvature_model;
    struct CurvatureSums {  // 출처(ESP12·locationd)별
        double diff = 0.0, angle = 0.0, yaw = 0.0, roll = 0.0;
        long n = 0;
    } curv[2];
    LocalizationState loc{};
    bool have_loc = false;
    if (!cache.empty())
        std::printf("torqued cache %zu B: %s\n", cache.size(), torque_restore_name(learners.torque_restore_status()));
    SteeringPressedFilter steering_pressed;  // 컨트롤러와 같은 판정
    SpeedFilter speed_filter;

    std::FILE *out_file = outputs_path.empty() ? nullptr : std::fopen(outputs_path.c_str(), "w");
    if (out_file)
        std::fprintf(out_file,
                     "t,inputs_ok,valid,sensor_valid,steer_ratio,stiffness,roll_deg,"
                     "offset_avg_deg,offset_deg,sr_std,sf_std,yaw_bias\n");
    std::FILE *tout_file = torque_outputs_path.empty() ? nullptr : std::fopen(torque_outputs_path.c_str(), "w");
    if (tout_file)
        std::fprintf(tout_file,
                     "t,inputs_ok,valid,factor_raw,offset_raw,friction_raw,factor,offset,"
                     "friction,points,cal_perc,decay\n");

    VehicleCanState vehicle{};
    // ControlState 레코드 시각은 controlsd가 만든 시각이라 앞에 기록된 CAN 배치보다 이를 수
    // 있다. 실시간이면 now가 항상 수신 시각 뒤이므로, 재생에서도 본 CAN 중 최신을 하한으로 둔다.
    double latest_can_s = 0.0;
    long publishes = 0, active_ticks = 0, ticks = 0;
    double first_t = -1.0, last_t = 0.0;
    // 레코드는 기록 시각 순으로 읽는다(RouteEvents). 곡률 대조는 레코드가 든 파일 번호로 고른다.
    RouteEvents route_events(events);
    TimeOrderedEvents::Record record;
    while (route_events.next(&record)) {
        const bool count_metric = record.file >= static_cast<size_t>(metric_from_file);
        const EventRecordHeader &rh = record.header;
        const RecordType type = record_type(rh);
        const std::vector<char> &buf = record.payload;
        const double record_s = record_time_s(rh);
        const double now_s = std::max(record_s, latest_can_s);
        if (type == RecordType::CanRx) {
            latest_can_s = std::max(latest_can_s, record_s);
            apply_recorded_can(buf.data(), rh.payload_size, now_s, &vehicle);
            continue;
        }
        if (type == RecordType::Localization) {
            if (read_record(buf, &loc)) have_loc = true;
            continue;
        }
        // 보드가 실제로 쓴 조향 지연(torqued lag도 이 값을 따른다). 예전 기록은 0이라 건너뛴다
        if (type == RecordType::LearnerState) {
            LearnerState ls;
            if (read_record(buf, &ls) && ls.plan_delay_s > 0.0f) learners.set_lateral_delay(ls.plan_delay_s);
            continue;
        }
        // 제어 틱(ControlState, 100 Hz)마다 한 번 넣는다. controlsd가 부를 자리와 같다.
        ControlState cs;
        if (type != RecordType::ControlState || !read_record(buf, &cs)) continue;
        const bool pressed = steering_pressed.update(cs.driver_torque, sp.steering_pressed_threshold);
        // controlsd처럼 매 틱 최신 locationd 상태로 판단한다(기록 시각 = 학습기 시계)
        if (have_loc) {
            const double sample_t_s = static_cast<double>(loc.timestamp_ns) * 1e-9;
            LocalizerSample sample;
            const bool use = localizer_sample_from(loc, use_locationd, now_s - sample_t_s, sample_t_s, &sample);
            learners.set_localizer(use, sample);
        }
        // controlsd처럼 컨트롤러가 거른 vEgo를 준다(상류 carState.vEgo)
        const double v_ego = speed_filter.update(vehicle_speed_kph(vehicle, now_s, kVehicleStateTimeoutS) / 3.6);
        const bool lat_active = recorded_lat_active(cs);
        // 신선도도 컨트롤러와 같은 판정이다(LateralControlOutput::vehicle_fresh·esp12_fresh)
        learners.update(vehicle, now_s, vehicle_state_fresh(vehicle, now_s, kVehicleStateTimeoutS),
                        signal_time_fresh(vehicle.esp12_time_s, now_s, kVehicleStateTimeoutS), v_ego, lat_active,
                        cs.apply_torque, pressed);

        const VehicleParamsInput &in = learners.last_vehicle_input();
        const LiveLateralParams live = learners.live();
        if (count_metric && cs.active && in.inputs_fresh && in.yaw_rate_valid && in.speed_mps > 12.0 &&
            std::fabs(cs.desired_curvature) < 3e-4f && live.use_vehicle) {
            const double angle_curv = curvature_model.estimate_actual_curvature(
                static_cast<float>(in.speed_mps), static_cast<float>(in.steering_angle_deg), sp, 0.0f, false, live);
            const double yaw_curv = -in.yaw_rate_rad_s / in.speed_mps;  // 좌측 양수 → 우측 양수
            CurvatureSums &c = curv[in.localizer_roll_given ? 1 : 0];   // 이 틱에 넣은 요레이트의 출처
            c.diff += angle_curv - yaw_curv;
            c.angle += angle_curv;
            c.yaw += yaw_curv;
            c.roll += live.roll_rad;
            ++c.n;
        }
        if (first_t < 0.0) first_t = now_s;
        last_t = now_s;
        ++ticks;
        if (in.inputs_fresh && in.speed_mps > 1.0 && std::fabs(in.steering_angle_deg) < 45.0 && in.gear != kGearReverse)
            ++active_ticks;
        if (learners.vehicle_published()) {
            ++publishes;
            const VehicleParams &p = learners.vehicle_params();
            if (out_file)
                std::fprintf(out_file, "%.4f,%d,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", now_s - first_t,
                             p.inputs_ok, p.valid, p.sensor_valid, p.steer_ratio, p.stiffness_factor,
                             p.roll_rad * 180.0 / kPi, p.angle_offset_average_deg, p.angle_offset_deg,
                             p.steer_ratio_std, p.stiffness_factor_std, learners.yaw_bias_rad_s());
        }
        if (learners.torque_persist_due() && !torque_cache_path.empty()) {
            std::ofstream f(torque_cache_path, std::ios::binary | std::ios::trunc);
            f.write(learners.torque_persist_data().data(),
                    static_cast<std::streamsize>(learners.torque_persist_data().size()));
        }
        if (learners.torque_published() && tout_file) {
            const TorqueParams &q = learners.torque_params();
            std::fprintf(tout_file, "%.4f,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g\n", now_s - first_t,
                         q.inputs_ok, q.valid, q.lat_accel_factor_raw, q.lat_accel_offset_raw, q.friction_raw,
                         q.lat_accel_factor, q.lat_accel_offset, q.friction, q.total_bucket_points, q.cal_perc,
                         q.decay);
        }
    }
    if (out_file) std::fclose(out_file);
    if (tout_file) std::fclose(tout_file);
    const VehicleParams &p = learners.vehicle_params();
    std::printf(
        "%.0fs, ticks %ld, active %.0fs, outputs %ld | SR %.3f (±%.3f) stiffness %.3f offset %+.3f deg "
        "(total %+.3f) roll %+.3f deg | valid %d | gyro bias %+.4f deg/s\n",
        last_t - first_t, ticks, active_ticks / 100.0, publishes, p.steer_ratio, p.steer_ratio_std, p.stiffness_factor,
        p.angle_offset_average_deg, p.angle_offset_deg, p.roll_rad * 180.0 / kPi, p.valid,
        learners.yaw_bias_rad_s() * 180.0 / kPi);
    for (int k = 0; k < 2; ++k) {
        const CurvatureSums &c = curv[k];
        if (c.n == 0) continue;
        std::printf(
            "engaged straight %ld ticks (%s yaw rate): angle curvature %+.3fe-4, yaw-rate curvature %+.3fe-4, "
            "difference %+.3fe-4 1/m | mean roll %+.3f deg\n",
            c.n, k ? "locationd" : "ESP12", c.angle / c.n * 1e4, c.yaw / c.n * 1e4, c.diff / c.n * 1e4,
            c.roll / c.n * 180.0 / kPi);
    }
    const TorqueParams &q = learners.torque_params();
    std::printf(
        "torqued: points %d (progress %d%%) valid %d | raw factor %.3f offset %+.3f friction %.3f | "
        "filtered factor %.3f offset %+.3f friction %.3f (prior %.3f/%.3f) decay %.1f\n",
        q.total_bucket_points, q.cal_perc, q.valid, q.lat_accel_factor_raw, q.lat_accel_offset_raw, q.friction_raw,
        q.lat_accel_factor, q.lat_accel_offset, q.friction, sp.torque_lat_accel_factor, sp.torque_friction, q.decay);
    for (int b = 0; b < TorqueEstimator::kBuckets; ++b)
        std::printf("%s%d", b ? " " : "  buckets ", learners.torque_estimator().bucket_size(b));
    std::printf("\n");
    return route_events.exit_code();
}
