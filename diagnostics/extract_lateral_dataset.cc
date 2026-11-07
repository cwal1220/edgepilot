// recordd 녹화(이벤트 파일)에서 횡제어 분석/학습용 CSV를 뽑는다. CAN 디코드와 부호 규약은
// 런타임과 같은 vehicle_can을 그대로 쓴다. 레코드는 기록 시각 순으로 읽는다(TimeOrderedEvents): 행마다 그 시각까지
// 들어온 CAN과 모델만 본다(controlsd가 그 틱에 본 것).
// 사용: extract_lateral_dataset <out.csv> <events.bin...>
#include "car/hyundai_can.h"
#include "car/vehicle_can.h"
#include "common/ipc_messages.h"
#include "common/utils_math.h"
#include "common/utils_time.h"
#include "controls/control_params.h"
#include "controls/lateral_torque.h"
#include "controls/road_bank.h"
#include "recorded_vehicle_can.h"
#include "recording/event_log_reader.h"
#include "recording/recording_format.h"
#include "route_events.h"
#include "tool_output.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr float kGravity = 9.81f;  // 컨트롤러·학습기와 같다
// openpilot NNFF future_times. 플랜에서 이 시점의 값을 뽑는다.
constexpr float kFutureTimes[] = {0.3f, 0.6f, 1.0f, 1.5f};
constexpr int kFutureCount = 4;
// 요각 미분 폭(초). 플랜 격자가 근거리에서 촘촘해 이 정도면 안정적이다.
constexpr float kYawDiffHalfWindowS = 0.15f;

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

    if (!tool_output_allowed(argv[1])) return 2;
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
                 "roll_p03,roll_p06,roll_p10,roll_p15,model_age,"
                 "toi_active,steer_fault,blinker,eps_out_tq,nn_ff\n");

    const float torque_scale = static_cast<float>(kTorqueOutputSign) / static_cast<float>(kHyundaiSteerMax);

    VehicleCanState vehicle{};
    ModelState model{};
    bool have_model = false;
    double model_time_s = -1.0;
    RoadBankEstimator road_bank;  // 컨트롤러와 같은 편경사 추정(직선에서만 갱신, 30초 낡으면 0으로)
    double last_row_s = -1.0;
    double route_start_s = -1.0;
    uint64_t rows = 0;

    // 8-19 route처럼 tmpfs가 차서 끊긴 파일은 0으로 채워진 구간이 남는다. 리더는 재동기화하지 않고 거기서 끝내고
    // RouteEvents가 알린다. 창은 예전 그대로 0.5초다(데이터셋 CSV가 바뀌지 않게).
    RouteEvents events(std::vector<std::string>(argv + 2, argv + argc), 500'000'000ULL);
    TimeOrderedEvents::Record record;
    while (events.next(&record)) {
        const EventRecordHeader &rh = record.header;
        const RecordType type = record_type(rh);
        const std::vector<char> &buf = record.payload;
        const double now_s = record_time_s(rh);
        if (route_start_s < 0.0) route_start_s = now_s;

        if (type == RecordType::CanRx) {
            apply_recorded_can(buf.data(), rh.payload_size, now_s, &vehicle);
            continue;
        }

        if (type == RecordType::ModelState && read_record(buf, &model)) {
            have_model = model.valid != 0;
            model_time_s = now_s;
            continue;
        }

        ControlState cs{};
        if (type != RecordType::ControlState || !read_record(buf, &cs)) continue;

        const float v = vehicle_speed_kph(vehicle, now_s, kVehicleStateTimeoutS) / 3.6f;
        const bool yaw_fresh =
            signal_time_fresh(vehicle.esp12_time_s, now_s, kVehicleStateTimeoutS) && vehicle.yaw_rate_valid;

        // 편경사: 컨트롤러와 같은 추정기를 100 Hz 대신 ControlState 간격(dt)으로 돌린다.
        const double dt = last_row_s < 0.0 ? 0.0 : now_s - last_row_s;
        last_row_s = now_s;
        if (dt > 0.0 && dt < 0.5) {
            road_bank.update(static_cast<float>(dt), v, vehicle.lat_accel_mps2, vehicle.lat_accel_valid,
                             vehicle.yaw_rate_rad_s, yaw_fresh);
        }
        const float bank = road_bank.lat_accel();
        const float roll = bank / kGravity;

        float future_la[kFutureCount] = {};
        float future_roll[kFutureCount] = {};
        float model_age = -1.0f;
        if (have_model && model_time_s >= 0.0) {
            model_age = static_cast<float>(now_s - model_time_s);
            float grid[kTrajectorySize];
            float yaw[kTrajectorySize];
            // 녹화에는 plan orientation이 없다. 요각은 plan 위치의 중앙차분으로 대신하고(전진 0.5 m 미만이면 heading이
            // 무의미해 0), 미래 롤은 지금의 편경사 롤 그대로다.
            for (int i = 0; i < kTrajectorySize; ++i) {
                grid[i] = model_t_idx(i);
                const int prev = std::max(0, i - 1), next = std::min(kTrajectorySize - 1, i + 1);
                const float dx = model.plan[next].x - model.plan[prev].x;
                const float dy = model.plan[next].y - model.plan[prev].y;
                yaw[i] = std::fabs(dx) >= 0.5f ? std::atan2(dy, dx) : 0.0f;
            }
            for (int i = 0; i < kFutureCount; ++i) {
                const float t = kFutureTimes[i];
                // 미래 횡가속: openpilot은 플랜 acceleration.y를 쓰지만 녹화에는
                // 없다. 요각 미분으로 대체한다. a_y = v * dψ/dt.
                const float ahead = interp_plan(t + kYawDiffHalfWindowS, grid, yaw);
                const float behind = interp_plan(std::max(0.0f, t - kYawDiffHalfWindowS), grid, yaw);
                const float span = (t + kYawDiffHalfWindowS) - std::max(0.0f, t - kYawDiffHalfWindowS);
                future_la[i] = span > 1e-6f ? v * (ahead - behind) / span : 0.0f;
                future_roll[i] = roll;
            }
        }

        const float des_la = cs.desired_curvature * v * v;
        // EPS 상태: ToiActive는 MDPS가 LKAS 토크를 실제로 받아 쓰는 중이라는 뜻이다(학습 표본 고르기). OutTq는
        // MDPS가 실제로 낸 토크로, 보낸 토크와의 상관으로 부호를 확인한다(nnlc coverage.py). MDPS12가 낡았으면 0.
        const bool mdps_fresh = signal_time_fresh(vehicle.mdps12_time_s, now_s, kVehicleStateTimeoutS);
        const Mdps12Values mdps = decode_mdps12(vehicle.mdps12_seed);
        std::fprintf(out,
                     "%.3f,%.3f,%u,%u,%s,%.2f,%d,%d,%.5f,%.5f,"
                     "%.3f,%d,%.5f,%.3f,%.4f,%.5f,"
                     "%.7f,%.7f,%.4f,"
                     "%.4f,%.4f,%.4f,%.4f,"
                     "%.5f,%.5f,%.5f,%.5f,%.3f,"
                     "%d,%d,%d,%d,%d\n",
                     now_s - route_start_s, v, cs.active, cs.desire, cs.active_block[0] ? cs.active_block : "-",
                     cs.steering_angle_deg, cs.driver_torque, cs.apply_torque,
                     static_cast<float>(cs.apply_torque) * torque_scale,
                     static_cast<float>(cs.desired_torque) * torque_scale, vehicle.lat_accel_mps2,
                     vehicle.lat_accel_valid ? 1 : 0, vehicle.yaw_rate_rad_s, vehicle.long_accel_mps2, bank, roll,
                     cs.desired_curvature, cs.actual_curvature, des_la, future_la[0], future_la[1], future_la[2],
                     future_la[3], future_roll[0], future_roll[1], future_roll[2], future_roll[3], model_age,
                     mdps_fresh && mdps.toi_active ? 1 : 0, vehicle.steer_fault_temporary ? 1 : 0,
                     vehicle.left_blinker || vehicle.right_blinker ? 1 : 0, mdps_fresh ? mdps.output_torque : 0,
                     (cs.hud_flags & kHudFlagNnFeedforward) != 0 ? 1 : 0);
        ++rows;
    }

    std::fclose(out);
    std::fprintf(stderr, "%s: %llu rows\n", argv[1], static_cast<unsigned long long>(rows));
    return events.exit_code();
}
