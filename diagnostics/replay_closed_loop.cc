/* 녹화된 인지 출력을 자세 보정해 제어 루프를 폐루프로 재생한다.
 * 개루프 재생(replay_planner)은 차 응답이 녹화에 고정돼 토크를 바꿔도 오차가
 * 안 움직인다. 여기서는 시뮬 차가 녹화 차와 벌어진 만큼(dy, dpsi) 차선 기하를
 * 차체 좌표로 다시 돌려, 제어 변경이 거동에 반영된다.
 * 운전자가 개입했거나 비활성인 틱은 실측 상태로 재동기화한다. 시뮬은 hands-off
 * 구간만 자유 주행하고 각 구간은 실제 자세에서 출발한다(seg 열이 구간 번호).
 * 사용: replay_closed_loop [옵션] <out.csv|-> <events.bin...>   (-는 CSV 없이 점수만)
 *   --open-loop                자세 보정을 고정해 플랜트 재현도만 본다
 *   --wn/--zeta/--delay N      플랜트 고유진동수·감쇠·지연 틱
 *   --gain G, --gain-pts a,b,c,d  플랜트 이득(속도 노드 전부 G, 또는 노드별)
 *   --sad S, --kp, --ki, --laf  컨트롤러 steer_actuator_delay·토크 이득
 *   --driver-high/--driver-low T, --driver-release N  운전자 개입 히스테리시스
 *   --steering route/params/steering.json  녹화 당시 튜닝(없으면 코드 기본값). 녹화의 LearnerState
 *                              학습값(paramsd·torqued)은 controlsd처럼 매번 컨트롤러에 넣는다.
 *   --driving route/params/driving.json  녹화 당시 주행 파라미터(laneless 모드 등)
 *   --torque F,O,R             torqued 학습값(배율·절편·마찰)을 이 값으로 바꿔 쓴다(유효해진 뒤를 본다)
 *   --camera-shift D           카메라 장착 오프셋을 녹화보다 D m 바꾼 것처럼 모델 출력을 옮긴다
 *                              (예: 0.08로 달린 녹화로 0을 보려면 -0.08). */
#include "controls/control_params.h"
#include "recording/event_log_reader.h"
#include "car/hyundai_can.h"
#include "common/ipc_messages.h"
#include "recording/recorded_model_state.h"
#include "controls/lateral_controller.h"
#include "controls/lateral_path.h"
#include "planning/lateral_planner.h"
#include "controls/lateral_torque.h"
#include "recording/recording_format.h"
#include "common/utils_time.h"
#include "car/vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace {

constexpr double kDtCtrl = 0.01;
// 녹화가 본 차선 범위 밖은 외삽이라 신뢰 구간을 벗어난다.
constexpr float kMaxDeviationY = 2.0f;
constexpr float kMaxDeviationPsi = 0.15f;

// 비워 둔 값은 코드 기본값을 쓴다.
struct Options {
  bool open_loop = false;
  std::optional<float> sad, kp, ki, laf, gain;
  const char *gain_pts = nullptr;
  const char *steering_path = nullptr;
  const char *driving_path = nullptr;
  const char *torque = nullptr;
  float camera_shift = 0.0f;
  float wn = 10.0f, zeta = 4.0f;
  int delay = 0, driver_high = 150, driver_low = 60, driver_release = 50;
  std::vector<const char *> positional;
};

[[noreturn]] void usage(const char *argv0) {
  std::fprintf(stderr,
               "usage: %s [--open-loop] [--wn W] [--zeta Z] [--delay N] [--gain G | --gain-pts a,b,c,d]\n"
               "       [--sad S] [--kp KP] [--ki KI] [--laf LAF] [--driver-high T] [--driver-low T]\n"
               "       [--driver-release N] [--steering steering.json] [--driving driving.json] [--camera-shift D]\n"
               "       [--torque factor,offset,friction]\n"
               "       <out.csv|-> <events.bin...>\n",
               argv0);
  std::exit(2);
}

Options parse_options(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> const char * {
      if (i + 1 >= argc) usage(argv[0]);
      return argv[++i];
    };
    if (arg == "--open-loop") o.open_loop = true;
    else if (arg == "--wn") o.wn = static_cast<float>(std::atof(value()));
    else if (arg == "--zeta") o.zeta = static_cast<float>(std::atof(value()));
    else if (arg == "--delay") o.delay = std::atoi(value());
    else if (arg == "--gain") o.gain = static_cast<float>(std::atof(value()));
    else if (arg == "--gain-pts") o.gain_pts = value();
    else if (arg == "--sad") o.sad = static_cast<float>(std::atof(value()));
    else if (arg == "--kp") o.kp = static_cast<float>(std::atof(value()));
    else if (arg == "--ki") o.ki = static_cast<float>(std::atof(value()));
    else if (arg == "--laf") o.laf = static_cast<float>(std::atof(value()));
    else if (arg == "--driver-high") o.driver_high = std::atoi(value());
    else if (arg == "--driver-low") o.driver_low = std::atoi(value());
    else if (arg == "--driver-release") o.driver_release = std::atoi(value());
    else if (arg == "--steering") o.steering_path = value();
    else if (arg == "--driving") o.driving_path = value();
    else if (arg == "--torque") o.torque = value();
    else if (arg == "--camera-shift") o.camera_shift = static_cast<float>(std::atof(value()));
    else if (arg.rfind("--", 0) == 0) usage(argv[0]);
    else o.positional.push_back(argv[i]);
  }
  if (o.positional.size() < 2) usage(argv[0]);
  return o;
}

// 이득 곡선의 속도 노드(m/s). 일정한 kf 가정이 저속에서 깨져 노드가 필요하다.
constexpr float kGainSpeedsMps[] = {3.0f, 8.0f, 15.0f, 25.0f};
constexpr int kGainNodes = 4;

/* 토크 -> 횡가속도 플랜트. DC 이득은 1/kf(=FF 보정이 맞다는 가정)이고
 * 이득 곡선이 그 속도별 오차를 담는다. 순수 지연 + 2차 지연으로 MDPS/타이어
 * 응답을 낸다. */
class Plant {
public:
  Plant(float wn, float zeta, const float *gain, int delay_frames)
      : wn_(wn), zeta_(zeta),
        delay_(static_cast<size_t>(std::max(0, delay_frames)) + 1, 0.0f) {
    std::copy(gain, gain + kGainNodes, gain_);
  }

  float gain_at(float v) const {
    if (v <= kGainSpeedsMps[0]) return gain_[0];
    for (int i = 1; i < kGainNodes; ++i) {
      if (v <= kGainSpeedsMps[i]) {
        const float p = (v - kGainSpeedsMps[i - 1]) /
                        (kGainSpeedsMps[i] - kGainSpeedsMps[i - 1]);
        return gain_[i - 1] + p * (gain_[i] - gain_[i - 1]);
      }
    }
    return gain_[kGainNodes - 1];
  }

  float step(float a_cmd, float v) {
    delay_[head_] = a_cmd;
    head_ = (head_ + 1) % delay_.size();
    const float u = delay_[head_] * gain_at(v);
    const float acc = wn_ * wn_ * (u - a_) - 2.0f * zeta_ * wn_ * a_dot_;
    a_dot_ += acc * static_cast<float>(kDtCtrl);
    a_ += a_dot_ * static_cast<float>(kDtCtrl);
    return a_;
  }

  // 자유 주행 구간의 시작을 실측 상태에 맞춘다. 지연선까지 채워야 첫 틱이 안 튄다.
  void resync(float a) {
    a_ = a;
    a_dot_ = 0.0f;
    std::fill(delay_.begin(), delay_.end(), a);
  }

private:
  float wn_, zeta_;
  float gain_[kGainNodes] = {};
  std::vector<float> delay_;
  size_t head_ = 0;
  float a_ = 0.0f;
  float a_dot_ = 0.0f;
};

// 녹화 차 기준 (dy, dpsi)만큼 벌어진 시뮬 차의 차체 좌표로 옮긴다.
void transform_model(ModelState &ms, float dy, float dpsi) {
  const float c = std::cos(dpsi);
  const float s = std::sin(dpsi);
  const auto tf = [&](IpcPoint &p) {
    const float x = p.x;
    const float y = p.y - dy;
    p.x = c * x + s * y;
    p.y = -s * x + c * y;
  };
  for (int i = 0; i < kTrajectorySize; ++i) {
    tf(ms.plan[i]);
    for (auto &lane : ms.lanes) tf(lane[i]);
    for (auto &edge : ms.road_edges) tf(edge[i]);
  }
  // 플래너는 lane x를 보간축으로 쓴다. 회전 뒤 역행하면 보간이 깨진다.
  for (auto &lane : ms.lanes)
    for (int i = 1; i < kTrajectorySize; ++i)
      lane[i].x = std::max(lane[i].x, lane[i - 1].x);
}

// 녹화에서 그대로 가져오는 외생 입력. 시뮬이 바꿀 수 없는 것들이다.
struct Exogenous {
  float v_kph = 0.0f;
  float k_rec = 0.0f;
  float angle_rec = 0.0f;
  int driver_torque = 0;
  int apply_rec = 0;
  uint32_t active_rec = 0;
};

}  // namespace

int main(int argc, char **argv) {
  const Options opt = parse_options(argc, argv);
  const bool open_loop = opt.open_loop;

  SteeringParams steering;
  DrivingParams driving;
  if (opt.steering_path != nullptr) {
    std::string error;
    if (!load_steering_params_json(opt.steering_path, &steering, &error)) {
      std::fprintf(stderr, "%s: %s\n", opt.steering_path, error.c_str());
      return 1;
    }
  }
  if (opt.driving_path != nullptr) {
    std::string error;
    if (!load_driving_params_json(opt.driving_path, &driving, &error)) {
      std::fprintf(stderr, "%s: %s\n", opt.driving_path, error.c_str());
      return 1;
    }
  }
  steering.steer_actuator_delay = opt.sad.value_or(steering.steer_actuator_delay);
  steering.torque_kp = opt.kp.value_or(steering.torque_kp);
  steering.torque_ki = opt.ki.value_or(steering.torque_ki);
  steering.torque_lat_accel_factor = opt.laf.value_or(steering.torque_lat_accel_factor);
  /* 편경사 추정은 ESP12 실측이 필요한데 ControlState에 없다. 꺼서 0으로 고정하고
   * 그만큼을 플랜트 이득이 아니라 미모델 외란으로 남긴다. */
  steering.live_bank_compensation = false;

  /* MaixCAM2 차선 주행(2026-10-03 00-05-56, 10-04 02-29-44)에서 개루프 재현 R2가 가장 높았던
   * 곡선(이전 식별 곡선 0.70/0.35/0.70/1.20의 1.5배: R2 0.960/0.956 → 0.971/0.976). 이전 값은
   * 차의 토크 응답을 작게 잡아, 횡가속 계수를 올리면(torqued 3.15) 곡선 바깥으로 12 cm 밀린다는
   * 잘못된 결론을 냈다. docs/closed-loop-replay.md */
  const float default_gain[kGainNodes] = {1.05f, 0.525f, 1.05f, 1.80f};
  float gain_curve[kGainNodes];
  for (int i = 0; i < kGainNodes; ++i) gain_curve[i] = opt.gain.value_or(default_gain[i]);
  if (opt.gain_pts != nullptr) {
    std::sscanf(opt.gain_pts, "%f,%f,%f,%f", &gain_curve[0], &gain_curve[1], &gain_curve[2],
                &gain_curve[3]);
  }
  Plant plant(opt.wn, opt.zeta, gain_curve, opt.delay);

  LateralPlanner planner(steering, driving);
  LateralControllerConfig cfg;
  cfg.force_engaged = true;
  cfg.steering_params = steering;
  cfg.driving_params = driving;
  LateralController controller(cfg);
  constexpr uint64_t kControllerNowNs = 1'000'000'000'000ULL;  // plan 나이를 재는 컨트롤러 시계(고정)
  controller.set_clock([] { return kControllerNowNs; });
  TorqueController inverse_model;

  const char *out_path = opt.positional[0];
  const bool want_csv = std::strcmp(out_path, "-") != 0;
  std::FILE *out = std::fopen(want_csv ? out_path : "/dev/null", "w");
  if (out == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", out_path);
    return 1;
  }
  /* lane_y는 시뮬 차체 기준 차선 중앙 오프셋이다. dy와 달리 녹화 주행을
   * 기준으로 삼지 않아 설정 간 A/B에 편향이 없다. */
  std::fprintf(out, "t,seg,v_kph,active,block,k_des,k_sim,k_rec,a_cmd,a_act,"
                    "angle_sim,angle_rec,t_des,t_apply,t_rec,driver,active_rec,dy,dpsi,"
                    "lane_y,lane_ok,clamped\n");

  Exogenous ex;
  ModelState ms_sim{};
  bool have_model = false;
  LateralTarget target{};
  double sim_t = -1.0;
  double route_t0 = -1.0;
  float dy = 0.0f;
  float dpsi = 0.0f;
  bool calibrated = true;
  LiveLateralParams live_now;
  float angle_sim = 0.0f;
  float k_sim = 0.0f;
  bool active_prev = false;
  int frame = 0;
  int clamped = 0;
  int segment = 0;
  int segment_frames = 0;
  bool resynced_prev = true;
  /* 재현 점수: 자유 주행 구간에서 시뮬 횡가속도가 실측을 얼마나 설명하는가.
   * 구간 시작 0.5초는 재동기화 과도라 뺀다. */
  long score_n = 0;
  double score_err2 = 0.0, score_sum = 0.0, score_sum2 = 0.0;
  // 속도 대역별 점수. 단일 이득으로는 저속과 고속이 같이 안 맞는다.
  constexpr int kBands = 4;
  const float band_edges[kBands + 1] = {0.0f, 20.0f, 35.0f, 55.0f, 200.0f};
  long band_n[kBands] = {};
  double band_err2[kBands] = {}, band_sum[kBands] = {}, band_sum2[kBands] = {};
  /* 운전자 토크는 hands-off에서도 노이즈가 크다(활성 중 p50 14, p90 156).
   * 단일 임계로 자르면 구간이 0.06초로 부서진다. 컨트롤러처럼 히스테리시스와
   * 해제 지연을 둔다. */
  const int driver_high = opt.driver_high;
  const int driver_low = opt.driver_low;
  const int driver_release = opt.driver_release;
  bool driver_engaged = true;
  int driver_quiet_frames = 0;

  const auto tick = [&]() {
    const float v_mps = ex.v_kph / 3.6f;
    const float v_clamped = std::max(v_mps, 1.0f);
    const double age_s = have_model
        ? std::max(0.0, sim_t - static_cast<double>(ms_sim.model_timestamp_ns) * 1e-9)
        : 0.0;

    VehicleCanState vehicle{};
    vehicle.has_lkas11_seed = vehicle.has_clu11_seed = vehicle.has_mdps12_seed = true;
    for (double *stamp : {&vehicle.lkas11_time_s, &vehicle.clu11_time_s,
                          &vehicle.sas11_time_s, &vehicle.esp12_time_s,
                          &vehicle.whl_spd11_time_s, &vehicle.scc11_time_s,
                          &vehicle.mdps12_time_s, &vehicle.tcs13_time_s,
                          &vehicle.tcs15_time_s, &vehicle.e_ems11_time_s,
                          &vehicle.elect_gear_time_s, &vehicle.cgw1_time_s,
                          &vehicle.cgw2_time_s}) {
      *stamp = sim_t;
    }
    vehicle.gear = kGearDrive;
    vehicle.cluster_speed_raw = ex.v_kph;
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = ex.v_kph;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = ex.v_kph;
    vehicle.steering_angle_deg = angle_sim;
    vehicle.driver_torque = ex.driver_torque;
    vehicle.yaw_rate_valid = false;

    /* plan 나이는 컨트롤러가 자기 시계로 잰다. 그 시계를 고정해 두고 캡처 시각을 나이만큼
     * 앞당겨, 호스트 시간과 무관하게 결정론적으로 만든다. */
    LateralTarget t = target;
    if (t.valid) t.capture_timestamp_ns = kControllerNowNs -
        static_cast<uint64_t>(age_s * 1e9);
    const uint64_t path_now_ns = ms_sim.model_timestamp_ns +
        static_cast<uint64_t>(age_s * 1e9);
    LateralPath path = have_model ? path_from_model_state(ms_sim, path_now_ns)
                                  : LateralPath{};

    const LateralControlResult r =
        controller.update(path, t, vehicle, sim_t, frame++, true, true);
    active_prev = r.active;

    // 토크 -> 요청 횡가속도. 부호는 kTorqueOutputSign의 역이다(±1이라 같은 값을 곱한다).
    const float a_cmd = static_cast<float>(kTorqueOutputSign * r.apply_torque) /
        ((1.0f / steering.torque_lat_accel_factor) * static_cast<float>(steering.steer_max));
    const float a_act = plant.step(a_cmd, v_mps);
    k_sim = a_act / (v_clamped * v_clamped);

    /* 곡률 -> 조향각. 컨트롤러와 같은 차량 모델(학습값을 쓰면 그 SR·강성·영점·롤)을 역으로
     * 써서, 컨트롤러가 재는 곡률이 시뮬 곡률과 같다. */
    const LiveLateralParams live_vm =
        steering.use_live_vehicle_params ? live_now : LiveLateralParams{};
    const float offset_deg = live_vm.use_vehicle ? live_vm.angle_offset_deg : steering.angle_offset_deg;
    const float k0 = inverse_model.estimate_actual_curvature(v_mps, offset_deg, steering, 0.0f, false, live_vm);
    const float per_deg =
        inverse_model.estimate_actual_curvature(v_mps, offset_deg + 1.0f, steering, 0.0f, false, live_vm) - k0;
    if (std::fabs(per_deg) > 1e-9f)
      angle_sim = (k_sim - k0) / per_deg + offset_deg;

    /* 운전자 토크나 비활성은 시뮬이 못 재현하는 외란이다. 그 구간은 실측으로
     * 되돌리고, 다음 자유 주행 구간이 실제 자세에서 출발하게 한다. */
    const int driver_abs = std::abs(ex.driver_torque);
    if (driver_abs >= driver_high) {
      driver_engaged = true;
      driver_quiet_frames = 0;
    } else if (driver_abs < driver_low) {
      if (++driver_quiet_frames >= driver_release) driver_engaged = false;
    } else {
      driver_quiet_frames = 0;
    }
    const bool resync = !r.active || driver_engaged;
    if (resync) {
      plant.resync(ex.k_rec * v_clamped * v_clamped);
      k_sim = ex.k_rec;
      angle_sim = ex.angle_rec;
      dy = 0.0f;
      dpsi = 0.0f;
    } else {
      if (resynced_prev) { ++segment; segment_frames = 0; }
      if (++segment_frames > 50) {
        const double a_sim = static_cast<double>(k_sim) * v_clamped * v_clamped;
        const double a_rec = static_cast<double>(ex.k_rec) * v_clamped * v_clamped;
        ++score_n;
        score_err2 += (a_sim - a_rec) * (a_sim - a_rec);
        score_sum += a_rec;
        score_sum2 += a_rec * a_rec;
        for (int b = 0; b < kBands; ++b) {
          if (ex.v_kph >= band_edges[b] && ex.v_kph < band_edges[b + 1]) {
            ++band_n[b];
            band_err2[b] += (a_sim - a_rec) * (a_sim - a_rec);
            band_sum[b] += a_rec;
            band_sum2[b] += a_rec * a_rec;
            break;
          }
        }
      }
      dpsi += static_cast<float>(kDtCtrl) * v_mps * (k_sim - ex.k_rec);
      dy += static_cast<float>(kDtCtrl) * v_mps * dpsi;
      const float dy_c = std::clamp(dy, -kMaxDeviationY, kMaxDeviationY);
      const float dpsi_c = std::clamp(dpsi, -kMaxDeviationPsi, kMaxDeviationPsi);
      if (dy_c != dy || dpsi_c != dpsi) clamped = 1;
      dy = dy_c;
      dpsi = dpsi_c;
      if (open_loop) { dy = 0.0f; dpsi = 0.0f; }
    }
    resynced_prev = resync;

    const bool lane_ok = have_model && ms_sim.lane_probabilities[1] >= 0.3f &&
                         ms_sim.lane_probabilities[2] >= 0.3f;
    const float lane_y = lane_ok
        ? 0.5f * (ms_sim.lanes[1][0].y + ms_sim.lanes[2][0].y) : 0.0f;
    std::fprintf(out,
                 "%.3f,%d,%.2f,%d,%s,%.6f,%.6f,%.6f,%.4f,%.4f,%.3f,%.3f,"
                 "%d,%d,%d,%d,%d,%.4f,%.5f,%.4f,%d,%d\n",
                 sim_t - route_t0, resync ? 0 : segment, ex.v_kph, r.active ? 1 : 0,
                 block_reason_name(r.active_block), r.desired_curvature, k_sim,
                 ex.k_rec, a_cmd, a_act, angle_sim, ex.angle_rec, r.desired_torque,
                 r.apply_torque, ex.apply_rec, ex.driver_torque,
                 static_cast<int>(ex.active_rec), dy, dpsi, lane_y,
                 lane_ok ? 1 : 0, clamped);
    clamped = 0;
  };

  for (size_t a = 1; a < opt.positional.size(); ++a) {
    EventLogReader reader(opt.positional[a]);
    if (!reader.ok()) continue;
    EventRecordHeader rh{};
    std::vector<char> buf;
    while (reader.next(&rh, &buf)) {
      const double rec_t = static_cast<double>(rh.timestamp_ns) * 1e-9;

      if (sim_t > 0.0) {
        while (sim_t + kDtCtrl <= rec_t) {
          sim_t += kDtCtrl;
          tick();
        }
      }

      // controlsd처럼 학습값을 컨트롤러에 넣는다(스위치는 steering 파라미터가 정한다)
      if (rh.type == static_cast<uint16_t>(RecordType::LearnerState) &&
          rh.payload_size >= sizeof(LearnerState)) {
        LearnerState ls{};
        std::memcpy(&ls, buf.data(), sizeof(ls));
        LiveLateralParams live;
        live.use_vehicle = (ls.flags & kLearnerUseVehicle) != 0;
        live.steer_ratio = ls.steer_ratio;
        live.stiffness_factor = ls.stiffness_factor;
        live.angle_offset_deg = ls.angle_offset_deg;
        live.roll_rad = ls.roll_rad;
        live.use_torque = (ls.flags & kLearnerUseTorque) != 0;
        live.lat_accel_factor = ls.lat_accel_factor;
        live.lat_accel_offset = ls.lat_accel_offset;
        live.friction = ls.friction;
        if (opt.torque != nullptr) {
          float f = 0, o = 0, r = 0;
          if (std::sscanf(opt.torque, "%f,%f,%f", &f, &o, &r) == 3) {
            live.use_torque = true;
            live.lat_accel_factor = f;
            live.lat_accel_offset = o;
            live.friction = r;
          }
        }
        controller.set_live_params(live, (ls.flags & kLearnerVehicleValid) != 0, calibrated);
        live_now = live;
      }
      if (rh.type == static_cast<uint16_t>(RecordType::ControlState) &&
          rh.payload_size >= sizeof(ControlState)) {
        ControlState cs{};
        std::memcpy(&cs, buf.data(), sizeof(cs));
        ex.v_kph = cs.ego_speed_kph > 0.0f ? cs.ego_speed_kph : cs.cluster_speed_kph;
        ex.k_rec = cs.actual_curvature;
        ex.angle_rec = cs.steering_angle_deg;
        ex.driver_torque = cs.driver_torque;
        ex.apply_rec = cs.apply_torque;
        ex.active_rec = cs.active;
      } else if (rh.type == static_cast<uint16_t>(RecordType::ModelState)) {
        ModelState ms{};
        if (!decode_recorded_model_state(buf.data(), rh.payload_size, reader.version(), &ms)) continue;
        if (sim_t < 0.0) {
          sim_t = rec_t;
          route_t0 = rec_t;
        }
        calibrated = ms.calibration.status == 1U;
        ms_sim = ms;
        // 카메라 오프셋 변경: 모델은 실제 점을 y - 오프셋으로 본다(set_camera_mount)
        transform_model(ms_sim, dy + opt.camera_shift, dpsi);
        have_model = true;
        VehicleCanState planner_vehicle{};
        target = planner.update(ms_sim, planner_vehicle, ex.v_kph / 3.6f, k_sim,
                                active_prev);
      }
    }
  }
  std::fclose(out);
  if (score_n > 1) {
    const double var = score_sum2 - score_sum * score_sum / static_cast<double>(score_n);
    const double rmse = std::sqrt(score_err2 / static_cast<double>(score_n));
    const double r2 = var > 0.0 ? 1.0 - score_err2 / var : 0.0;
    std::printf("n=%ld rmse=%.4f r2=%.4f", score_n, rmse, r2);
    for (int b = 0; b < kBands; ++b) {
      if (band_n[b] < 2) { std::printf(" b%d=nan", b); continue; }
      const double bv = band_sum2[b] - band_sum[b] * band_sum[b] /
                                           static_cast<double>(band_n[b]);
      std::printf(" b%d=%.4f", b, bv > 0.0 ? 1.0 - band_err2[b] / bv : 0.0);
    }
    std::printf("\n");
  } else {
    std::printf("n=0 rmse=nan r2=nan\n");
  }
  return 0;
}
