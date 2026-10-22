// 녹화된 인지 출력을 자세 보정해 제어 루프를 폐루프로 재생한다.
// 개루프 재생(replay_planner)은 차 응답이 녹화에 고정돼 토크를 바꿔도 오차가
// 안 움직인다. 여기서는 시뮬 차가 녹화 차와 벌어진 만큼(dy, dpsi) 차선 기하를
// 차체 좌표로 다시 돌려, 제어 변경이 거동에 반영된다.
// 플랜트는 토크를 조향각 곡률(컨트롤러가 재는 값)로 내고, 차의 진행 곡률은 그것을 조향각 → 진행 지연만큼 늦춘
// 것이다(실차 0.10~0.11 s). 자세 차이는 진행 곡률로 쌓는다. 녹화 쪽 진행도 녹화 조향각 곡률을 같은 만큼 늦춘다.
// controlsd가 쓰는 입력 가운데 ESP12(요레이트·횡가속·종가속)는 녹화 CAN에서 시뮬 차가 벌어진 만큼 고쳐 넣고
// (편경사 추정이 녹화와 같다), lagd 지연은 녹화 locationd 상태에서 넣는다.
// 운전자가 개입했거나 비활성인 틱은 실측 상태로 재동기화한다. 시뮬은 hands-off
// 구간만 자유 주행하고 각 구간은 실제 자세에서 출발한다(seg 열이 구간 번호).
// 사용: replay_closed_loop [옵션] <out.csv|-> <events.bin...>   (-는 CSV 없이 점수만)
//   --open-loop                자세 보정을 고정해 플랜트 재현도만 본다
//   --wn/--zeta/--delay N      플랜트 고유진동수·감쇠·지연 틱
//   --gain G, --gain-pts a,b,c,d  플랜트 이득(속도 노드 전부 G, 또는 노드별)
//   --path-lag S               조향각 → 진행 지연(초, 기본 0.11, 0이면 조향각 곡률로 진행한다)
//   --sad S, --kp, --ki, --laf  컨트롤러 steer_actuator_delay·토크 이득. lagd 지연을 쓰는 설정(use_live_delay)이면
//                              녹화의 lagd 추정이 유효한 동안 그것이 경로 지연이다. --laf는 컨트롤러만 바꾸고
//                              플랜트의 토크 환산은 --steering 값 그대로다. 녹화가 torqued를 쓰는 동안 컨트롤러는
//                              학습 배율을 쓰므로 --laf가 닿지 않는다(--torque로 바꾼다, 끝에 경고).
//   --driver-high/--driver-low T, --driver-release N  운전자 개입 히스테리시스
//   --steering route/params/steering.json  녹화 당시 튜닝(없으면 코드 기본값). 녹화의 LearnerState
//                              학습값(paramsd·torqued)은 controlsd처럼 매번 컨트롤러에 넣는다.
//   --torque F,O,R             torqued 학습값(배율·절편·마찰)을 이 값으로 바꿔 쓴다(유효해진 뒤를 본다)
//   --camera-shift D           카메라 장착 오프셋을 녹화보다 D m 바꾼 것처럼 모델 출력을 옮긴다
//                              (예: 0.08로 달린 녹화로 0을 보려면 -0.08).
//   --lane-filter W            차선선 시간 필터의 새 측정 가중(planning/lane_line_filter, 1이면 끈다).
//   --lateral-nn m.json        학습한 토크 모델(docs/lateral-nn.md). --steering의 nn_feedforward가 켜져 있을 때 쓴다.
#include "car/hyundai_can.h"
#include "car/vehicle_can.h"
#include "cli_args.h"
#include "common/ipc_messages.h"
#include "common/utils_time.h"
#include "controls/control_holds.h"
#include "controls/control_params.h"
#include "controls/lateral_controller.h"
#include "controls/lateral_torque.h"
#include "localization/lateral_lag.h"
#include "planning/lateral_planner.h"
#include "recorded_vehicle_can.h"
#include "recording/event_log_reader.h"
#include "recording/recorded_can.h"
#include "recording/recording_format.h"
#include "route_events.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

// 재동기화한 구간의 처음 이만큼(0.5초)은 과도라 점수에서 뺀다.
constexpr int kSegmentWarmupFrames = 50;
// 차선 중앙 오차(CSV lane_y)는 두 자차 차선선이 이만큼 확실할 때만 쓴다.
constexpr float kLaneSureProbability = 0.3f;
// 녹화가 본 차선 범위 밖은 외삽이라 신뢰 구간을 벗어난다.
constexpr float kMaxDeviationY = 2.0f;
constexpr float kMaxDeviationPsi = 0.15f;
// 조향각(차량 모델) 곡률 → 요레이트 곡률 지연. 2026-10-12~10-16 주행 다섯(선형·학습 FF, 30~90 km/h)의 상관 최대가
// 0.10~0.11 s(IMU, ESP12는 0~8 ms 더 늦다)이고, 순수 지연·1차 지연·둘을 합친 모양의 재현도가 같다(R² 0.94~0.98).
constexpr float kDefaultPathLagS = 0.11f;
// 녹화 ESP12가 이보다 낡으면 시뮬 차의 ESP12 값을 무효로 둔다(50 Hz 메시지)
constexpr double kEsp12MaxAgeS = 0.1;
// controlsd처럼 lagd 추정은 이 안의 것만 쓴다(ControlsTick::on_localization)
constexpr double kLocalizationMaxAgeS = 2.0;

// 이득 곡선의 속도 노드(m/s). 일정한 kf 가정이 저속에서 깨져 노드가 필요하다.
constexpr float kGainSpeedsMps[] = {3.0f, 8.0f, 15.0f, 25.0f};
constexpr int kGainNodes = 4;

// 비워 둔 값은 코드 기본값을 쓴다.
struct Options {
    bool open_loop = false;
    std::optional<float> sad, kp, ki, laf, gain, lane_filter;
    std::optional<std::array<float, kGainNodes>> gain_pts;
    std::optional<std::array<float, 3>> torque;  // factor, offset, friction
    const char *steering_path = nullptr;
    const char *lateral_nn = nullptr;
    float camera_shift = 0.0f;
    float wn = 16.0f, zeta = 3.0f;
    float path_lag = kDefaultPathLagS;
    int delay = 0, driver_high = 150, driver_low = 60, driver_release = 50;
    std::vector<const char *> positional;
};

[[noreturn]] void usage(const char *argv0) {
    std::fprintf(stderr,
                 "usage: %s [--open-loop] [--wn W] [--zeta Z] [--delay N] [--gain G | --gain-pts a,b,c,d]\n"
                 "       [--path-lag S] [--sad S] [--kp KP] [--ki KI] [--laf LAF] [--driver-high T]\n"
                 "       [--driver-low T] [--driver-release N] [--steering steering.json] [--camera-shift D]\n"
                 "       [--torque factor,offset,friction] [--lane-filter W] [--lateral-nn m.json]\n"
                 "       <out.csv|-> <events.bin...>\n",
                 argv0);
    std::exit(2);
}

// 숫자 범위는 파라미터 로더와 같다(sad·kp·ki·laf).
Options parse_options(int argc, char **argv) {
    Options o;
    ArgReader args(argc, argv);
    const auto optional = [&](std::optional<float> *out, double lo, double hi) {
        float v = 0.0f;
        args.read(&v, lo, hi);
        *out = v;
    };
    while (args.next()) {
        if (args.is("--open-loop")) {
            o.open_loop = true;
        } else if (args.is("--wn")) {
            args.read(&o.wn, 0.1, 1000.0);
        } else if (args.is("--zeta")) {
            args.read(&o.zeta, 0.0, 100.0);
        } else if (args.is("--delay")) {
            args.read(&o.delay, 0, 1000);
        } else if (args.is("--gain")) {
            optional(&o.gain, 0.0, 100.0);
        } else if (args.is("--gain-pts")) {
            std::array<float, kGainNodes> pts{};
            args.read_list(pts.data(), kGainNodes);
            o.gain_pts = pts;
        } else if (args.is("--sad")) {
            optional(&o.sad, 0.01, 1.0);
        } else if (args.is("--kp")) {
            optional(&o.kp, 0.0, 10.0);
        } else if (args.is("--ki")) {
            optional(&o.ki, 0.0, 2.0);
        } else if (args.is("--laf")) {
            optional(&o.laf, 0.5, 5.0);
        } else if (args.is("--driver-high")) {
            args.read(&o.driver_high, 0, 10000);
        } else if (args.is("--driver-low")) {
            args.read(&o.driver_low, 0, 10000);
        } else if (args.is("--driver-release")) {
            args.read(&o.driver_release, 0, 100000);
        } else if (args.is("--steering")) {
            o.steering_path = args.value();
        } else if (args.is("--torque")) {
            std::array<float, 3> torque{};
            args.read_list(torque.data(), 3);
            o.torque = torque;
        } else if (args.is("--camera-shift")) {
            args.read(&o.camera_shift, -kMaxDeviationY, kMaxDeviationY);
        } else if (args.is("--lane-filter")) {
            optional(&o.lane_filter, 0.0, 1.0);
            if (*o.lane_filter <= 0.0f) args.fail();
        } else if (args.is("--lateral-nn")) {
            o.lateral_nn = args.value();
        } else if (args.is("--path-lag")) {
            args.read(&o.path_lag, 0.0, 1.0);
        } else if (args.is_option()) {
            args.fail();
        } else {
            o.positional.push_back(args.arg());
        }
    }
    if (args.error() || o.positional.size() < 2) usage(argv[0]);
    return o;
}

// 고정 틱 수만큼 늦춘 값을 내는 지연선. 넣은 값이 delay틱 뒤에 나온다(0이면 그대로).
class DelayLine {
public:
    explicit DelayLine(int delay_frames) : buffer_(static_cast<size_t>(std::max(0, delay_frames)) + 1, 0.0f) {}
    float push(float value) {
        buffer_[head_] = value;
        head_ = (head_ + 1) % buffer_.size();
        return buffer_[head_];
    }
    // 지연선 전체를 value로 채운다(그 값이 계속 들어온 것처럼)
    void fill(float value) { std::fill(buffer_.begin(), buffer_.end(), value); }

private:
    std::vector<float> buffer_;
    size_t head_ = 0;
};

// 토크 -> 횡가속도 플랜트. DC 이득은 1/kf(=FF 보정이 맞다는 가정)이고
// 이득 곡선이 그 속도별 오차를 담는다. 순수 지연 + 2차 지연으로 MDPS/타이어
// 응답을 낸다.
class Plant {
public:
    Plant(float wn, float zeta, const float *gain, int delay_frames) : wn_(wn), zeta_(zeta), delay_(delay_frames) {
        std::copy(gain, gain + kGainNodes, gain_);
    }

    float gain_at(float v) const {
        if (v <= kGainSpeedsMps[0]) return gain_[0];
        for (int i = 1; i < kGainNodes; ++i) {
            if (v <= kGainSpeedsMps[i]) {
                const float p = (v - kGainSpeedsMps[i - 1]) / (kGainSpeedsMps[i] - kGainSpeedsMps[i - 1]);
                return gain_[i - 1] + p * (gain_[i] - gain_[i - 1]);
            }
        }
        return gain_[kGainNodes - 1];
    }

    float step(float a_cmd, float v) {
        const float u = delay_.push(a_cmd) * gain_at(v);
        const float acc = wn_ * wn_ * (u - a_) - 2.0f * zeta_ * wn_ * a_dot_;
        a_dot_ += acc * kDtCtrlF;
        a_ += a_dot_ * kDtCtrlF;
        return a_;
    }

    // 자유 주행 구간의 시작을 실측 상태에 맞춘다. 지연선까지 채워야 첫 틱이 안 튄다.
    void resync(float a) {
        a_ = a;
        a_dot_ = 0.0f;
        delay_.fill(a);
    }

private:
    float wn_, zeta_;
    float gain_[kGainNodes] = {};
    DelayLine delay_;
    float a_ = 0.0f;
    float a_dot_ = 0.0f;
};

// 녹화 차 기준 (dy, dpsi)만큼 벌어진 시뮬 차의 차체 좌표로 옮긴다. 차선·도로 경계는 모델 출력처럼 고정 격자
// (X_IDXS)의 점이라, 옮긴 점을 그 격자에 다시 보간한다. 회전으로 앞 점보다 뒤로 간 점은 앞 점의 x로 묶는다.
void transform_model(ModelState &ms, float dy, float dpsi) {
    const float c = std::cos(dpsi);
    const float s = std::sin(dpsi);
    for (IpcPoint &p : ms.plan) {
        const float x = p.x;
        const float y = p.y - dy;
        p.x = c * x + s * y;
        p.y = -s * x + c * y;
    }
    const auto regrid = [&](IpcLanePoint(&line)[kTrajectorySize]) {
        float xs[kTrajectorySize], ys[kTrajectorySize];
        for (int i = 0; i < kTrajectorySize; ++i) {
            const float x = model_x_idx(i);
            const float y = line[i].y - dy;
            xs[i] = i > 0 ? std::max(c * x + s * y, xs[i - 1]) : c * x + s * y;
            ys[i] = -s * x + c * y;
        }
        for (int i = 0, j = 1; i < kTrajectorySize; ++i) {
            const float x = model_x_idx(i);
            while (j < kTrajectorySize - 1 && xs[j] < x) ++j;
            const float span = xs[j] - xs[j - 1];
            const float p = span > 0.0f ? std::clamp((x - xs[j - 1]) / span, 0.0f, 1.0f) : 1.0f;
            line[i].y = ys[j - 1] + p * (ys[j] - ys[j - 1]);
        }
    };
    for (auto &lane : ms.lanes) regrid(lane);
    for (auto &edge : ms.road_edges) regrid(edge);
}

// 재현 점수의 합: 시뮬 횡가속도(a_sim)가 실측(a_rec)을 얼마나 설명하는가.
struct Score {
    long n = 0;
    double err2 = 0.0, sum = 0.0, sum2 = 0.0;
    void add(double a_sim, double a_rec) {
        ++n;
        err2 += (a_sim - a_rec) * (a_sim - a_rec);
        sum += a_rec;
        sum2 += a_rec * a_rec;
    }
    // 1 − 오차 제곱합 / 실측 분산(분산이 0이면 0)
    double r2() const {
        const double var = sum2 - sum * sum / static_cast<double>(n);
        return var > 0.0 ? 1.0 - err2 / var : 0.0;
    }
};

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
    if (opt.steering_path != nullptr) {
        std::string error;
        if (!load_steering_params(opt.steering_path, &steering, &error)) {
            std::fprintf(stderr, "%s: %s\n", opt.steering_path, error.c_str());
            return 1;
        }
    }
    // 플랜트(차)의 토크 → 횡가속 환산은 route의 steering.json 값으로 고정한다. --laf는 컨트롤러만 바꾼다(예전에는
    // 플랜트 이득까지 같이 바꿔 컨트롤러 효과와 상쇄되거나, torqued를 쓰는 route에서는 플랜트만 바뀌었다).
    const float plant_lat_accel_factor = steering.torque_lat_accel_factor;
    steering.steer_actuator_delay = opt.sad.value_or(steering.steer_actuator_delay);
    steering.torque_kp = opt.kp.value_or(steering.torque_kp);
    steering.torque_ki = opt.ki.value_or(steering.torque_ki);
    steering.torque_lat_accel_factor = opt.laf.value_or(steering.torque_lat_accel_factor);

    // 2026-10-17 다시 식별(docs/closed-loop-replay.md). 녹화 토크 → 조향각 횡가속의 개루프 맞춤은 지연 0~60 ms,
    // wn 6~16의 여러 모양이 R² 0.83으로 같아 시간을 정하지 못한다. 그래서 자세를 고정한 재생에서 녹화 당시
    // 컨트롤러(선형 주행 셋은 선형, 10-16 둘은 학습 토크 모델)의 요구 → 시뮬 조향각 지연이 실차와 맞는 것을 골랐다:
    // 선형 0.444 s(실차 0.444), 학습 FF 0.551 s(0.563), 재현 R² 0.973(이전 wn 10·zeta 4는 0.548/0.640 s로 0.1 s
    // 느렸다). 이득은 같은 맞춤의 노드별 최소제곱(3 m/s는 데이터가 없어 이전 값). 이전 곡선(1.05/0.525/1.05/1.80,
    // 10-03·10-04 차선 주행의 R²로 고른 것)과 8 m/s 아래만 다르다.
    const float default_gain[kGainNodes] = {1.05f, 0.385f, 1.04f, 1.74f};
    float gain_curve[kGainNodes];
    for (int i = 0; i < kGainNodes; ++i)
        gain_curve[i] = opt.gain_pts ? (*opt.gain_pts)[i] : opt.gain.value_or(default_gain[i]);
    Plant plant(opt.wn, opt.zeta, gain_curve, opt.delay);

    LateralPlanner planner(steering);
    if (opt.lane_filter) planner.set_lane_filter_weight(*opt.lane_filter);
    LateralControllerConfig cfg;
    cfg.force_engaged = true;
    cfg.steering_params = steering;
    LateralController controller(cfg);
    if (opt.lateral_nn != nullptr) {
        auto nn = std::make_shared<LateralNnModel>();
        std::string error;
        if (!nn->load(opt.lateral_nn, &error)) {
            std::fprintf(stderr, "lateral nn: %s\n", error.c_str());
            return 1;
        }
        controller.set_lateral_nn(nn);
    } else if (steering.nn_feedforward) {
        std::fprintf(stderr,
                     "warning: nn_feedforward is on but no --lateral-nn model is given; the replay uses the "
                     "linear feed-forward\n");
    }
    PathHoldGate path_gate;
    long laf_ignored_learner_states = 0;  // --laf를 줬는데 녹화가 torqued를 써 컨트롤러가 학습 배율을 쓴 LearnerState
    TorqueController inverse_model;

    const char *out_path = opt.positional[0];
    const bool want_csv = std::strcmp(out_path, "-") != 0;
    std::FILE *out = want_csv ? std::fopen(out_path, "w") : nullptr;  // -면 행을 서식화하지 않는다(점수만)
    if (want_csv && out == nullptr) {
        std::fprintf(stderr, "cannot open %s\n", out_path);
        return 1;
    }
    // lane_y는 시뮬 차체 기준 차선 중앙 오프셋이다. dy와 달리 녹화 주행을
    // 기준으로 삼지 않아 설정 간 A/B에 편향이 없다. k_path는 시뮬 차의 진행 곡률(k_sim을 조향각 → 진행 지연만큼 늦춘
    // 것), delay는 컨트롤러가 쓴 경로 지연, nn과 ff는 학습한 토크 모델을 섞었는지와 피드포워드(횡가속 공간), bank는
    // 컨트롤러의 편경사 추정이다.
    if (out)
        std::fprintf(out,
                     "t,seg,v_kph,active,block,k_des,k_sim,k_rec,a_cmd,a_act,"
                     "angle_sim,angle_rec,t_des,t_apply,t_rec,driver,active_rec,dy,dpsi,"
                     "lane_y,lane_ok,clamped,k_path,delay,nn,ff,bank\n");
    // 진행 곡률: 시뮬과 녹화의 조향각 곡률을 같은 지연선에 지나게 해, 시뮬이 녹화와 같으면 자세 차이도 0이다
    const int path_lag_frames = static_cast<int>(std::lround(opt.path_lag / kDtCtrl));
    DelayLine sim_path(path_lag_frames), rec_path(path_lag_frames);
    float path_deviation = 0.0f;  // 직전 틱의 시뮬 진행 곡률 - 녹화 진행 곡률(ESP12 보정에 쓴다)
    float k_path = 0.0f;
    // controlsd 입력 중 녹화에서 가져오는 것: CAN이 만든 차량 상태(ESP12만 쓴다)와 locationd 상태(lagd 지연)
    VehicleCanState recorded_vehicle{};
    LocalizationState localization{};
    bool have_localization = false;

    Exogenous ex;
    ModelState ms_sim{};
    bool have_model = false;
    LateralTarget target{};
    double sim_t = -1.0;
    double route_t0 = -1.0;
    float dy = 0.0f;
    float dpsi = 0.0f;
    LiveLateralParams live_now;
    float angle_sim = 0.0f;
    float k_sim = 0.0f;
    // 직전 틱의 컨트롤러 판정. controlsd(ControlsTick)처럼 플래너에 latActive와 핸들 조작으로 준다.
    bool lat_active_prev = false, steering_pressed_prev = false;
    int frame = 0;
    int clamped = 0;
    int segment = 0;
    int segment_frames = 0;
    bool resynced_prev = true;
    // 재현 점수: 자유 주행 구간에서 시뮬 횡가속도가 실측을 얼마나 설명하는가(구간 시작 kSegmentWarmupFrames는 뺀다).
    Score score;
    // 속도 대역별 점수. 단일 이득으로는 저속과 고속이 같이 안 맞는다.
    constexpr int kBands = 4;
    const float band_edges[kBands + 1] = {0.0f, 20.0f, 35.0f, 55.0f, 200.0f};
    Score bands[kBands];
    // 운전자 토크는 hands-off에서도 노이즈가 크다(활성 중 p50 14, p90 156).
    // 단일 임계로 자르면 구간이 0.06초로 부서진다. 컨트롤러처럼 히스테리시스와
    // 해제 지연을 둔다.
    const int driver_high = opt.driver_high;
    const int driver_low = opt.driver_low;
    const int driver_release = opt.driver_release;
    bool driver_engaged = true;
    int driver_quiet_frames = 0;

    const auto tick = [&]() {
        const float v_mps = ex.v_kph / 3.6f;
        const float v_clamped = std::max(v_mps, 1.0f);
        // 계획 나이는 controlsd처럼 근거 프레임의 촬영 시각부터 잰다(모델 출력 시각보다 modeld 지연 약 64 ms 이르다)
        const double age_s =
            have_model ? std::max(0.0, sim_t - static_cast<double>(ms_sim.capture_timestamp_ns) * 1e-9) : 0.0;

        VehicleCanState vehicle{};
        vehicle.has_lkas11_seed = vehicle.has_clu11_seed = vehicle.has_mdps12_seed = true;
        stamp_received_messages(&vehicle, sim_t);
        vehicle.scc11_time_s = sim_t;
        vehicle.gear = kGearDrive;
        vehicle.cluster_speed_raw = ex.v_kph;
        vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = ex.v_kph;
        vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = ex.v_kph;
        vehicle.steering_angle_deg = angle_sim;
        vehicle.driver_torque = ex.driver_torque;
        // ESP12: 녹화 값에 시뮬 차가 녹화 차보다 더 도는 만큼(직전 틱의 진행 곡률 차이)을 더한다. 편경사 추정(횡가속 +
        // 요레이트·v)에서는 그 몫이 지워져 녹화와 같은 편경사가 나온다. 속도가 녹화 그대로라 종가속도 그대로 쓴다.
        const bool esp12_fresh =
            recorded_vehicle.esp12_time_s > 0.0 && sim_t - recorded_vehicle.esp12_time_s <= kEsp12MaxAgeS;
        vehicle.yaw_rate_valid = esp12_fresh && recorded_vehicle.yaw_rate_valid;
        vehicle.lat_accel_valid = esp12_fresh && recorded_vehicle.lat_accel_valid;
        if (esp12_fresh) {
            // 요레이트는 ESP12 관례(왼쪽 +), 곡률과 횡가속은 제어 관례(오른쪽 +)다
            vehicle.yaw_rate_rad_s = recorded_vehicle.yaw_rate_rad_s - path_deviation * v_mps;
            vehicle.lat_accel_mps2 = recorded_vehicle.lat_accel_mps2 + path_deviation * v_mps * v_mps;
            vehicle.long_accel_mps2 = recorded_vehicle.long_accel_mps2;
        }
        // lagd 지연: controlsd처럼 확정이고 2초 안의 추정만 쓴다(ControlsTick::on_localization)
        const bool delay_fresh =
            have_localization &&
            sim_t - static_cast<double>(localization.timestamp_ns) * 1e-9 <= kLocalizationMaxAgeS &&
            localization.lag_status == static_cast<uint32_t>(LateralLagStatus::Estimated);
        controller.set_live_delay(localization.lateral_delay_s, delay_fresh);

        // 경로 게이트와 plan 나이의 시각: 녹화 시각(sim_t)으로 잰 나이만큼 캡처 시각 뒤(호스트 시간과 무관하다)
        const uint64_t path_now_ns = ms_sim.capture_timestamp_ns + static_cast<uint64_t>(age_s * 1e9);
        // controlsd(ControlsTick)처럼 무효 plan 한 프레임은 PathHoldGate가 덮는다
        LateralPath path = have_model ? path_gate.update(ms_sim, path_now_ns, kModelTimeoutNs).path : LateralPath{};

        const LateralControlOutput r =
            controller.update(path, target, vehicle, sim_t, frame++, panda_allows_control(), path_now_ns);
        lat_active_prev = r.lat_active;
        steering_pressed_prev = r.steering_pressed;

        // 토크 -> 요청 횡가속도. 부호는 kTorqueOutputSign의 역이다(±1이라 같은 값을 곱한다).
        const float a_cmd = static_cast<float>(kTorqueOutputSign * r.apply_torque) /
                            ((1.0f / plant_lat_accel_factor) * static_cast<float>(kHyundaiSteerMax));
        const float a_act = plant.step(a_cmd, v_mps);
        k_sim = a_act / (v_clamped * v_clamped);

        // 곡률 -> 조향각. 컨트롤러와 같은 차량 모델(학습값을 쓰면 그 SR·강성·영점·롤)을 역으로
        // 써서, 컨트롤러가 재는 곡률이 시뮬 곡률과 같다.
        const LiveLateralParams live_vm = steering.use_live_vehicle_params ? live_now : LiveLateralParams{};
        const float offset_deg = live_vm.use_vehicle ? live_vm.angle_offset_deg : steering.angle_offset_deg;
        const float k0 = inverse_model.estimate_actual_curvature(v_mps, offset_deg, steering, 0.0f, false, live_vm);
        const float per_deg =
            inverse_model.estimate_actual_curvature(v_mps, offset_deg + 1.0f, steering, 0.0f, false, live_vm) - k0;
        if (std::fabs(per_deg) > 1e-9f) angle_sim = (k_sim - k0) / per_deg + offset_deg;

        // 운전자 토크나 비활성은 시뮬이 못 재현하는 외란이다. 그 구간은 실측으로
        // 되돌리고, 다음 자유 주행 구간이 실제 자세에서 출발하게 한다.
        const int driver_abs = std::abs(ex.driver_torque);
        if (driver_abs >= driver_high) {
            driver_engaged = true;
            driver_quiet_frames = 0;
        } else if (driver_abs < driver_low) {
            if (++driver_quiet_frames >= driver_release) driver_engaged = false;
        } else {
            driver_quiet_frames = 0;
        }
        // 보드가 조향하지 않은 틱(해제, 정차 대기)도 되돌린다. 그 구간을 자유 주행으로 채점하면 사람이 조향한 녹화와
        // 비교하게 된다(10-16 route의 채점 틱 35 %).
        const bool resync = !r.active || driver_engaged || ex.active_rec == 0;
        if (resync) {
            plant.resync(ex.k_rec * v_clamped * v_clamped);
            k_sim = ex.k_rec;
            angle_sim = ex.angle_rec;
        }
        // 진행 곡률. 재동기화 틱에는 k_sim이 녹화와 같아 두 지연선에 같은 값이 들어간다.
        k_path = sim_path.push(k_sim);
        const float k_path_rec = rec_path.push(ex.k_rec);
        path_deviation = resync ? 0.0f : k_path - k_path_rec;
        if (resync) {
            dy = 0.0f;
            dpsi = 0.0f;
        } else {
            if (resynced_prev) {
                ++segment;
                segment_frames = 0;
            }
            if (++segment_frames > kSegmentWarmupFrames) {
                const double a_sim = static_cast<double>(k_sim) * v_clamped * v_clamped;
                const double a_rec = static_cast<double>(ex.k_rec) * v_clamped * v_clamped;
                score.add(a_sim, a_rec);
                for (int b = 0; b < kBands; ++b) {
                    if (ex.v_kph >= band_edges[b] && ex.v_kph < band_edges[b + 1]) {
                        bands[b].add(a_sim, a_rec);
                        break;
                    }
                }
            }
            dpsi += kDtCtrlF * v_mps * path_deviation;
            dy += kDtCtrlF * v_mps * dpsi;
            const float dy_c = std::clamp(dy, -kMaxDeviationY, kMaxDeviationY);
            const float dpsi_c = std::clamp(dpsi, -kMaxDeviationPsi, kMaxDeviationPsi);
            if (dy_c != dy || dpsi_c != dpsi) clamped = 1;
            dy = dy_c;
            dpsi = dpsi_c;
            if (open_loop) {
                dy = 0.0f;
                dpsi = 0.0f;
            }
        }
        resynced_prev = resync;

        if (out) {
            const bool lane_ok = have_model && ms_sim.lane_probabilities[1] >= kLaneSureProbability &&
                                 ms_sim.lane_probabilities[2] >= kLaneSureProbability;
            const float lane_y = lane_ok ? 0.5f * (ms_sim.lanes[1][0].y + ms_sim.lanes[2][0].y) : 0.0f;
            std::fprintf(out,
                         "%.3f,%d,%.2f,%d,%s,%.6f,%.6f,%.6f,%.4f,%.4f,%.3f,%.3f,"
                         "%d,%d,%d,%d,%d,%.4f,%.5f,%.4f,%d,%d,%.6f,%.3f,%d,%.4f,%.4f\n",
                         sim_t - route_t0, resync ? 0 : segment, ex.v_kph, r.active ? 1 : 0,
                         block_reason_name(r.active_block), r.desired_curvature, k_sim, ex.k_rec, a_cmd, a_act,
                         angle_sim, ex.angle_rec, r.desired_torque, r.apply_torque, ex.apply_rec, ex.driver_torque,
                         static_cast<int>(ex.active_rec), dy, dpsi, lane_y, lane_ok ? 1 : 0, clamped, k_path,
                         controller.plan_delay_s(), r.nn_feedforward_used ? 1 : 0, r.feedforward,
                         controller.road_bank_lat_accel());
        }
        clamped = 0;
    };

    // 레코드는 기록 시각 순으로 읽는다(RouteEvents). 못 연 파일이 있으면 끝에 종료 코드 1이다.
    RouteEvents events(std::vector<std::string>(opt.positional.begin() + 1, opt.positional.end()));
    TimeOrderedEvents::Record record;
    while (events.next(&record)) {
        const EventRecordHeader &rh = record.header;
        const RecordType type = record_type(rh);
        const std::vector<char> &buf = record.payload;
        const double rec_t = record_time_s(rh);

        if (sim_t > 0.0) {
            while (sim_t + kDtCtrl <= rec_t) {
                sim_t += kDtCtrl;
                tick();
            }
        }

        // controlsd처럼 학습값을 컨트롤러에 넣는다(스위치는 steering 파라미터가 정한다)
        LearnerState ls;
        if (type == RecordType::LearnerState && read_record(buf, &ls)) {
            LiveLateralParams live;
            live.use_vehicle = (ls.flags & kLearnerUseVehicle) != 0;
            live.steer_ratio = ls.steer_ratio;
            live.stiffness_factor = ls.stiffness_factor;
            live.angle_offset_deg = ls.angle_offset_deg;
            live.roll_rad = ls.roll_rad;
            live.use_torque = (ls.flags & kLearnerUseTorque) != 0;
            if (live.use_torque && opt.laf && !opt.torque) ++laf_ignored_learner_states;
            live.lat_accel_factor = ls.lat_accel_factor;
            live.lat_accel_offset = ls.lat_accel_offset;
            live.friction = ls.friction;
            if (opt.torque) {
                live.use_torque = true;
                live.lat_accel_factor = (*opt.torque)[0];
                live.lat_accel_offset = (*opt.torque)[1];
                live.friction = (*opt.torque)[2];
            }
            controller.set_live_params(live, (ls.flags & kLearnerVehicleValid) != 0);
            live_now = live;
        }
        // controlsd와 같은 거름과 파서. 시뮬 차는 이 상태에서 ESP12만 가져간다.
        if (type == RecordType::CanRx) apply_recorded_can(buf.data(), buf.size(), rec_t, &recorded_vehicle);
        if (type == RecordType::Localization && read_record(buf, &localization)) have_localization = true;
        ControlState cs;
        if (type == RecordType::ControlState && read_record(buf, &cs)) {
            ex.v_kph = cs.ego_speed_kph > 0.0f ? cs.ego_speed_kph : cs.cluster_speed_kph;
            ex.k_rec = cs.actual_curvature;
            ex.angle_rec = cs.steering_angle_deg;
            ex.driver_torque = cs.driver_torque;
            ex.apply_rec = cs.apply_torque;
            ex.active_rec = cs.active;
        } else if (type == RecordType::ModelState) {
            ModelState ms{};
            if (!read_record(buf, &ms)) continue;
            if (sim_t < 0.0) {
                sim_t = rec_t;
                route_t0 = rec_t;
            }
            controller.set_calibration_status(
                static_cast<CalibrationStatus>(ms.calibration.status));  // controlsd(ControlsTick)처럼
            ms_sim = ms;
            // 카메라 오프셋 변경: 모델은 실제 점을 y - 오프셋으로 본다(set_camera_mount)
            transform_model(ms_sim, dy + opt.camera_shift, dpsi);
            have_model = true;
            // controlsd처럼 녹화한 CAN의 차량 상태(깜빡이·운전자 토크·사각지대)를 준다. 조향각은 시뮬 차의 것이다.
            // 예전에는 빈 상태를 줘서 차선 변경·회전 desire가 한 번도 돌지 않았다.
            VehicleCanState planner_vehicle = recorded_vehicle;
            planner_vehicle.steering_angle_deg = angle_sim;
            LateralPlannerInput planner_in;
            planner_in.v_ego = ex.v_kph / 3.6f;
            planner_in.measured_curvature = k_sim;
            planner_in.lat_active = lat_active_prev;
            planner_in.steering_pressed = steering_pressed_prev;
            target = planner.update(ms_sim, planner_vehicle, planner_in);
        }
    }
    if (out) std::fclose(out);
    if (laf_ignored_learner_states > 0)
        std::fprintf(stderr,
                     "warning: --laf did not reach the controller in %ld LearnerState records: the recording uses "
                     "torqued (use --torque to replace the learned factor)\n",
                     laf_ignored_learner_states);
    if (score.n > 1) {
        std::printf("n=%ld rmse=%.4f r2=%.4f", score.n, std::sqrt(score.err2 / static_cast<double>(score.n)),
                    score.r2());
        for (int b = 0; b < kBands; ++b) {
            if (bands[b].n < 2)
                std::printf(" b%d=nan", b);
            else
                std::printf(" b%d=%.4f", b, bands[b].r2());
        }
        std::printf("\n");
    } else {
        std::printf("n=0 rmse=nan r2=nan\n");
    }
    return events.exit_code();
}
