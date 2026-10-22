#include "planning/lateral_planner.h"

#include "common/utils_time.h"  // kDtMdl

#include "car/car_specs.h"
#include "common/ipc_messages.h"
#include "controls/control_params.h"
#include "planning/desire_helper.h"
#include "planning/lane_line_filter.h"
#include "planning/lateral_mpc.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {

// plan 끝점이 이보다 가까우면 plan이 붕괴한 것(정지: 실측 5.6 m, 주행: 60 m+).
constexpr double kMinPlanReachM = 10.0;
// 경로 인계 램프(1/s). 차선을 버릴 때 0.5초, 되찾을 때 0.25초.
constexpr double kPlanMixOutRatePerS = 2.0;
constexpr double kPlanMixInRatePerS = 4.0;
// Lane MPC 출발 곡률에서 잰 곡률의 몫(solve_lane_mpc). 폐루프 재생(2026-10-22, 주행 다섯, 플랜트 셋)에서 곡선 유지의
// 안쪽 치우침이 0.25로 절반이 되고(우곡선 11.0 → 5.8 cm) 0.5·1.0은 조금 더 줄 뿐이었다. 대신 차선 측정 잡음을 더
// 따라가 0.5~3 Hz 횡가속이 0.25에서 16 %, 1.0에서 24 % 늘었다.
constexpr double kMeasuredCurvatureSeed = 0.25;
// Lane 모드에서 모델 경로로 넘어가는 판정(update_path_source, 판정 확률 = 좌우 평균). 0.3 밑에 kFallbackEnterFrames
// 프레임 머물러야 넘어가고, 0.15 밑이면 바로 넘어간다. 0.4 위에서 차선으로 돌아온다.
constexpr double kFallbackEnterProbability = 0.3;
constexpr double kFallbackImmediateProbability = 0.15;
constexpr int kFallbackEnterFrames = 6;  // 0.3초
constexpr double kFallbackExitProbability = 0.4;

// 차선 점의 앞 거리(openpilot X_IDXS 격자, 모델이 쓰는 float 값)
const std::array<double, kTrajectorySize> &lane_x_grid() {
    static const std::array<double, kTrajectorySize> grid = [] {
        std::array<double, kTrajectorySize> x{};
        for (int i = 0; i < kTrajectorySize; ++i) x[i] = model_x_idx(i);
        return x;
    }();
    return grid;
}

double interp(double x, const double *xp, const double *fp, size_t count) {
    if (count == 0) return 0.0;
    if (x <= xp[0]) return fp[0];
    for (size_t i = 1; i < count; ++i) {
        if (x <= xp[i]) {
            const double span = xp[i] - xp[i - 1];
            if (span <= 0.0) return fp[i];
            const double p = (x - xp[i - 1]) / span;
            return fp[i - 1] + p * (fp[i] - fp[i - 1]);
        }
    }
    return fp[count - 1];
}

// interp()은 xp가 단조 증가라고 가정한다. 양자화된 plan은 정차 부근에서
// 먼 knot이 뒤로 뛰므로, 보간축으로 쓰기 전에 역행 구간을 평평하게 만든다.
void make_monotonic(double *xp, size_t count) {
    for (size_t i = 1; i < count; ++i)
        if (xp[i] < xp[i - 1]) xp[i] = xp[i - 1];
}

double path_heading_at(const std::array<std::array<double, 3>, kTrajectorySize> &path, int index) {
    const int previous = std::max(0, index - 1);
    const int next = std::min(kTrajectorySize - 1, index + 1);
    if (previous == next) return 0.0;

    const double dx = path[next][0] - path[previous][0];
    const double dy = path[next][1] - path[previous][1];
    const double safe_dx = std::fabs(dx) < 1e-3 ? std::copysign(1e-3, dx == 0.0 ? 1.0 : dx) : dx;
    return std::atan2(dy, safe_dx);
}

class FirstOrderFilter {
public:
    FirstOrderFilter(double value, double rc) : value_(value), alpha_(kDtMdl / (rc + kDtMdl)) {}

    double update(double value) {
        value_ = (1.0 - alpha_) * value_ + alpha_ * value;
        return value_;
    }

    double value() const { return value_; }

private:
    double value_;
    double alpha_;
};

class LanePlanner {
public:
    void set_filter_weight(double weight) {
        left_filter_.set_measurement_weight(weight);
        right_filter_.set_measurement_weight(weight);
    }

    // 자차 차로의 좌우 선을 읽어 시간 필터(planning/lane_line_filter)에 거른다.
    // v_ego와 곡률은 직전 프레임 뒤 자차 움직임이다.
    void parse(const ModelState &model, double v_ego, double curvature) {
        const double t_s = static_cast<double>(model.capture_timestamp_ns) * 1e-9;
        // 차선 점의 앞 거리는 X_IDXS 격자이고, 그 점에 plan이 닿는 시각(lane_t)은 plan에서 나온다
        std::array<float, kTrajectorySize> plan_x{}, lane_t{}, lane_x{}, left{}, right{};
        for (int i = 0; i < kTrajectorySize; ++i) {
            plan_x[i] = model.plan[i].x;
            lane_x[i] = model_x_idx(i);
            left[i] = model.lanes[1][i].y;
            right[i] = model.lanes[2][i].y;
        }
        compute_lane_t(plan_x.data(), lane_t.data());
        for (int i = 0; i < kTrajectorySize; ++i) lane_t_[i] = lane_t[i];
        left_filter_.update(t_s, lane_x.data(), left.data(), v_ego, curvature, left.data());
        right_filter_.update(t_s, lane_x.data(), right.data(), v_ego, curvature, right.data());
        for (int i = 0; i < kTrajectorySize; ++i) {
            left_y_[i] = left[i];
            right_y_[i] = right[i];
        }
        left_prob_ = model.lane_probabilities[1];
        right_prob_ = model.lane_probabilities[2];
        lane_change_scale_ = 1.0;
        left_std_ = model.lane_stds[1];
        right_std_ = model.lane_stds[2];
    }

    // 폭/std 보정을 반영한 유효 확률과 폭 추정을 갱신한다. 랜리스 모드에서도
    // 매 프레임 호출해야 폭 필터와 로그 값이 멈추지 않는다.
    void update_probabilities(float v_ego) {
        std::array<double, kTrajectorySize> width{};
        for (int i = 0; i < kTrajectorySize; ++i) width[i] = right_y_[i] - left_y_[i];

        double left_prob = left_prob_;
        double right_prob = right_prob_;
        // 폭 밴드를 상류 [4, 5]에서 0.5 m 올렸다. 국내 분기구간은 유도선이 차로
        // 한가운데를 지나 흰 차선 기준 폭이 4.2~5.0 m가 되는데, 상류 값에서는
        // 확률 0.95가 0.3 밑으로 깎여 laneless로 빠진다(2026-09-22 잠실대교 진입:
        // 전환 14.7회/분, 우측선 거리 요동 3.7배). 실주행 폭은 p99 4.69 m라
        // 5.5 m 위는 여전히 막는다.
        double width_mod = 1.0;
        for (double t : {0.0, 1.5, 3.0}) {
            const double lane_width = interp(t * (v_ego + 7.0), lane_x_grid().data(), width.data(), width.size());
            const double candidate = lane_width <= 4.5 ? 1.0 : lane_width >= 5.5 ? 0.0 : 5.5 - lane_width;
            width_mod = std::min(width_mod, candidate);
        }
        left_prob *= width_mod;
        right_prob *= width_mod;
        // 막 나타난 차선은 std가 높다. 컷오프를 0.3에서 0.4로 완화해
        // 재획득 직후에도 가중치가 일찍 차오르게 한다.
        const auto std_mod = [](double value) {
            return value <= 0.15 ? 1.0 : value >= 0.4 ? 0.0 : (0.4 - value) / 0.25;
        };
        // 모델 경로 대체 판정은 std를 최근 0.5초의 최솟값으로 본다. 차선이 또렷해도 std가 0.4를 잠깐 넘어(15경로에서
        // 확률 0.5 넘는 프레임의 20~60%가 std 보정 1 미만) 그때마다 대체로 넘어갔다 왔다(분당 3회, 60%가 1초
        // 안). 높은 std의 차선은 실제로 덜 맞으므로(LKA14와의 오차 p50 3.9 cm → std 0.4~0.6에서 8.5 cm) 블렌드
        // 가중치는 그 프레임의 std로 낮추고, 0.5초 넘게 이어질 때만 경로 출처를 바꾼다. 확률이 떨어지는 교차로는 그대로다.
        // 판정의 std 밴드는 블렌드보다 넓다(0.25~0.5, switch_std_mod). 새로 보인 차선은 std가 높아(2026-10-22 시내 주행,
        // 재획득 88건의 처음 std 중앙값 0.62) 블렌드 밴드로는 양쪽 std가 0.3 밑으로 내려올 때까지 모델 경로에 남았다
        // (차선이 보인 뒤 전환까지 중앙값 0.70 s, p90 1.81 s). 그 동안 차는 LKA14 차로 중앙에서 중앙값 12~27 cm
        // 벗어나 있었지만, std 0.4~0.6인 차선의 위치 오차는 중앙값 7 cm였다. 넓힌 밴드로는 0.55 s, p90 1.31 s이고
        // 1초 안에 다시 모델 경로로 돌아가는 재획득은 오히려 줄었다(분당 0.76 → 0.53). 0.3~0.6은 더 빠르지만 새로 차선을
        // 쓰는 프레임의 오차 p90이 55 cm까지 갔다.
        left_std_history_[std_history_index_] = left_std_;
        right_std_history_[std_history_index_] = right_std_;
        std_history_index_ = (std_history_index_ + 1) % kSwitchStdFrames;
        const auto switch_std_mod = [](double value) {
            return value <= kSwitchStdFull   ? 1.0
                   : value >= kSwitchStdZero ? 0.0
                                             : (kSwitchStdZero - value) / (kSwitchStdZero - kSwitchStdFull);
        };
        const double left_switch =
            left_prob * switch_std_mod(*std::min_element(left_std_history_.begin(), left_std_history_.end()));
        const double right_switch =
            right_prob * switch_std_mod(*std::min_element(right_std_history_.begin(), right_std_history_.end()));
        left_prob *= std_mod(left_std_);
        right_prob *= std_mod(right_std_);
        // 차선 std는 std_mod 밴드(0.15~0.40)를 0.10초에 가로지른다(실측 2.43/s).
        // 그대로 쓰면 블렌드 가중치와 랜리스 전환이 같은 프레임에 통째로 뒤집힌다.
        // 둘의 입력을 늦춘다. 평균 가중치는 바뀌지 않는다.
        const double prob_step = kProbRatePerS * kDtMdl;
        left_prob_limited_ = std::clamp(left_prob, left_prob_limited_ - prob_step, left_prob_limited_ + prob_step);
        right_prob_limited_ = std::clamp(right_prob, right_prob_limited_ - prob_step, right_prob_limited_ + prob_step);
        left_switch_limited_ =
            std::clamp(left_switch, left_switch_limited_ - prob_step, left_switch_limited_ + prob_step);
        right_switch_limited_ =
            std::clamp(right_switch, right_switch_limited_ - prob_step, right_switch_limited_ + prob_step);
        // 차선 변경 페이드는 변화율 제한 뒤에 곱한다(상류처럼 바로 적용, 페이드는 desire_helper가 이미 2/s로 바꾼다).
        // 앞에서 곱하면 변경이 Starting에서 취소돼 페이드가 1로 돌아와도 판정 확률이 2/s로만 올라와, 0.3 밑에 머무는
        // 몇 프레임 동안 모델 경로로 빠졌다(10월 주행: Starting에서 끝난 변경 13번 중 11번).
        left_prob_eff_ = left_prob_limited_ * lane_change_scale_;
        right_prob_eff_ = right_prob_limited_ * lane_change_scale_;
        left_switch_prob_ = left_switch_limited_ * lane_change_scale_;
        right_switch_prob_ = right_switch_limited_ * lane_change_scale_;
        left_prob = left_prob_eff_;
        right_prob = right_prob_eff_;

        const double certainty = left_prob * right_prob;
        lane_width_certainty_.update(certainty);
        // 양쪽이 다 보일 때만 폭을 학습한다. estimate는 certainty보다 시정수가 10배
        // 길어(약 10초 대 1초), 한 번 오염되면 재획득 직후 그대로 신뢰된다.
        if (certainty > kWidthLearnMinCertainty) lane_width_estimate_.update(std::fabs(right_y_[0] - left_y_[0]));
        const double speed_width = interp(v_ego, kLaneWidthSpeed.data(), kLaneWidth.data(), kLaneWidthSpeed.size());
        lane_width_ = lane_width_certainty_.value() * lane_width_estimate_.value() +
                      (1.0 - lane_width_certainty_.value()) * speed_width;
        d_prob_ = left_prob + right_prob - left_prob * right_prob;
    }

    std::array<std::array<double, 3>, kTrajectorySize> lane_path(
        const std::array<double, kTrajectorySize> &path_t,
        std::array<std::array<double, 3>, kTrajectorySize> path) const {
        const double half_width = std::min(4.0, lane_width_) * 0.5;
        const double denominator = left_prob_eff_ + right_prob_eff_ + 0.0001;

        std::array<double, kTrajectorySize> lane_path_y{};
        for (int i = 0; i < kTrajectorySize; ++i) {
            const double from_left = left_y_[i] + half_width;
            const double from_right = right_y_[i] - half_width;
            lane_path_y[i] = (left_prob_eff_ * from_left + right_prob_eff_ * from_right) / denominator;
        }

        std::array<double, kTrajectorySize> valid_t{};
        std::array<double, kTrajectorySize> valid_y{};
        size_t valid_count = 0;
        for (int i = 0; i < kTrajectorySize; ++i) {
            if (std::isfinite(lane_t_[i])) {
                valid_t[valid_count] = lane_t_[i];
                valid_y[valid_count] = lane_path_y[i];
                ++valid_count;
            }
        }
        if (valid_count > 0) {
            for (int i = 0; i < kTrajectorySize; ++i) {
                const double lane_y = interp(path_t[i], valid_t.data(), valid_y.data(), valid_count);
                path[i][1] = d_prob_ * lane_y + (1.0 - d_prob_) * path[i][1];
            }
        }
        return path;
    }

    // 모델 경로 대체 판정값: 블렌드와 같은 보정이되 std는 최근 0.5초의 최솟값이다(update_probabilities).
    double mean_switch_probability() const { return (left_switch_prob_ + right_switch_prob_) * 0.5; }
    // 로그/분석용 관측값 노출. left_prob()/right_prob()는 폭·std 보정 전의 값이다: 모델 확률에 차선 변경 페이드와
    // 회전 중 0을 곱한 것.
    double near_left_y() const { return left_y_[0]; }
    double near_right_y() const { return right_y_[0]; }
    double lane_width() const { return lane_width_; }
    double left_prob() const { return left_prob_ * lane_change_scale_; }
    double right_prob() const { return right_prob_ * lane_change_scale_; }
    double left_std() const { return left_std_; }
    double right_std() const { return right_std_; }
    double d_prob() const { return d_prob_; }
    // 차선 변경 중 가까운 차선의 페이드(변화율 제한 뒤에 곱한다)
    void fade_for_lane_change(double scale) { lane_change_scale_ = scale; }
    // 회전 중에는 차선을 뺀다. 변화율 제한 앞에서 0으로 두어 0.5초에 걸쳐 빠진다.
    void drop_for_turn() {
        left_prob_ = 0.0;
        right_prob_ = 0.0;
    }

private:
    static constexpr std::array<double, 2> kLaneWidthSpeed = {0.0, 31.0};
    static constexpr std::array<double, 2> kLaneWidth = {2.8, 3.5};
    static constexpr double kWidthLearnMinCertainty = 0.25;
    // 유효 확률 변화율 상한(1/s). 0→1에 0.5초.
    static constexpr double kProbRatePerS = 2.0;
    // 대체 판정이 std를 보는 창(모델 20 Hz로 0.5초)
    static constexpr size_t kSwitchStdFrames = 10;
    // 모델 경로 대체 판정의 std 보정 밴드: 이 아래는 1, 위는 0(update_probabilities)
    static constexpr double kSwitchStdFull = 0.25;
    static constexpr double kSwitchStdZero = 0.5;
    // 아직 받지 않은 칸은 최솟값에 들지 않는다
    static std::array<double, kSwitchStdFrames> filled_history() {
        std::array<double, kSwitchStdFrames> history;
        history.fill(std::numeric_limits<double>::infinity());
        return history;
    }
    std::array<double, kTrajectorySize> lane_t_{};
    std::array<double, kTrajectorySize> left_y_{};
    std::array<double, kTrajectorySize> right_y_{};
    FirstOrderFilter lane_width_estimate_{3.7, 9.95};
    FirstOrderFilter lane_width_certainty_{1.0, 0.95};
    double left_prob_ = 0.0;
    double right_prob_ = 0.0;
    double lane_change_scale_ = 1.0;  // 이번 프레임의 차선 변경 페이드(fade_for_lane_change)
    // 변화율 제한의 상태(페이드 전)와, 페이드를 곱한 출력
    double left_prob_limited_ = 0.0;
    double right_prob_limited_ = 0.0;
    double left_switch_limited_ = 0.0;
    double right_switch_limited_ = 0.0;
    double left_prob_eff_ = 0.0;
    double right_prob_eff_ = 0.0;
    double left_switch_prob_ = 0.0;
    double right_switch_prob_ = 0.0;
    std::array<double, kSwitchStdFrames> left_std_history_ = filled_history();
    std::array<double, kSwitchStdFrames> right_std_history_ = filled_history();
    size_t std_history_index_ = 0;
    double left_std_ = 0.0;
    double right_std_ = 0.0;
    double lane_width_ = 3.7;
    double d_prob_ = 0.0;
    LaneLineFilter left_filter_, right_filter_;
};

// MPC 노드 시각의 기준 횡위치와 heading.
struct MpcReferences {
    std::array<double, kLatMpcNodes> y{};
    std::array<double, kLatMpcNodes> heading{};
};

// 경로(차선 융합 뒤)에서 MPC 노드 시각의 목표를 차속 x 시간 거리로 보간한다. knot을 직접
// 인덱싱하면 모델 knot의 프레임 간 노이즈가 그대로 들어가 des가 2~3배 떨리고 토크 슬루 리미터가
// 요구 토크의 절반을 잘라낸다(0.8.x 재생 실측).
MpcReferences mpc_references(const std::array<std::array<double, 3>, kTrajectorySize> &path,
                             const std::array<double, kTrajectorySize> &path_t, float v_ego) {
    std::array<double, kTrajectorySize> distance{};
    std::array<double, kTrajectorySize> path_y{};
    std::array<double, kTrajectorySize> path_heading{};
    const bool plan_collapsed = path[kTrajectorySize - 1][0] < kMinPlanReachM;
    for (int i = 0; i < kTrajectorySize; ++i) {
        distance[i] = std::sqrt(path[i][0] * path[i][0] + path[i][1] * path[i][1] + path[i][2] * path[i][2]);
        path_y[i] = path[i][1];
        // 최종 경로와 heading을 같은 좌표계에서 계산한다. 차선 융합이나
        // 경로 오프셋 뒤에 모델 원본 heading을 재사용하면 좌우 곡률 부호가
        // 서로 달라져 한쪽 커브에서 경로를 안쪽으로 자를 수 있다.
        path_heading[i] = path_heading_at(path, i);
        // 정지 부근에서는 plan 전체가 몇 m로 붕괴해 기하학적 heading이 dy를
        // ±90도로 부풀리고, 양자화된 모델은 먼 knot이 0.5 m 이상 뒤로도 뛴다
        // (±180도 → 출발 시 좌측 급조향). 차가 못 움직인 구간이라 목표
        // heading은 현재 방위(0)가 맞다. 주행 중에도 역방향 step은 물리적으로
        // 불가능한 기하이므로 0으로 둔다.
        const int prev = std::max(0, i - 1), next = std::min(kTrajectorySize - 1, i + 1);
        if (plan_collapsed || path[next][0] <= path[prev][0]) path_heading[i] = 0.0;
    }
    make_monotonic(distance.data(), distance.size());

    MpcReferences references;
    for (int i = 0; i < kLatMpcNodes; ++i) {
        const double query = std::max(0.0f, v_ego) * path_t[i];
        references.y[i] = interp(query, distance.data(), path_y.data(), distance.size());
        references.heading[i] = interp(query, distance.data(), path_heading.data(), distance.size());
    }
    return references;
}

}  // namespace

struct LateralPlanner::Impl {
    explicit Impl(const SteeringParams &steering) { update_params(steering); }

    void update_params(const SteeringParams &steering) {
        lane_path_weight = steering.lane_path_weight;
        DesireHelperParams desire_params;
        // desire_helper의 torque_applied는 carstate.steeringPressed에서 나오므로
        // 컨트롤러와 같은 임계값을 써야 한다.
        desire_params.steering_pressed_threshold = steering.steering_pressed_threshold;
        desire_params.lane_change_min_speed_mps = steering.lane_change_min_speed_kph / 3.6;
        desire_helper.update_params(desire_params);
        laneless_mode = steering.laneless_mode;
        constexpr double wheelbase = kWheelbaseM, center_to_front = kCenterToFrontM;
        // 뒷바퀴 강성은 질량당 값이다. 상류 식 aF·m / (L·cR)에서 질량이 약분된다.
        const double tire_rear_per_mass =
            scale_tire_stiffness_per_mass(wheelbase, center_to_front, double{steering.tire_stiffness_factor}).second;
        factor1 = wheelbase - center_to_front;
        factor2 = center_to_front / (wheelbase * tire_rear_per_mass);
    }

    LateralTarget update(const ModelState &model, const VehicleCanState &vehicle, const LateralPlannerInput &in) {
        const float v_ego = in.v_ego, measured_curvature = in.measured_curvature;
        const bool lat_active = in.lat_active, steering_pressed = in.steering_pressed;
        LateralTarget target;
        lane_planner.parse(model, v_ego, measured_curvature);
        update_desire(model, vehicle, v_ego, lat_active, steering_pressed);
        std::array<std::array<double, 3>, kTrajectorySize> path{};
        std::array<double, kTrajectorySize> path_t{};
        for (int i = 0; i < kTrajectorySize; ++i) {
            path[i] = {model.plan[i].x, model.plan[i].y, model.plan[i].z};
            path_t[i] = model_t_idx(i);
        }

        lane_planner.update_probabilities(v_ego);
        if (laneless_mode) return upstream_target(model, v_ego, measured_curvature);
        // 여기부터는 Lane 모드다(laneless 모드는 위에서 upstream_target으로 끝난다).
        const bool use_model_path = update_path_source();
        const MpcReferences references = mpc_references(lane_planner.lane_path(path_t, path), path_t, v_ego);
        solve_lane_mpc(references, v_ego, measured_curvature, path_t);

        // 모델 경로 쪽 목표. 섞는 동안 MPC는 실제로 낸 곡률에서 다음 해를 시작해 되돌아올 때 튀지 않는다.
        const double model_weight = plan_mix;
        LateralTarget model_target;
        if (model_weight > 0.0) {
            model_target = plan_yaw_target(model, v_ego);
            std::array<double, kLateralControlN> times{}, model_curvatures{};
            for (int i = 0; i < kLateralControlN; ++i) {
                times[i] = model_t_idx_double(i);
                model_curvatures[i] = model_target.curvatures[i];
            }
            initial_curvature = (1.0 - model_weight) * initial_curvature +
                                model_weight * interp(kDtMdl, times.data(), model_curvatures.data(), times.size());
        }

        finish_target(model, model_weight >= 1.0 || invalid_count < 2, use_model_path,
                      use_model_path ? 0.0 : lane_planner.d_prob(), references.y[1], &target);
        for (int i = 0; i < kLateralControlN; ++i) {
            target.psis[i] = static_cast<float>(mpc.nodes()[i].psi);
            target.curvatures[i] = static_cast<float>(mpc.nodes()[i].curvature);
        }
        if (model_weight > 0.0) {
            // lag_adjusted_curvature는 psi와 곡률에 선형이라 입력을 섞으면 목표 곡률이 그대로 섞인다.
            const auto mix = [model_weight](float lane_value, float model_value) {
                return static_cast<float>((1.0 - model_weight) * lane_value + model_weight * model_value);
            };
            for (int i = 0; i < kLateralControlN; ++i) {
                target.psis[i] = mix(target.psis[i], model_target.psis[i]);
                target.curvatures[i] = mix(target.curvatures[i], model_target.curvatures[i]);
            }
            target.target_y_m = mix(target.target_y_m, model_target.target_y_m);
        }
        return target;
    }

    // 모델 프레임의 차선 변경 확률·차선·경계로 desire를 갱신하고, 변경 중이면 가까운 차선선을 뺀다.
    void update_desire(const ModelState &model, const VehicleCanState &vehicle, float v_ego, bool lat_active,
                       bool steering_pressed) {
        DesireModelInput desire_inputs;
        desire_inputs.lane_change_prob = model.desire_state[3] + model.desire_state[4];
        for (size_t i = 0; i < desire_inputs.lane_probabilities.size(); ++i)
            desire_inputs.lane_probabilities[i] = model.lane_probabilities[i];
        for (size_t i = 0; i < desire_inputs.road_edge_stds.size(); ++i)
            desire_inputs.road_edge_stds[i] = model.road_edge_stds[i];
        desire_helper.update(vehicle, v_ego, lat_active, steering_pressed, desire_inputs);
        if (desire_helper.changing_lanes()) lane_planner.fade_for_lane_change(desire_helper.lane_change_lane_prob());
        // 회전 중에는 차선선을 빼 Lane 모드에서도 회전 desire를 받은 모델 경로를 따른다(교차로는 원래 차선이 흐리다)
        if (desire_helper.turning()) lane_planner.drop_for_turn();
    }

    // Lane 모드에서 이번 프레임에 모델 경로를 따를지(차선이 안 보일 때) 정하고, 출력 인계 비율 plan_mix를
    // 그쪽으로 옮긴다.
    bool update_path_source() {
        const double lane_probability = lane_planner.mean_switch_probability();
        // 차선을 실제로 빼는 변경 단계(Starting, Finishing)에서는 모델 경로로 빠지지 않는다: 그동안 차선 확률을 줄여
        // 0.3 밑으로 떨어지기 때문이다. 깜빡이만 켠 단계(PreLaneChange)는 차선을 빼지 않으므로 평소 규칙을 따른다
        // (예전에는 이 단계에서도 대체를 꺼, 차선이 약한 출구 분기에서 깜빡이를 켜면 약한 차선이 경로에 섞였다).
        // 복귀 문턱을 openpilot의 0.5에서 0.4로 내렸다. 교차로 후 차선 재획득이 빨라지고, 진입(0.3)과의 밴드가 남아
        // 채터링은 없다.
        // 진입은 0.3초 늦춘다: 대체의 42~72 %가 1초 안에 끝났고, 그 순간에도 LKA14는 79~96 % 양쪽 선을 보고 있었다
        // (10월 주행 일곱). 대체 중 차로 중앙 오차 RMS는 18~26 cm로 차선 경로(8~11 cm)의 두 배가 넘고, 계획 밖 핸들
        // 조작 63번 중 57번이 대체 중이나 직후였다. 선이 아주 약하면(0.15 밑, 교차로) 기다리지 않는다.
        const LaneChangeState lane_change = desire_helper.lane_change_state();
        if (lane_change == LaneChangeState::Starting || lane_change == LaneChangeState::Finishing) {
            laneless_buffer = false;
            low_probability_frames = 0;
        } else if (lane_probability < kFallbackEnterProbability) {
            low_probability_frames = std::min(low_probability_frames + 1, kFallbackEnterFrames);
            if (low_probability_frames >= kFallbackEnterFrames || lane_probability < kFallbackImmediateProbability)
                laneless_buffer = true;
        } else {
            low_probability_frames = 0;
            if (lane_probability > kFallbackExitProbability) laneless_buffer = false;
        }
        const bool use_model_path = laneless_buffer;
        // 모델 경로 구간(교차로처럼 차선이 안 보일 때)은 laneless 모드와 같은 openpilot
        // 메인 계산(plan_yaw_target)으로 목표를 낸다. 예전에는 모델 경로 위치를 Lane MPC로 따라갔는데,
        // 경로를 "지금 속도 x 시간" 거리에서 읽고 MPC가 곡률 변화를 눌러 25 km/h 아래에서 laneless보다
        // 26~30% 덜 꺾고 0.3~0.4초 늦었다(2026-10-05 재생; 실주행 저속 회전의 운전자 토크 중앙값
        // 152 vs laneless 80). MPC는 늘 차선 경로를 따르고, 둘 사이의 인계만 출력(psi·곡률)에서 섞는다.
        // 전환 임계값(유효확률 0.3)에서는 차선 블렌드 가중치가 아직 0.51이라 한 프레임에 바꾸면 튄다
        // (2026-09-13 실측, 전환 434회). 차선을 버리는 방향은 0.5초, 되찾는 방향은 0.25초에 섞는다.
        const double mix_target = use_model_path ? 1.0 : 0.0;
        const double mix_step = (mix_target > plan_mix ? kPlanMixOutRatePerS : kPlanMixInRatePerS) * kDtMdl;
        plan_mix = std::clamp(mix_target, plan_mix - mix_step, plan_mix + mix_step);
        return use_model_path;
    }

    // 차선 MPC를 references로 풀고 다음 사이클의 초기 곡률과 연속 실패 수(invalid_count)를 갱신한다.
    // 풀이가 실패하면 MPC를 비우고 실제 곡률에서 다시 시작한다.
    void solve_lane_mpc(const MpcReferences &references, float v_ego, float measured_curvature,
                        const std::array<double, kTrajectorySize> &path_t) {
        const double lateral_factor = std::max(0.0, factor1 - factor2 * v_ego * v_ego);
        // lane 모드도 laneless와 같은 스케줄. heading은 차선 경로에서 나오므로
        // 고속에서 heading 고정 1.0이면 횡 offset에 대한 DC 강성이 0이 되어
        // 커브에서 바깥쪽 0.3~0.5m 평형이 생긴다(2026-08-25 실측: 요구곡률
        // 3.4% 부족 + 바깥 offset +0.375m).
        const double heading_weight = v_ego <= 5.0f ? 1.0 : v_ego >= 10.0f ? 0.15 : 1.0 - (v_ego - 5.0) * 0.17;
        LateralMpcWeights weights;
        weights.path = lane_path_weight;
        weights.heading = heading_weight;
        // 출발 곡률은 직전 해의 곡률(0.8.x)에 잰 곡률을 kMeasuredCurvatureSeed만큼 섞는다(0.7.x는 잰 곡률 그대로).
        // 직전 해에서만 출발하면 그 오차가 다음 해로 넘어가, 중앙으로 되돌리는 힘이 조향 지연이 길수록 줄었다(지연
        // 0.55 s에서 0.96, 0.70 s에서 0.44: lagd 0.70 s 재생에서 곡선 RMS가 두 배 가까이 된 것도 이것으로 설명된다).
        // 그래서 차가 요구보다 조금만 더 돌아도 곡선 안쪽으로 붙은 채 남았다.
        initial_curvature =
            kMeasuredCurvatureSeed * measured_curvature + (1.0 - kMeasuredCurvatureSeed) * initial_curvature;
        mpc.run(initial_curvature, std::max(0.0f, v_ego), lateral_factor, references.y, references.heading, weights);
        const bool solver_failed = mpc.status() != 0;
        if (solver_failed) {
            mpc.reset();
            initial_curvature = measured_curvature;
        } else {
            std::array<double, kLatMpcNodes> curvatures{};
            for (int i = 0; i < kLatMpcNodes; ++i) curvatures[i] = mpc.nodes()[i].curvature;
            initial_curvature = interp(kDtMdl, path_t.data(), curvatures.data(), curvatures.size());
        }
        invalid_count = (mpc.cost() > 20000.0 || solver_failed) ? invalid_count + 1 : 0;
    }

    // 목표의 메타데이터: 근거 프레임, MPC 유효, 경로 출처, 차선 관측(로그·HUD용: 가까운 좌우 차선, 폭, 모델 원 확률과
    // std), 차선 가중치, 목표 위치, 이번 프레임의 desire와 차선 변경 단계.
    void finish_target(const ModelState &model, bool mpc_valid, bool use_model_path, double d_prob, double target_y,
                       LateralTarget *target) const {
        target->valid = true;
        target->capture_timestamp_ns = model.capture_timestamp_ns;
        target->mpc_solution_valid = mpc_valid;
        target->laneless_mode = use_model_path;
        target->lane_left_y_m = static_cast<float>(lane_planner.near_left_y());
        target->lane_right_y_m = static_cast<float>(lane_planner.near_right_y());
        target->lane_width_m = static_cast<float>(lane_planner.lane_width());
        target->lane_left_prob = static_cast<float>(lane_planner.left_prob());
        target->lane_right_prob = static_cast<float>(lane_planner.right_prob());
        target->lane_left_std = static_cast<float>(lane_planner.left_std());
        target->lane_right_std = static_cast<float>(lane_planner.right_std());
        target->lane_d_prob = static_cast<float>(d_prob);
        target->target_y_m = static_cast<float>(target_y);
        target->desire = desire_helper.desire();
        target->lane_change_state = desire_helper.lane_change_state();
        target->lane_change_direction = desire_helper.direction();
    }

    // laneless 모드 = openpilot 메인의 get_curvature_from_plan. 차선·MPC·경로 오프셋 없이
    // 모델 plan의 yaw와 yaw rate를 그대로 목표로 넘긴다. 컨트롤러의 lag_adjusted_curvature가
    // 같은 식 2·ψ(t_d)/(v·t_d) − ψ̇(0)/v를 t_d = 조향 지연 + 실측 plan 나이에서 적용한다
    // (메인은 t_d = lateralDelay + 프레임 지연 50 ms + 25 ms). 경로 오프셋은 laneless에서
    // 위치가 아니라 꾸준한 곡률 편향이 되므로 쓰지 않는다.
    LateralTarget upstream_target(const ModelState &model, float v_ego, float measured_curvature) {
        LateralTarget target = plan_yaw_target(model, v_ego);
        // 차선 모드로 돌아가면 낡은 MPC 해가 아니라 지금 곡률에서 출발한다.
        laneless_buffer = false;
        low_probability_frames = 0;
        plan_mix = 1.0;
        mpc.reset();
        initial_curvature = measured_curvature;
        invalid_count = 0;
        return target;
    }

    // openpilot 메인 get_curvature_from_plan 목표. 상태를 바꾸지 않아 Lane 모드의 모델 경로 구간도 쓴다.
    LateralTarget plan_yaw_target(const ModelState &model, float v_ego) const {
        constexpr double kMinSpeed = 1.0;  // openpilot drive_helpers.MIN_SPEED
        const double speed = std::max<double>(v_ego, kMinSpeed);
        std::array<double, kTrajectorySize> t{}, yaw{}, yaw_rate{}, y{};
        for (int i = 0; i < kTrajectorySize; ++i) {
            t[i] = model_t_idx(i);  // 고정된 증가 시각 격자
            yaw[i] = model.plan_yaw[i];
            yaw_rate[i] = model.plan_yaw_rate[i];
            y[i] = model.plan[i].y;
        }
        LateralTarget target;
        for (int i = 0; i < kLateralControlN; ++i) {
            const double ti = model_t_idx_double(i);
            target.psis[i] = static_cast<float>(interp(ti, t.data(), yaw.data(), t.size()));
            target.curvatures[i] = static_cast<float>(interp(ti, t.data(), yaw_rate.data(), t.size()) / speed);
        }
        // MPC를 쓰지 않는다
        finish_target(model, true, true, 0.0, interp(model_t_idx_double(1), t.data(), y.data(), t.size()), &target);
        return target;
    }

    LanePlanner lane_planner;
    LateralMpc mpc;
    // 다음 사이클의 초기 curvature. 나머지 초기 상태는 자차 기준 0이다.
    double initial_curvature = 0.0;
    double factor1 = 0.0;
    double factor2 = 0.0;
    double lane_path_weight = 3.0;
    bool laneless_mode = false;
    DesireHelper desire_helper;
    bool laneless_buffer = false;
    int low_probability_frames = 0;  // 판정 확률이 kFallbackEnterProbability 밑에 머문 프레임(진입 지연)
    // 0 = 차선 융합 경로, 1 = 모델 플랜. 전환 판정을 그대로 따라가되 램프로 움직인다.
    double plan_mix = 1.0;
    int invalid_count = 0;
};

LateralPlanner::LateralPlanner(const SteeringParams &params) : impl_(std::make_unique<Impl>(params)) {}

LateralPlanner::~LateralPlanner() = default;

void LateralPlanner::update_params(const SteeringParams &params) { impl_->update_params(params); }

void LateralPlanner::set_lane_filter_weight(double weight) { impl_->lane_planner.set_filter_weight(weight); }

LateralTarget LateralPlanner::update(const ModelState &model, const VehicleCanState &vehicle,
                                     const LateralPlannerInput &in) {
    return impl_->update(model, vehicle, in);
}
