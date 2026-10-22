#include "learners/vehicle_params_learner.h"

#include "car/can_frame.h"
#include "car/car_specs.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "common/utils_json.h"
#include "common/utils_math.h"
#include "common/utils_time.h"  // kDtMdl

namespace {

constexpr double kGravity = 9.81;
double rad(double deg) { return deg * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }

// paramsd.py
constexpr double kMaxAngleOffsetDelta = 20.0 * kDtMdl;  // deg/출력
constexpr double kRollMaxDelta = 20.0 * kPi / 180.0 * kDtMdl;
constexpr double kRollMin = -10.0 * kPi / 180.0;
constexpr double kRollMax = 10.0 * kPi / 180.0;
constexpr double kRollLoweredMax = 8.0 * kPi / 180.0;
constexpr double kRollStdMax = 1.5 * kPi / 180.0;
constexpr double kLateralAccSensorThreshold = 4.0;
constexpr double kOffsetMax = 10.0;
constexpr double kOffsetLoweredMax = 8.0;
constexpr double kMinActiveSpeed = 1.0;
constexpr double kLowActiveSpeed = 10.0;
constexpr int kPersistEveryFrames = 1200;

// ESP12 입력. 요레이트 std는 VehicleParamsInput 기본값, 롤은 횡가속 유도값이라 거의
// 정상상태(|u·r| 작음)에서만 관측한다.
constexpr double kRollStd = 1.0 * kPi / 180.0;
constexpr double kRollMinSpeed = 5.0;
constexpr double kRollMaxCentripetal = 0.5;
constexpr double kPredictStepS = 0.001;
constexpr int kPersistVersion = 1;

bool check_valid_with_hysteresis(bool current_valid, double val, double threshold, double lowered_threshold) {
    return std::fabs(val) < (current_valid ? threshold : lowered_threshold);
}

double clip(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

bool near_constant(double a, double b) { return std::fabs(a - b) <= 1e-3 * std::max(1.0, std::fabs(b)); }

}  // namespace

// ---------------------------------------------------------------- CarKalman

CarKalman::Vec CarKalman::initial_x() { return {1.0, 15.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0}; }

CarKalman::Mat CarKalman::q() {
    const double d[kN] = {std::pow(0.05 / 100.0, 2), std::pow(0.01, 2),     std::pow(rad(0.02), 2),
                          std::pow(rad(0.25), 2),    std::pow(0.1, 2),      std::pow(0.01, 2),
                          std::pow(rad(0.1), 2),     std::pow(rad(0.1), 2), std::pow(rad(1.0), 2)};
    Mat m{};
    for (int i = 0; i < kN; ++i) m[i][i] = d[i];
    return m;
}

CarKalman::Globals CarKalman::globals_from(const VehicleModelConstants &c) {
    Globals g;
    const double l = c.wheelbase_m;
    g.center_to_front = c.center_to_front_m;
    g.center_to_rear = l - c.center_to_front_m;
    // opendbc scale_rot_inertia_per_mass, scale_tire_stiffness_per_mass
    g.inertia = kCivicInertia * l * l / (kCivicMass * kCivicWheelbase * kCivicWheelbase);
    const auto stiffness = scale_tire_stiffness_per_mass(l, c.center_to_front_m, c.tire_stiffness_factor);
    g.stiffness_front = stiffness.first;
    g.stiffness_rear = stiffness.second;
    return g;
}

// Guiggiani 7.211-7.213 선형 단일트랙. 횡속도·요레이트만 움직이고 나머지는 랜덤워크라 F = I + hJ에서 J는
// kSpeedY·kYawRate 행만 0이 아니다(predict가 이것을 쓴다).
void CarKalman::step(const Globals &g, const Vec &x, double h, Vec *x_next, Mat *f) {
    const double sf = x[kStiffness], sr = x[kSteerRatio], u = x[kSpeedX];
    const double v = x[kSpeedY], r = x[kYawRate], th = x[kRoadRoll];
    const double e = x[kSteerAngle] - x[kAngleOffset] - x[kAngleOffsetFast];
    // 관성과 강성이 질량당 값이라 단위 질량으로 같은 식을 쓴다(상류 car_kf m = 1)
    const double m = 1.0, j = g.inertia, af = g.center_to_front, ar = g.center_to_rear;
    const double cf0 = g.stiffness_front, cr0 = g.stiffness_rear;
    const double csum = cf0 + cr0, d1 = cf0 * af - cr0 * ar, d2 = cf0 * af * af + cr0 * ar * ar;

    const double a00 = -sf * csum / (m * u), a01 = -sf * d1 / (m * u) - u;
    const double a10 = -sf * d1 / (j * u), a11 = -sf * d2 / (j * u);
    const double b0 = sf * cf0 / (m * sr), b1 = sf * cf0 * af / (j * sr);

    *x_next = x;
    (*x_next)[kSpeedY] += h * (a00 * v + a01 * r + b0 * e - kGravity * th);
    (*x_next)[kYawRate] += h * (a10 * v + a11 * r + b1 * e);

    Mat jac{};
    jac[kSpeedY][kStiffness] = (-csum / (m * u)) * v + (-d1 / (m * u)) * r + (cf0 / (m * sr)) * e;
    jac[kSpeedY][kSteerRatio] = -b0 * e / sr;
    jac[kSpeedY][kAngleOffset] = -b0;
    jac[kSpeedY][kAngleOffsetFast] = -b0;
    jac[kSpeedY][kSpeedX] = (sf * csum / (m * u * u)) * v + (sf * d1 / (m * u * u) - 1.0) * r;
    jac[kSpeedY][kSpeedY] = a00;
    jac[kSpeedY][kYawRate] = a01;
    jac[kSpeedY][kSteerAngle] = b0;
    jac[kSpeedY][kRoadRoll] = -kGravity;
    jac[kYawRate][kStiffness] = (-d1 / (j * u)) * v + (-d2 / (j * u)) * r + (cf0 * af / (j * sr)) * e;
    jac[kYawRate][kSteerRatio] = -b1 * e / sr;
    jac[kYawRate][kAngleOffset] = -b1;
    jac[kYawRate][kAngleOffsetFast] = -b1;
    jac[kYawRate][kSpeedX] = (sf * d1 / (j * u * u)) * v + (sf * d2 / (j * u * u)) * r;
    jac[kYawRate][kSpeedY] = a10;
    jac[kYawRate][kYawRate] = a11;
    jac[kYawRate][kSteerAngle] = b1;
    for (int i = 0; i < kN; ++i)
        for (int k = 0; k < kN; ++k) (*f)[i][k] = (i == k ? 1.0 : 0.0) + h * jac[i][k];
}

void CarKalman::init(const Vec &x, const Mat &p, bool has_time, double t) {
    x_ = x;
    p_ = p;
    has_time_ = has_time;
    t_ = t;
}

void CarKalman::predict(double dt) {
    static const Mat qm = q();
    const int steps = std::max(1, static_cast<int>(std::ceil(dt / kPredictStepS - 1e-9)));
    const double h = dt / steps;
    for (int s = 0; s < steps; ++s) {
        Vec xn;
        Mat f;
        step(g_, x_, h, &xn, &f);
        // P ← F·P·Fᵀ + hQ. F의 다른 행은 단위 벡터라 그 행(F·P)과 열((F·P)·Fᵀ)은 곱해도 그대로다. 움직이는 두
        // 행과 두 열만 전체 곱과 같은 순서로 내적하므로 결과가 비트 단위로 같고, 곱셈은 1,458번에서 324번이 된다.
        constexpr int kMoving[] = {kSpeedY, kYawRate};
        Mat fp = p_;
        for (const int i : kMoving)
            for (int k = 0; k < kN; ++k) {
                double acc = 0.0;
                for (int l = 0; l < kN; ++l) acc += f[i][l] * p_[l][k];
                fp[i][k] = acc;
            }
        for (int i = 0; i < kN; ++i) {
            for (int k = 0; k < kN; ++k) p_[i][k] = fp[i][k] + h * qm[i][k];
            for (const int k : kMoving) {
                double acc = 0.0;
                for (int l = 0; l < kN; ++l) acc += fp[i][l] * f[k][l];
                p_[i][k] = acc + h * qm[i][k];
            }
        }
        x_ = xn;
    }
}

// Joseph 형식(rednose): P = (I−KH)P(I−KH)ᵀ + K R Kᵀ, H = e_state.
void CarKalman::predict_and_observe(double t, int state, double z, double r) {
    if (!has_time_) {
        set_time(t);
    } else if (t > t_) {
        predict(t - t_);
        t_ = t;
    }
    const double s = p_[state][state] + r;
    Vec k;
    for (int i = 0; i < kN; ++i) k[i] = p_[i][state] / s;
    const double y = z - x_[state];
    for (int i = 0; i < kN; ++i) x_[i] += k[i] * y;
    Mat a;  // (I − K e_sᵀ) P
    for (int i = 0; i < kN; ++i)
        for (int c = 0; c < kN; ++c) a[i][c] = p_[i][c] - k[i] * p_[state][c];
    for (int i = 0; i < kN; ++i)
        for (int c = 0; c < kN; ++c) p_[i][c] = a[i][c] - a[i][state] * k[c] + r * k[i] * k[c];
}

// ---------------------------------------------------------------- VehicleParamsLearner

VehicleParamsLearner::VehicleParamsLearner(const VehicleModelConstants &c, double steer_ratio, double stiffness_factor,
                                           double angle_offset_rad, const VehicleParamsOptions &options)
    : c_(c),
      car_state_every_tick_(options.car_state_every_tick),
      min_sr_(0.5 * c.steer_ratio),
      max_sr_(2.0 * c.steer_ratio),
      kf_(CarKalman::globals_from(c)) {
    x_initial_ = CarKalman::initial_x();
    x_initial_[CarKalman::kSteerRatio] = steer_ratio;
    x_initial_[CarKalman::kStiffness] = stiffness_factor;
    x_initial_[CarKalman::kAngleOffset] = angle_offset_rad;
    p_initial_ = options.p_initial ? *options.p_initial : CarKalman::q();
    reset(false, 0.0);
}

void VehicleParamsLearner::reset(bool has_time, double t) {
    kf_.init(x_initial_, p_initial_, has_time, t);
    angle_offset_deg_ = deg(x_initial_[CarKalman::kAngleOffset]);
    roll_ = 0.0;
    active_ = false;
    avg_angle_offset_deg_ = angle_offset_deg_;
}

void VehicleParamsLearner::handle_car_state(const VehicleParamsInput &in) {
    const bool in_linear_region = std::fabs(in.steering_angle_deg) < 45.0;
    observed_speed_ = in.speed_mps;
    active_ = observed_speed_ > kMinActiveSpeed && in_linear_region && in.gear != kGearReverse;
    if (active_) {
        kf_.predict_and_observe(in.t_s, CarKalman::kSteerAngle, rad(in.steering_angle_deg), std::pow(rad(0.05), 2));
        kf_.predict_and_observe(in.t_s, CarKalman::kSpeedX, observed_speed_, std::pow(0.1, 2));
    }
}

void VehicleParamsLearner::handle_device_motion(const VehicleParamsInput &in) {
    double yaw_rate = in.yaw_rate_rad_s, yaw_rate_std = in.yaw_rate_std_rad_s;
    const bool yaw_rate_valid =
        in.yaw_rate_valid && yaw_rate_std > 0.0 && yaw_rate_std < 10.0 && std::fabs(yaw_rate) < 1.0;
    if (!yaw_rate_valid) {
        yaw_rate = 0.0;
        yaw_rate_std = rad(10.0);
    }
    observed_yaw_rate_ = yaw_rate;

    const double centripetal = in.speed_mps * yaw_rate;
    double roll = 0.0, roll_std = rad(10.0);
    bool localizer_roll = false;
    if (in.localizer_roll_given && in.localizer_roll_valid) {
        // 상류 paramsd: 자세 롤(표준편차 nan이면 1°)을 표준편차 2배로 관측
        const double loc_std = std::isfinite(in.localizer_roll_std_rad) ? in.localizer_roll_std_rad : rad(1.0);
        if (loc_std < kRollStdMax && in.localizer_roll_rad > kRollMin && in.localizer_roll_rad < kRollMax) {
            roll = in.localizer_roll_rad;
            roll_std = 2.0 * loc_std;
            localizer_roll = true;
        }
    }
    // 상류는 자세 롤이 무효면 0(10°)을 관측한다. 그대로 두면 롤 std가 1.49°(유효 한계 1.5°)에
    // 붙어 제어 틱 하나만 늦어도 무효 → Hard 해제가 된다(상류는 soft disable). 그 틱은 ESP12
    // 횡가속 롤로 대신한다.
    if (!localizer_roll && in.lat_accel_valid && in.speed_mps > kRollMinSpeed &&
        std::fabs(centripetal) < kRollMaxCentripetal) {
        const double candidate = std::asin(clip((in.lat_accel_mps2 - centripetal) / kGravity, -1.0, 1.0));
        if (candidate > kRollMin && candidate < kRollMax) {  // kRollStd는 유효 한계(kRollStdMax) 안이다
            roll = candidate;
            roll_std = kRollStd;
        }
    }
    observed_roll_ = clip(roll, observed_roll_ - kRollMaxDelta, observed_roll_ + kRollMaxDelta);

    if (active_) {
        // 상류는 −자세 요레이트를 관측한다. ESP12는 이미 좌측 양수라 그대로.
        kf_.predict_and_observe(in.t_s, CarKalman::kYawRate, observed_yaw_rate_, yaw_rate_std * yaw_rate_std);
        kf_.predict_and_observe(in.t_s, CarKalman::kRoadRoll, observed_roll_, roll_std * roll_std);
        kf_.predict_and_observe(in.t_s, CarKalman::kAngleOffsetFast, 0.0, std::pow(rad(10.0), 2));
        // 자기관측: 값을 당기지 않고 긴 직선에서 공분산이 무한히 커지는 것만 막는다.
        const double stiffness = kf_.x()[CarKalman::kStiffness];
        const double steer_ratio = kf_.x()[CarKalman::kSteerRatio];
        kf_.predict_and_observe(in.t_s, CarKalman::kStiffness, stiffness, 0.5 * 0.5);
        kf_.predict_and_observe(in.t_s, CarKalman::kSteerRatio, steer_ratio, 5.0 * 5.0);
    }
}

bool VehicleParamsLearner::update(const VehicleParamsInput &in) {
    persist_due_ = false;
    // 요레이트·롤 관측과 출력은 deviceMotion처럼 20 Hz. 조향각·속도는 기본 매 틱이고,
    // 상류 스케줄이면 그 20 Hz 틱의 최신값 하나만 본다(conflate).
    const bool motion_due = !has_motion_t_ || in.t_s - last_motion_t_ >= kDtMdl - 1e-3;
    if (!motion_due && !car_state_every_tick_) return false;
    if (in.inputs_fresh) {
        handle_car_state(in);
        if (!active_) kf_.set_time(in.t_s);
        if (motion_due) {
            handle_device_motion(in);
            if (!active_) kf_.set_time(in.t_s);
        }
    } else {
        // 상류는 낡은 입력을 버리고 시각도 멈춘다. 재개 시 긴 dt 예측이 발산하므로 비활성처럼 둔다.
        kf_.set_time(in.t_s);
    }
    if (!motion_due) return false;
    has_motion_t_ = true;
    last_motion_t_ = in.t_s;
    ++motion_frame_;
    last_ = get_msg(in.inputs_fresh);
    persist_due_ = motion_frame_ % kPersistEveryFrames == 0;
    return true;
}

VehicleParams VehicleParamsLearner::get_msg(bool inputs_ok) {
    bool finite = true;
    for (double v : kf_.x()) finite = finite && std::isfinite(v);
    if (!finite) {
        std::fprintf(stderr, "vehicle params: NaN in estimate, resetting\n");
        reset(true, kf_.time());
    }
    const CarKalman::Vec &x = kf_.x();
    const CarKalman::Mat &p = kf_.p();

    avg_angle_offset_deg_ = clip(deg(x[CarKalman::kAngleOffset]), avg_angle_offset_deg_ - kMaxAngleOffsetDelta,
                                 avg_angle_offset_deg_ + kMaxAngleOffsetDelta);
    angle_offset_deg_ = clip(deg(x[CarKalman::kAngleOffset] + x[CarKalman::kAngleOffsetFast]),
                             angle_offset_deg_ - kMaxAngleOffsetDelta, angle_offset_deg_ + kMaxAngleOffsetDelta);
    roll_ = clip(x[CarKalman::kRoadRoll], roll_ - kRollMaxDelta, roll_ + kRollMaxDelta);
    const double roll_std = std::sqrt(p[CarKalman::kRoadRoll][CarKalman::kRoadRoll]);

    bool sensors_valid = true;
    if (active_ && observed_speed_ > kLowActiveSpeed) {
        // 상류는 두 요레이트의 부호가 반대라 더한다. 여기서는 같은 부호라 뺀다.
        sensors_valid =
            std::fabs(observed_speed_ * (x[CarKalman::kYawRate] - observed_yaw_rate_)) < kLateralAccSensorThreshold;
    }
    avg_offset_valid_ =
        check_valid_with_hysteresis(avg_offset_valid_, avg_angle_offset_deg_, kOffsetMax, kOffsetLoweredMax);
    total_offset_valid_ =
        check_valid_with_hysteresis(total_offset_valid_, angle_offset_deg_, kOffsetMax, kOffsetLoweredMax);
    roll_valid_ = check_valid_with_hysteresis(roll_valid_, roll_, kRollMax, kRollLoweredMax);

    VehicleParams out;
    out.inputs_ok = inputs_ok;
    out.sensor_valid = sensors_valid;
    out.steer_ratio = x[CarKalman::kSteerRatio];
    out.stiffness_factor = x[CarKalman::kStiffness];
    out.roll_rad = roll_;
    out.angle_offset_average_deg = avg_angle_offset_deg_;
    out.angle_offset_deg = angle_offset_deg_;
    out.steer_ratio_valid = min_sr_ <= out.steer_ratio && out.steer_ratio <= max_sr_;
    out.stiffness_factor_valid = 0.2 <= out.stiffness_factor && out.stiffness_factor <= 5.0;
    out.angle_offset_average_valid = avg_offset_valid_;
    out.angle_offset_valid = total_offset_valid_;
    out.valid = out.angle_offset_average_valid && out.angle_offset_valid && roll_valid_ && roll_std < kRollStdMax &&
                out.stiffness_factor_valid && out.steer_ratio_valid;
    out.steer_ratio_std = std::sqrt(p[CarKalman::kSteerRatio][CarKalman::kSteerRatio]);
    out.stiffness_factor_std = std::sqrt(p[CarKalman::kStiffness][CarKalman::kStiffness]);
    out.angle_offset_average_std = std::sqrt(p[CarKalman::kAngleOffset][CarKalman::kAngleOffset]);
    out.angle_offset_fast_std = std::sqrt(p[CarKalman::kAngleOffsetFast][CarKalman::kAngleOffsetFast]);
    return out;
}

// ---------------------------------------------------------------- 저장/복원

std::string vehicle_params_persist_data(const VehicleParams &p, const VehicleModelConstants &c, double yaw_bias_rad_s) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\n  \"version\": %d,\n  \"wheelbase_m\": %.6g,\n"
                  "  \"center_to_front_m\": %.6g,\n  \"tire_stiffness_factor\": %.6g,\n"
                  "  \"steer_ratio\": %.8g,\n  \"stiffness_factor\": %.8g,\n"
                  "  \"angle_offset_average_deg\": %.8g,\n  \"yaw_bias_rad_s\": %.8g\n}\n",
                  kPersistVersion, c.wheelbase_m, c.center_to_front_m, c.tire_stiffness_factor, p.steer_ratio,
                  p.stiffness_factor, p.angle_offset_average_deg, yaw_bias_rad_s);
    return buf;
}

VehicleParamsInit restore_vehicle_params(const std::string &json, const VehicleModelConstants &c) {
    VehicleParamsInit init;
    init.steer_ratio = c.steer_ratio;
    JsonValue root;
    float version = 0, wheelbase = 0, cf = 0, tsf = 0, sr = 0, sf = 0, offset = 0, bias = 0;
    // 2026-10-22까지의 파일에는 mass_kg도 있다. 질량은 차량 모델에서 약분돼(상류도 지웠다) 읽지 않는다.
    const bool parsed = parse_json_object(json, &root) && json_float(root, "version", &version) &&
                        json_float(root, "wheelbase_m", &wheelbase) && json_float(root, "center_to_front_m", &cf) &&
                        json_float(root, "tire_stiffness_factor", &tsf) && json_float(root, "steer_ratio", &sr) &&
                        json_float(root, "stiffness_factor", &sf) &&
                        json_float(root, "angle_offset_average_deg", &offset) &&
                        json_float(root, "yaw_bias_rad_s", &bias);
    // 상류 지문 대조 자리. 단일 차종이라 차량 상수가 같으면 같은 차로 본다.
    const bool same_car = parsed && static_cast<int>(version) == kPersistVersion &&
                          near_constant(wheelbase, c.wheelbase_m) && near_constant(cf, c.center_to_front_m) &&
                          near_constant(tsf, c.tire_stiffness_factor);
    // JSON에는 유한한 수만 있다(아니면 parse_json_object가 거부한다)
    const bool sane = same_car && 0.5 * c.steer_ratio <= sr && sr <= 2.0 * c.steer_ratio;
    if (!sane) return init;
    init.restored = true;
    init.steer_ratio = sr;
    init.angle_offset_deg = offset;
    init.yaw_bias_rad_s = bias;
    init.stiffness_factor = 1.0;
    return init;
}
