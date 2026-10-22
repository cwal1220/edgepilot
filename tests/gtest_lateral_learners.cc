// paramsd·torqued 이식 검사: 필터 수학, 합성 주행 수렴, 게이트, 출력 제한, 저장/복원.
#include "car/hyundai_can.h"
#include "car/vehicle_can.h"
#include "common/ipc_messages.h"
#include "common/utils_time.h"
#include "learners/lateral_learners.h"
#include "learners/localizer_inputs.h"

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }

VehicleModelConstants k7() {
    VehicleModelConstants c;
    c.wheelbase_m = 2.855;
    c.center_to_front_m = 2.855 * 0.4;
    c.tire_stiffness_factor = 1.0;
    c.steer_ratio = 16.8;
    return c;
}

TEST(LateralLearners, Jacobian) {
    const CarKalman::Globals g = CarKalman::globals_from(k7());
    const CarKalman::Vec x = {1.05, 15.5, 0.01, 0.002, 15.0, 0.1, 0.05, 0.03, 0.01};
    CarKalman::Vec xn;
    CarKalman::Mat f;
    CarKalman::step(g, x, 0.01, &xn, &f);
    double worst = 0.0;
    for (int k = 0; k < CarKalman::kN; ++k) {
        const double h = 1e-6 * std::fmax(1.0, std::fabs(x[k]));
        CarKalman::Vec xp = x, xm = x, fp, fm;
        CarKalman::Mat dummy;
        xp[k] += h;
        xm[k] -= h;
        CarKalman::step(g, xp, 0.01, &fp, &dummy);
        CarKalman::step(g, xm, 0.01, &fm, &dummy);
        for (int i = 0; i < CarKalman::kN; ++i)
            worst = std::fmax(worst, std::fabs((fp[i] - fm[i]) / (2 * h) - f[i][k]));
    }
    ASSERT_LT(worst, 1e-7) << "해석적 야코비안이 중심차분과 같다";
}

bool positive_definite(const CarKalman::Mat &p) {
    double l[CarKalman::kN][CarKalman::kN] = {};
    for (int i = 0; i < CarKalman::kN; ++i) {
        for (int j = 0; j <= i; ++j) {
            double s = p[i][j];
            for (int k = 0; k < j; ++k) s -= l[i][k] * l[j][k];
            if (i == j) {
                if (s <= 0.0) return false;
                l[i][i] = std::sqrt(s);
            } else {
                l[i][j] = s / l[j][j];
            }
        }
    }
    return true;
}

// 같은 모델을 참값으로 1 ms 적분해 조향각·속도·요레이트·횡가속을 만든다.
struct Truth {
    double sr = 15.2, stiffness = 0.9, offset = rad(-1.6), roll = rad(0.5);
};

struct SynthSample {
    double t, angle_deg, speed, yaw, lat;
};

template <class F>
void synthesize(const Truth &truth, double seconds, F &&emit) {
    CarKalman::Globals g = CarKalman::globals_from(k7());
    CarKalman::Vec x = CarKalman::initial_x();
    x[CarKalman::kStiffness] = truth.stiffness;
    x[CarKalman::kSteerRatio] = truth.sr;
    x[CarKalman::kAngleOffset] = truth.offset;
    x[CarKalman::kAngleOffsetFast] = 0.0;
    x[CarKalman::kRoadRoll] = truth.roll;
    const double fine = 0.001;
    const int per_tick = 10;
    const int ticks = static_cast<int>(seconds * 100.0);
    for (int n = 0; n < ticks; ++n) {
        const double t = n * 0.01;
        // 5~25 m/s를 천천히 오가고, 직진 구간과 굽은 구간을 섞는다.
        const double speed = 15.0 + 10.0 * std::sin(2 * kPi * t / 240.0);
        const double weave = std::sin(2 * kPi * t / 90.0) > 0.0 ? 1.0 : 0.15;
        const double effective =
            weave * (rad(18.0) * std::sin(2 * kPi * t / 7.0) + rad(9.0) * std::sin(2 * kPi * t / 2.3));
        // 측정은 구간 시작의 상태로 찍는다. 적분 뒤 값을 붙이면 요레이트가 조향보다 10 ms
        // 앞서 보여 필터가 강성을 올려 읽는다.
        x[CarKalman::kSpeedX] = speed;
        x[CarKalman::kSteerAngle] = effective + truth.offset;
        CarKalman::Vec xn;
        CarKalman::Mat f;
        CarKalman::step(g, x, fine, &xn, &f);
        const double vdot = (xn[CarKalman::kSpeedY] - x[CarKalman::kSpeedY]) / fine;
        const double lat = vdot + speed * x[CarKalman::kYawRate] + 9.81 * truth.roll;
        emit(SynthSample{t, deg(effective + truth.offset), speed, x[CarKalman::kYawRate], lat});
        for (int k = 0; k < per_tick; ++k) {
            CarKalman::step(g, x, fine, &xn, &f);
            x = xn;
        }
    }
}

VehicleParamsInput input_from(const SynthSample &s) {
    VehicleParamsInput in;
    in.t_s = s.t;
    in.inputs_fresh = true;
    in.steering_angle_deg = s.angle_deg;
    in.speed_mps = s.speed;
    in.gear = 5;
    in.yaw_rate_valid = true;
    in.yaw_rate_rad_s = s.yaw;
    in.lat_accel_valid = true;
    in.lat_accel_mps2 = s.lat;
    return in;
}

CarKalman::Mat open_p0() {
    CarKalman::Mat p0 = CarKalman::q();
    p0[CarKalman::kSteerRatio][CarKalman::kSteerRatio] = 3.0 * 3.0;
    p0[CarKalman::kStiffness][CarKalman::kStiffness] = 0.3 * 0.3;
    p0[CarKalman::kAngleOffset][CarKalman::kAngleOffset] = rad(2.0) * rad(2.0);
    return p0;
}

// 매 틱 관측이면 필터 수학만 남는다. 빠른 영점이 조향의 일부를 흡수해 SR은 약 1% 낮다.
TEST(LateralLearners, ConvergesAndStaysPositiveDefinite) {
    const CarKalman::Mat p0 = open_p0();
    VehicleParamsOptions options;
    options.p_initial = &p0;
    options.car_state_every_tick = true;
    VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0, options);
    const Truth truth;
    bool always_pd = true, always_sensor_valid = true;
    int publishes = 0;
    synthesize(truth, 900.0, [&](const SynthSample &s) {
        if (learner.update(input_from(s))) {
            ++publishes;
            always_pd = always_pd && positive_definite(learner.kf().p());
            if (s.t > 60.0) always_sensor_valid = always_sensor_valid && learner.params().sensor_valid;
        }
    });
    const VehicleParams &p = learner.params();
    // 20 Hz로 발행한다
    ASSERT_GT(publishes, 17900);
    ASSERT_LT(publishes, 18100);
    ASSERT_TRUE(always_pd) << "Joseph 갱신은 공분산을 양정치로 유지한다";
    ASSERT_NEAR(p.steer_ratio, truth.sr, 0.3) << "조향비가 참값으로 수렴한다";
    ASSERT_NEAR(p.stiffness_factor, truth.stiffness, 0.1) << "강성이 수렴한다";
    ASSERT_NEAR(p.angle_offset_average_deg, deg(truth.offset), 0.15) << "각도 오프셋이 수렴한다";
    ASSERT_NEAR(p.roll_rad, truth.roll, rad(0.3)) << "롤이 횡가속도계를 따른다";
    // 수렴한 추정은 유효하다
    ASSERT_TRUE(p.valid);
    ASSERT_TRUE(p.inputs_ok);
    ASSERT_TRUE(always_sensor_valid) << "모델 요레이트와 관측 요레이트가 맞는다";
}

// 상류 스케줄(20 Hz 관측)은 조향이 50 ms씩 멈춰 들어가 SR을 낮게 읽는다. 영점·롤은 그대로다.
// 이 성질이 바뀌면(스케줄이 달라지면) 여기서 걸린다.
TEST(LateralLearners, UpstreamScheduleReadsSteerRatioLow) {
    const CarKalman::Mat p0 = open_p0();
    VehicleParamsOptions options;
    options.p_initial = &p0;
    options.car_state_every_tick = false;
    VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0, options);
    const Truth truth;
    synthesize(truth, 900.0, [&](const SynthSample &s) { learner.update(input_from(s)); });
    const VehicleParams &p = learner.params();
    // 상류 20 Hz 스케줄은 조향비를 낮게 읽는다
    ASSERT_LT(p.steer_ratio, truth.sr - 0.3);
    ASSERT_GT(p.steer_ratio, truth.sr - 1.5);
    ASSERT_NEAR(p.angle_offset_average_deg, deg(truth.offset), 0.15) << "상류 스케줄로도 각도 오프셋은 찾는다";
    ASSERT_NEAR(p.roll_rad, truth.roll, rad(0.3)) << "상류 스케줄로도 롤은 찾는다";
}

TEST(LateralLearners, GatesHoldState) {
    VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0);
    const CarKalman::Vec before = learner.kf().x();
    auto feed = [&](double speed, double angle, int gear, bool fresh) {
        for (int n = 0; n < 500; ++n) {
            VehicleParamsInput in;
            in.t_s = n * 0.01;
            in.inputs_fresh = fresh;
            in.steering_angle_deg = angle;
            in.speed_mps = speed;
            in.gear = gear;
            in.yaw_rate_valid = true;
            in.yaw_rate_rad_s = 0.2;
            learner.update(in);
        }
    };
    feed(0.9, 5.0, 5, true);
    feed(10.0, 50.0, 5, true);
    feed(10.0, 5.0, 7, true);
    feed(10.0, 5.0, 5, false);
    ASSERT_EQ(learner.kf().x(), before) << "저속·큰 조향각·후진·낡은 입력은 상태를 바꾸지 않는다";
}

TEST(LateralLearners, OutputLimitsAndHysteresis) {
    VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0);
    double t = 0.0;
    auto publish = [&]() {
        for (;;) {
            VehicleParamsInput in;
            in.t_s = t;
            t += 0.01;
            in.inputs_fresh = true;
            in.speed_mps = 0.0;  // 비활성: 상태를 우리가 정한 값에 둔다
            if (learner.update(in)) return learner.params();
        }
    };
    publish();
    learner.kf().mutable_x()[CarKalman::kAngleOffset] = rad(12.0);
    double last = publish().angle_offset_average_deg;
    ASSERT_NEAR(static_cast<float>(last), 1.0f, 1e-9f) << "오프셋 출력은 발행마다 최대 1도 움직인다";
    VehicleParams p{};
    for (int i = 0; i < 20; ++i) p = publish();
    ASSERT_NEAR(static_cast<float>(p.angle_offset_average_deg), 12.0f, 1e-9f) << "오프셋 출력이 상태값에 도달한다";
    // 10도를 넘는 오프셋은 무효
    ASSERT_FALSE(p.angle_offset_average_valid);
    ASSERT_FALSE(p.valid);
    learner.kf().mutable_x()[CarKalman::kAngleOffset] = rad(9.0);
    for (int i = 0; i < 20; ++i) p = publish();
    ASSERT_FALSE(p.angle_offset_average_valid) << "다시 유효가 되려면 낮춘 기준 8도 아래로 와야 한다";
    learner.kf().mutable_x()[CarKalman::kAngleOffset] = rad(7.5);
    for (int i = 0; i < 20; ++i) p = publish();
    ASSERT_TRUE(p.angle_offset_average_valid) << "8도 아래에서 다시 유효";
    learner.kf().mutable_x()[CarKalman::kSteerRatio] = 34.0;
    p = publish();
    // 사전값의 두 배를 넘는 조향비는 무효
    ASSERT_FALSE(p.steer_ratio_valid);
    ASSERT_FALSE(p.valid);
}

TEST(LateralLearners, Persistence) {
    const VehicleModelConstants c = k7();
    VehicleParamsLearner learner(c, 16.8, 1.0, 0.0);
    VehicleParamsInput in;
    in.inputs_fresh = true;
    int due = 0, published = 0;
    for (int n = 0; n < 1201 * 5 + 2; ++n) {
        in.t_s = n * 0.01;
        if (learner.update(in)) {
            if (learner.persist_due()) {
                ASSERT_EQ(published % 1200, 0) << "발행 0, 1200, ...번째에 저장한다";
                ++due;
            }
            ++published;
        }
    }
    ASSERT_EQ(due, 2) << "첫 발행을 포함해 1분에 한 번 저장한다";

    VehicleParams p;
    p.steer_ratio = 15.1;
    p.stiffness_factor = 0.8;
    p.angle_offset_average_deg = -1.62;
    const std::string saved = vehicle_params_persist_data(p, c, -0.0017);
    const VehicleParamsInit ok = restore_vehicle_params(saved, c);
    // 조향비·평균 오프셋·요 바이어스를 복원한다
    ASSERT_TRUE(ok.restored);
    ASSERT_NEAR(static_cast<float>(ok.steer_ratio), 15.1f, 1e-4f);
    ASSERT_NEAR(static_cast<float>(ok.angle_offset_deg), -1.62f, 1e-4f);
    ASSERT_NEAR(static_cast<float>(ok.yaw_bias_rad_s), -0.0017f, 1e-7f);
    ASSERT_EQ(ok.stiffness_factor, 1.0) << "강성은 주행마다 초기화한다";

    VehicleModelConstants other = c;
    other.wheelbase_m += 0.05;
    ASSERT_FALSE(restore_vehicle_params(saved, other).restored) << "다른 차의 저장값은 복원하지 않는다";
    VehicleModelConstants tuned = c;
    tuned.tire_stiffness_factor = c.tire_stiffness_factor * 0.8;
    const VehicleParamsInit retuned = restore_vehicle_params(saved, tuned);
    ASSERT_TRUE(retuned.restored) << "타이어 강성 계수를 바꿔도 같은 차다(상류는 차종 지문만 본다)";
    ASSERT_NEAR(static_cast<float>(retuned.angle_offset_deg), -1.62f, 1e-4f);
    // 2026-10-22까지 쓴 파일에는 질량(mass_kg)이 있다. 질량은 차량 모델에서 약분돼 대조하지 않고 그대로 복원한다
    const std::string with_mass =
        "{\n  \"version\": 1,\n  \"mass_kg\": 1816,\n" + saved.substr(saved.find("  \"wheelbase_m\""));
    ASSERT_NE(with_mass.find("\"mass_kg\""), std::string::npos);
    const VehicleParamsInit old_file = restore_vehicle_params(with_mass, c);
    ASSERT_TRUE(old_file.restored);
    ASSERT_NEAR(static_cast<float>(old_file.steer_ratio), 15.1f, 1e-4f);
    VehicleModelConstants prior = c;
    prior.steer_ratio = 7.0;  // 15.1 > 2×7
    ASSERT_FALSE(restore_vehicle_params(saved, prior).restored) << "말이 안 되는 조향비는 복원하지 않는다";
    const VehicleParamsInit bad = restore_vehicle_params("{ nope", c);
    // 깨진 파일이면 사전값으로 돌아간다
    ASSERT_FALSE(bad.restored);
    ASSERT_EQ(bad.steer_ratio, c.steer_ratio);
    ASSERT_EQ(bad.angle_offset_deg, 0.0);
}

TEST(LateralLearners, YawBias) {
    YawBiasEstimator est;
    double t = 0.0;
    for (int i = 0; i < 150; ++i, t += 0.01) est.update(t, 0.0, true, -0.0019);
    ASSERT_EQ(est.bias(), 0.0) << "바이어스는 2초 정차를 기다린다";
    for (int i = 0; i < 2000; ++i, t += 0.01) est.update(t, 0.0, true, -0.0019);
    ASSERT_NEAR(static_cast<float>(est.bias()), -0.0019f, 1e-6f) << "바이어스는 정차 중 요레이트를 따른다";
    const double before = est.bias();
    est.update(t, 12.0, true, 0.3);
    ASSERT_EQ(est.bias(), before) << "주행 중에는 바이어스가 움직이지 않는다";
    ASSERT_NEAR(static_cast<float>(est.update(t, 12.0, true, 0.3)), static_cast<float>(0.3 + 0.0019), 1e-6f)
        << "보정한 요레이트는 바이어스를 뺀 값이다";
    // 휠속도 0으로 기어가며 도는 동안(>0.5°/s)은 흡수하지 않는다
    for (int i = 0; i < 1000; ++i, t += 0.01) est.update(t, 0.0, true, 0.05);
    ASSERT_EQ(est.bias(), before) << "휠 속도 0의 느린 회전은 바이어스가 아니다";
}

// ---------------------------------------------------------------- torqued

constexpr double kGravity = 9.81;
constexpr double kTorqueLag = 0.34;

// 조향 토크를 20초에 −0.5→0.5로 쓸어 되돌린다(꼭짓점은 20 Hz 틱 위). 지연 뒤 횡가속은 직선에서 법선 방향으로
// half_width만큼, 올릴 때 아래·내릴 때 위로 벗어난다(마찰 평행사변형). 요레이트는 v·r − g·sinθ가 그 횡가속이 되게
// 만든다(우측 양수). 폭이 넓을수록 적합 기울기가 커진다(상류 성질: 버킷 밀도가 토크 축 고정 경계에서 계단이라 두
// 가지가 다른 자리에서 잘린다. 폭 0.02 +0.4%, 0.05 +2.8%).
struct TorqueTruth {
    double factor = 1.6;
    double offset = 0.05;
    double half_width = 0.02;
    double roll_rad = 1.0 * kPi / 180.0;
    double speed = 20.0;
};

double sweep(double t) {
    const double phase = std::fmod(t + 400.0, 40.0);
    return phase < 20.0 ? -0.5 + phase / 20.0 : 0.5 - (phase - 20.0) / 20.0;
}
double rising(double t) { return std::fmod(t + 400.0, 40.0) < 20.0 ? 1.0 : -1.0; }

TorqueEstimatorInput torque_input(const TorqueTruth &tr, double t) {
    const double phi = std::atan(tr.factor);
    const double src = t - kTorqueLag;
    const double y = tr.factor * sweep(src) + tr.offset - rising(src) * tr.half_width * std::cos(phi);
    TorqueEstimatorInput in;
    in.t_s = t;
    in.inputs_fresh = true;
    in.lat_active = true;
    in.steer_torque = sweep(t) + rising(t) * tr.half_width * std::sin(phi);
    in.speed_mps = tr.speed;
    in.pose_valid = true;
    in.roll_rad = tr.roll_rad;
    in.yaw_rate_rad_s = (y + std::sin(tr.roll_rad) * kGravity) / tr.speed;
    return in;
}

// 100 Hz 틱 [tick0, tick0+ticks). edit이 입력을 바꿀 수 있다.
template <class F>
void drive_torque(TorqueEstimator *est, const TorqueTruth &tr, long tick0, long ticks, F &&edit) {
    for (long i = tick0; i < tick0 + ticks; ++i) {
        TorqueEstimatorInput in = torque_input(tr, i * 0.01);
        edit(i, &in);
        est->update(in);
    }
}
void drive_torque(TorqueEstimator *est, const TorqueTruth &tr, long tick0, long ticks) {
    drive_torque(est, tr, tick0, ticks, [](long, TorqueEstimatorInput *) {});
}

TorqueTuning prior(double factor = 1.4) {
    TorqueTuning t;
    t.lat_accel_factor = factor;
    t.friction = 0.1;
    return t;
}

// 독립 참조: [x,1,y]ᵀ[x,1,y]의 최소 고유값(3차 특성방정식 폐형해)과 그 고유벡터(두 행의 외적).
void reference_tls(const std::vector<std::array<double, 2>> &pts, double *slope, double *offset, double *friction) {
    double a[3][3] = {};
    for (const auto &p : pts) {
        const double r[3] = {p[0], 1.0, p[1]};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) a[i][j] += r[i] * r[j];
    }
    const double q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
    const double p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    const double p2 = std::pow(a[0][0] - q, 2) + std::pow(a[1][1] - q, 2) + std::pow(a[2][2] - q, 2) + 2 * p1;
    const double pp = std::sqrt(p2 / 6.0);
    double b[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) b[i][j] = (a[i][j] - (i == j ? q : 0.0)) / pp;
    const double det = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) -
                       b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) +
                       b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
    const double phi = std::acos(std::fmax(-1.0, std::fmin(1.0, det / 2.0))) / 3.0;
    const double lambda = q + 2 * pp * std::cos(phi + 2.0 * kPi / 3.0);
    double m[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m[i][j] = a[i][j] - (i == j ? lambda : 0.0);
    double best[3] = {}, best_norm = -1.0;
    for (int r0 = 0; r0 < 3; ++r0) {
        const int r1 = (r0 + 1) % 3;
        const double c[3] = {m[r0][1] * m[r1][2] - m[r0][2] * m[r1][1], m[r0][2] * m[r1][0] - m[r0][0] * m[r1][2],
                             m[r0][0] * m[r1][1] - m[r0][1] * m[r1][0]};
        const double n = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
        if (n > best_norm) {
            best_norm = n;
            for (int k = 0; k < 3; ++k) best[k] = c[k];
        }
    }
    *slope = -best[0] / best[2];
    *offset = -best[1] / best[2];
    const double sn = std::fabs(*slope) / std::sqrt(*slope * *slope + 1.0);
    const double cs = 1.0 / std::sqrt(*slope * *slope + 1.0);
    double mean = 0.0, var = 0.0;
    for (const auto &p : pts) mean += -p[0] * sn + p[1] * cs;
    mean /= static_cast<double>(pts.size());
    for (const auto &p : pts) var += std::pow(-p[0] * sn + p[1] * cs - mean, 2);
    *friction = std::sqrt(var / static_cast<double>(pts.size())) * 1.5;
}

double clipd(double v, double lo, double hi) { return std::fmin(std::fmax(v, lo), hi); }

// 한 번의 필터 갱신(상류 update_params).
void expect_filter_step(const TorqueParams &before_filters, double decay_before, const TorqueParams &raw,
                        const TorqueTuning &tuning, TorqueParams *out) {
    const double a = 0.05 / (decay_before + 0.05);
    out->lat_accel_factor =
        (1.0 - a) * before_filters.lat_accel_factor +
        a * clipd(raw.lat_accel_factor_raw, 0.7 * tuning.lat_accel_factor, 1.3 * tuning.lat_accel_factor);
    out->lat_accel_offset = (1.0 - a) * before_filters.lat_accel_offset + a * raw.lat_accel_offset_raw;
    out->friction =
        (1.0 - a) * before_filters.friction + a * clipd(raw.friction_raw, 0.5 * tuning.friction, 1.5 * tuning.friction);
    out->decay = std::fmin(decay_before + 0.05, 250.0);
}

bool same_filters(const TorqueParams &a, const TorqueParams &b, double tol) {
    return std::fabs(a.lat_accel_factor - b.lat_accel_factor) <= tol &&
           std::fabs(a.lat_accel_offset - b.lat_accel_offset) <= tol && std::fabs(a.friction - b.friction) <= tol &&
           std::fabs(a.decay - b.decay) <= tol;
}

TEST(LateralLearners, TorqueExactLine) {
    TorqueTruth tr;
    tr.half_width = 0.0;
    TorqueEstimator est(prior(), kTorqueLag);
    drive_torque(&est, tr, 0, 40000);
    const TorqueParams &p = est.params();
    // 한 직선 위의 점은 그 직선과 마찰 0을 준다
    ASSERT_NEAR(p.lat_accel_factor_raw, tr.factor, 1e-9);
    ASSERT_NEAR(p.lat_accel_offset_raw, tr.offset, 1e-9);
    ASSERT_LT(p.friction_raw, 1e-9);
}

// 긴 합성 주행: 유효 시점, 원시 추정, 필터 첫 걸음, 저장 틱의 이중 갱신, decay 상한.
TEST(LateralLearners, TorqueLearnsParallelogram) {
    TorqueTruth tr;
    const TorqueTuning tuning = prior();
    TorqueEstimator est(tuning, kTorqueLag);
    TorqueParams prev{};
    long frame = -1, first_valid_frame = -1;
    bool checked_first_step = false, checked_double = false;
    double prev_decay = 0.0;
    bool published = false;
    for (long i = 0; i < 150000 || !published; ++i) {  // 발행 틱에서 끝낸다
        published = est.update(torque_input(tr, i * 0.01));
        if (i % 5 == 0) ++frame;
        if (!published) continue;
        const TorqueParams &p = est.params();
        ASSERT_EQ((frame % 5), 0) << "토크 파라미터는 20 Hz 프레임 5개마다 발행한다";
        // 입력 범위·use_params·max_resets
        ASSERT_TRUE(p.inputs_ok);
        ASSERT_TRUE(p.use_params);
        ASSERT_EQ(p.max_resets, 1.0);
        if (p.valid && first_valid_frame < 0) {
            first_valid_frame = frame;
            // 유효는 모든 버킷이 찼다는 뜻이다
            ASSERT_EQ(p.cal_perc, 100);
            ASSERT_GE(p.total_bucket_points, 4000);
            // 직전 메시지는 아직 보정 중이었다
            ASSERT_LT(prev.cal_perc, 100);
            ASSERT_FALSE(prev.valid);
            // 버킷이 유효해질 때까지 필터는 사전값을 유지한다
            ASSERT_EQ(prev.lat_accel_factor, tuning.lat_accel_factor);
            ASSERT_EQ(prev.friction, tuning.friction);
            ASSERT_EQ(prev.lat_accel_offset, 0.0);
            ASSERT_EQ(prev.decay, 50.0);
            TorqueParams want;
            expect_filter_step(prev, prev.decay, p, tuning, &want);
            ASSERT_TRUE(same_filters(p, want, 1e-12)) << "첫 유효 메시지는 사전값에서 필터 한 스텝 움직인 값이다";
            checked_first_step = true;
        } else if (p.valid && first_valid_frame >= 0 && prev.valid && p.decay < 250.0) {
            // 직전 발행 틱이 저장 틱이면 그 사이 필터가 두 번 돌았다
            const double step = ((frame - 5) % 240 == 0) ? 0.10 : 0.05;
            ASSERT_LT(std::fabs(p.decay - prev_decay - step), 1e-9)
                << "decay는 갱신마다 0.05, 캐시 프레임에서는 두 번 늘어난다";
            if (step > 0.07) checked_double = true;
        }
        prev = p;
        prev_decay = p.decay;
    }
    // 긴 주행이 두 필터 검사에 모두 닿았다
    ASSERT_TRUE(checked_first_step);
    ASSERT_TRUE(checked_double);
    const TorqueParams &p = est.params();
    // decay는 250에서 포화한다
    ASSERT_TRUE(p.valid);
    ASSERT_EQ(p.decay, 250.0);
    // 보정 완료에는 안쪽 버킷의 500점이 걸린다
    ASSERT_GT(first_valid_frame * 0.05, 250.0);
    ASSERT_LT(first_valid_frame * 0.05, 400.0);
    // 마지막 발행이 쓴 점들로 독립 참조와 대조
    double slope, offset, friction;
    reference_tls(est.points(), &slope, &offset, &friction);
    // TLS 결과가 독립적인 닫힌 해와 같다
    ASSERT_NEAR(p.lat_accel_factor_raw, slope, 1e-9);
    ASSERT_NEAR(p.lat_accel_offset_raw, offset, 1e-9);
    ASSERT_NEAR(p.friction_raw, friction, 1e-9);
    // 평행사변형에서 그 기울기, 오프셋, 반폭의 1.5배가 나온다
    ASSERT_NEAR(p.lat_accel_factor_raw, tr.factor, 0.005 * tr.factor);
    ASSERT_NEAR(p.lat_accel_offset_raw, tr.offset, 0.002);
    ASSERT_NEAR(p.friction_raw, 1.5 * tr.half_width, 0.02 * 1.5 * tr.half_width);
    // 필터된 배율이 추정값 쪽으로 대부분 움직인다
    ASSERT_GT(p.lat_accel_factor, tuning.lat_accel_factor + 0.5 * (tr.factor - tuning.lat_accel_factor));
    ASSERT_LT(p.lat_accel_factor, tr.factor);
}

TEST(LateralLearners, TorqueClipsToPriorBand) {
    TorqueTruth tr;
    tr.factor = 1.9;       // 사전 1.4의 +30%(1.82) 밖
    tr.half_width = 0.12;  // 마찰 0.18, 사전 0.1의 +50%(0.15) 밖
    const TorqueTuning tuning = prior();
    TorqueEstimator est(tuning, kTorqueLag);
    TorqueParams prev{};
    for (long i = 0; i < 100000; ++i) {
        if (!est.update(torque_input(tr, i * 0.01))) continue;
        const TorqueParams &p = est.params();
        if (p.valid) {
            // raw 추정이 허용 대역 밖이다
            ASSERT_GT(p.lat_accel_factor_raw, 1.82);
            ASSERT_GT(p.friction_raw, 0.15);
            TorqueParams want;
            expect_filter_step(prev, prev.decay, p, tuning, &want);
            ASSERT_TRUE(same_filters(p, want, 1e-12)) << "필터는 클립한 값 쪽으로 움직인다";
            return;
        }
        prev = p;
    }
    FAIL() << "클립 주행이 끝내 유효해지지 않았다";
}

long count_points_added(TorqueEstimator *est, const TorqueTruth &tr, long tick0, long ticks,
                        const std::function<void(long, TorqueEstimatorInput *)> &edit) {
    const int before = est->total_points();
    drive_torque(est, tr, tick0, ticks, edit);
    return est->total_points() - before;
}

TEST(LateralLearners, TorqueGates) {
    TorqueTruth tr;
    const auto none = [](long, TorqueEstimatorInput *) {};
    {
        TorqueEstimator est(prior(), kTorqueLag);
        drive_torque(&est, tr, 0, 495);
        ASSERT_EQ(est.total_points(), 0) << "이력 100개 전에는 점을 넣지 않는다";
        drive_torque(&est, tr, 495, 1);
        ASSERT_EQ(est.total_points(), 1) << "100번째 표본에서 첫 점이 들어간다";
    }
    struct Gate {
        const char *what;
        std::function<void(long, TorqueEstimatorInput *)> edit;
    };
    const Gate gates[] = {
        {"speed at 15 m/s is not above MIN_VEL", [](long, TorqueEstimatorInput *in) { in->speed_mps = 15.0; }},
        {"invalid pose", [](long, TorqueEstimatorInput *in) { in->pose_valid = false; }},
        {"torque at the 0.02 threshold", [](long, TorqueEstimatorInput *in) { in->steer_torque = 0.02; }},
        {"lateral accel above 1",
         [](long, TorqueEstimatorInput *in) {
             in->yaw_rate_rad_s = (1.001 + std::sin(in->roll_rad) * kGravity) / in->speed_mps;
         }},
        {"lateral control inactive", [](long, TorqueEstimatorInput *in) { in->lat_active = false; }},
        {"driver override", [](long, TorqueEstimatorInput *in) { in->steer_override = true; }},
        {"stale inputs", [](long, TorqueEstimatorInput *in) { in->inputs_fresh = false; }},
    };
    for (const Gate &g : gates) {
        TorqueEstimator est(prior(), kTorqueLag);
        ASSERT_GT(count_points_added(&est, tr, 0, 2000, none), 200) << "기준 주행은 점을 넣는다";
        drive_torque(&est, tr, 2000, 40, g.edit);  // 속도·토크는 지연만큼 지난 이력에서 읽는다
        EXPECT_EQ(count_points_added(&est, tr, 2040, 1000, g.edit), 0) << g.what;
    }
    // 0.5초 개입·해제는 2초 창 + 지연이 지날 때까지 막는다. 낡은 입력은 이력을 지우지 않는다.
    for (int which = 0; which < 3; ++which) {
        SCOPED_TRACE(which);
        TorqueEstimator est(prior(), kTorqueLag);
        drive_torque(&est, tr, 0, 2000);
        const auto edit = [which](long, TorqueEstimatorInput *in) {
            if (which == 0)
                in->steer_override = true;
            else if (which == 1)
                in->lat_active = false;
            else
                in->inputs_fresh = false;
        };
        // 마지막 해제 표본 20.45초 → 창 [t−2−lag, t)가 그 뒤 20.5초를 벗어나는 22.84초까지 막힘
        count_points_added(&est, tr, 2000, 50, edit);
        const long soon = count_points_added(&est, tr, 2050, which == 2 ? 5 : 230, none);
        const long later = count_points_added(&est, tr, which == 2 ? 2055 : 2290, 100, none);
        ASSERT_EQ(soon, which < 2 ? 0 : 1);
        ASSERT_GT(later, 10);
    }
    // 상류 성질: 활성은 보간 후 0이 아니면 참이라 한 표본짜리 해제는 걸러지지 않고, 두 표본이면 걸린다
    for (int samples = 1; samples <= 2; ++samples) {
        TorqueEstimator est(prior(), kTorqueLag);
        drive_torque(&est, tr, 0, 2500);
        drive_torque(&est, tr, 2500, 10, [samples](long i, TorqueEstimatorInput *in) {
            if (i < 2500 + 5 * samples) in->lat_active = false;
        });
        const long added = count_points_added(&est, tr, 2510, 220, none);  // 두 표본이면 막히는 구간
        if (samples == 1)
            ASSERT_GT(added, 40) << "비활성 표본 하나는 보간하면 0이 아니라 통과한다";
        else
            ASSERT_EQ(added, 0) << "비활성 표본 둘은 그 사이가 0으로 보간돼 막힌다";
    }
}

// 새 점이 없으면(시동을 켜고 서 있으면) 처음 한 번 뒤로는 저장하지 않는다. 점이 그대로인 동안 필터값만
// 수렴해 바뀌는 캐시를 12초마다 다시 쓰지 않게 한다. 입력이 낡으면(차가 꺼짐) 처음 한 번도 쉰다.
TEST(LateralLearners, TorqueCacheSavedOnlyAfterNewPoints) {
    for (const bool fresh : {true, false}) {
        SCOPED_TRACE(fresh);
        TorqueEstimator est(prior(), kTorqueLag);
        int saves = 0;
        for (long i = 0; i < 60000; ++i) {  // 100 Hz로 600 s, 속도 0이라 점이 없다
            TorqueEstimatorInput in;
            in.t_s = i * 0.01;
            in.inputs_fresh = fresh;
            est.update(in);
            saves += est.persist_due() ? 1 : 0;
        }
        EXPECT_EQ(saves, fresh ? 1 : 0);
    }
}

// 상류 torqued는 시동이 켜졌을 때만 돈다. 입력이 낡은 동안은 발행·적합·필터·저장을 쉬고 지난 값을 둔다(점이
// 그대로인데 필터가 다시 적합한 값으로 움직이지 않는다). 돌아오면 그 자리에서 이어진다.
TEST(LateralLearners, TorquePausesWhileInputsAreStale) {
    TorqueTruth tr;
    TorqueEstimator est(prior(), kTorqueLag);
    long i = 0;
    for (; i < 100000 || !est.params().valid; ++i) est.update(torque_input(tr, i * 0.01));
    while (!est.update(torque_input(tr, i * 0.01))) ++i;  // 발행 틱에서 멈춘다
    const TorqueParams before = est.params();
    const int points = est.total_points();
    for (long k = 1; k <= 60000; ++k) {  // 600 s, 차가 꺼짐
        TorqueEstimatorInput in = torque_input(tr, (i + k) * 0.01);
        in.inputs_fresh = false;
        ASSERT_FALSE(est.update(in)) << "쉬는 동안은 발행하지 않는다";
        ASSERT_FALSE(est.persist_due()) << "쉬는 동안은 저장하지 않는다";
    }
    i += 60001;
    const TorqueParams &paused = est.params();
    ASSERT_FALSE(paused.inputs_ok);
    ASSERT_TRUE(paused.valid);
    ASSERT_EQ(paused.lat_accel_factor_raw, before.lat_accel_factor_raw);
    ASSERT_TRUE(same_filters(paused, before, 0.0)) << "필터와 decay가 그대로다";
    ASSERT_EQ(est.total_points(), points);
    // 다시 켜면 지난 값에서 이어 발행하고 필터가 다시 움직인다
    while (!est.update(torque_input(tr, i * 0.01))) ++i;
    ASSERT_TRUE(est.params().inputs_ok);
    ASSERT_GT(est.params().decay, before.decay);
}

TEST(LateralLearners, TorqueScheduleAndCache) {
    TorqueTruth tr;
    const TorqueTuning tuning = prior();
    TorqueEstimator est(tuning, kTorqueLag);
    long frame = -1;
    std::string calibrating_cache, valid_cache;
    std::vector<std::array<double, 2>> valid_points;
    TorqueParams at_valid_persist{};
    for (long i = 0; i < 90000; ++i) {
        const bool published = est.update(torque_input(tr, i * 0.01));
        if (i % 5 != 0) continue;
        ++frame;
        ASSERT_EQ(est.persist_due(), (frame % 240 == 0))
            << "첫 프레임부터 240프레임마다 캐시한다(주행 중에는 늘 새 점이 있다)";
        ASSERT_EQ(published, (frame % 5 == 0)) << "5프레임마다 발행한다";
        if (!est.persist_due()) continue;
        if (frame == 2400) calibrating_cache = est.persist_data();  // 120초: 계산 가능, 아직 무효
        if (est.params().valid && valid_cache.empty()) {
            valid_cache = est.persist_data();
            valid_points = est.points();
            at_valid_persist = est.params();
        }
    }
    // 캐시가 두 번 다 만들어졌다
    ASSERT_FALSE(calibrating_cache.empty());
    ASSERT_FALSE(valid_cache.empty());

    // 유효 캐시: 저장 get_msg는 발행값에서 필터를 한 번 더 돌린 값이다
    TorqueParams cached;
    expect_filter_step(at_valid_persist, at_valid_persist.decay, at_valid_persist, tuning, &cached);
    TorqueEstimator restored(tuning, kTorqueLag, valid_cache);
    ASSERT_EQ(restored.restore_status(), TorqueRestore::Restored) << "유효한 캐시를 복원한다";
    const std::vector<std::array<double, 2>> pts = restored.points();
    ASSERT_EQ(pts.size(), valid_points.size()) << "캐시한 점이 모두 돌아온다";
    for (size_t k = 0; k < pts.size(); ++k) {
        // 점이 float 정밀도로 순서대로 왕복한다
        ASSERT_EQ(pts[k][0], static_cast<float>(valid_points[k][0]));
        ASSERT_EQ(pts[k][1], static_cast<float>(valid_points[k][1]));
    }
    TorqueParams start;
    start.lat_accel_factor = static_cast<float>(cached.lat_accel_factor);
    start.lat_accel_offset = static_cast<float>(cached.lat_accel_offset);
    start.friction = static_cast<float>(cached.friction);
    start.decay = static_cast<float>(cached.decay);
    // 첫 신선한 틱 전에도 복원한 점의 추정을 보이고 필터는 캐시값 그대로다
    const TorqueParams &shown = restored.params();
    ASSERT_FALSE(shown.inputs_ok);
    ASSERT_TRUE(shown.valid);
    ASSERT_EQ(shown.total_bucket_points, static_cast<int>(valid_points.size()));
    ASSERT_TRUE(same_filters(shown, start, 0.0));
    TorqueEstimatorInput stale = torque_input(tr, 0.0);
    stale.inputs_fresh = false;
    ASSERT_FALSE(restored.update(stale)) << "입력이 낡으면 쉰다";
    ASSERT_TRUE(restored.update(torque_input(tr, 0.05))) << "복원한 추정기는 첫 신선한 프레임에 발행한다";
    const TorqueParams &r = restored.params();
    // 복원한 버킷은 유효하다
    ASSERT_TRUE(r.inputs_ok);
    ASSERT_TRUE(r.valid);
    ASSERT_EQ(r.max_resets, 1.0);
    TorqueParams want;
    expect_filter_step(start, start.decay, r, tuning, &want);
    ASSERT_TRUE(same_filters(r, want, 1e-12)) << "복원한 필터와 decay는 캐시된 메시지에서 이어진다";

    // 무효 캐시: 점과 decay만, 필터는 사전값
    TorqueEstimator partial(tuning, kTorqueLag, calibrating_cache);
    // 보정 중인 캐시도 점은 복원한다
    ASSERT_EQ(partial.restore_status(), TorqueRestore::Restored);
    ASSERT_GT(partial.total_points(), 0);
    partial.update(torque_input(tr, 0.0));
    // 보정 중인 캐시는 필터를 사전값으로 둔다
    ASSERT_FALSE(partial.params().valid);
    ASSERT_EQ(partial.params().lat_accel_factor, tuning.lat_accel_factor);
    ASSERT_EQ(partial.params().friction, tuning.friction);
    ASSERT_EQ(partial.params().decay, 50.0);

    // 튜닝이 바뀌면 무시(지우지 않음), 깨졌으면 지움
    TorqueEstimator other(prior(1.5), kTorqueLag, valid_cache);
    // 다른 튜닝의 캐시는 무시한다
    ASSERT_EQ(other.restore_status(), TorqueRestore::KeyMismatch);
    ASSERT_EQ(other.total_points(), 0);
    other.update(torque_input(tr, 0.0));
    // 무시한 캐시는 사전값을 남긴다
    ASSERT_EQ(other.params().lat_accel_factor, 1.5);
    ASSERT_EQ(other.params().decay, 50.0);
    const std::string truncated = valid_cache.substr(0, valid_cache.size() - 1);
    std::string bad_magic = valid_cache;
    bad_magic[0] = 'X';
    // 깨진 캐시는 지우도록 알린다
    ASSERT_EQ(TorqueEstimator(tuning, kTorqueLag, truncated).restore_status(), TorqueRestore::Corrupt);
    ASSERT_EQ(TorqueEstimator(tuning, kTorqueLag, bad_magic).restore_status(), TorqueRestore::Corrupt);
    ASSERT_EQ(TorqueEstimator(tuning, kTorqueLag, "").restore_status(), TorqueRestore::None);
    ASSERT_EQ(valid_cache.compare(0, 8, "EDGETQC1"), 0);

    // 다른 요레이트·롤 출처의 캐시: 점과 절편은 버리고(롤 출처 차이만큼 어긋난다) 배율·마찰만 이어 쓴다.
    TorqueEstimator switched(tuning, kTorqueLag, valid_cache, true);
    ASSERT_EQ(switched.restore_status(), TorqueRestore::SourceChanged);
    ASSERT_EQ(switched.total_points(), 0);
    switched.update(torque_input(tr, 0.0));
    ASSERT_FALSE(switched.params().valid);
    ASSERT_NEAR(switched.params().lat_accel_factor, cached.lat_accel_factor, 1e-6);
    ASSERT_NEAR(switched.params().friction, cached.friction, 1e-6);
    ASSERT_EQ(switched.params().lat_accel_offset, 0.0);
    ASSERT_EQ(switched.params().decay, 50.0);
    ASSERT_EQ(TorqueEstimator(tuning, kTorqueLag, switched.persist_data(), true).restore_status(),
              TorqueRestore::Restored)
        << "새 출처로 저장한 캐시는 그 출처로 복원한다";
}

// ---------------------------------------------------------------- controlsd 연결

// 모든 수신 메시지가 방금 들어온 60 km/h 주행 상태.
VehicleCanState driving_vehicle(double t) {
    VehicleCanState v;
    stamp_received_messages(&v, t);
    v.wheel_speed_fl_kph = v.wheel_speed_fr_kph = v.wheel_speed_rl_kph = v.wheel_speed_rr_kph = 60.0f;
    v.gear = 5;
    return v;
}

// controlsd처럼 컨트롤러가 같은 틱에 낸 신선도(kVehicleStateTimeoutS)와 휠 속도로 학습기를 돌린다(조향 중, 핸들 조작
// 없음).
void update_learners(LateralLearners *l, const VehicleCanState &vehicle, double now, int apply_torque) {
    l->update(vehicle, now, vehicle_state_fresh(vehicle, now, kVehicleStateTimeoutS),
              signal_time_fresh(vehicle.esp12_time_s, now, kVehicleStateTimeoutS),
              vehicle_speed_kph(vehicle, now, kVehicleStateTimeoutS) / 3.6, true, apply_torque, false);
}

// controlsd·재생 도구 공용 판단: 스위치·0.5초·필터 유효면 쓴다. inputsOK는 보지 않는다(상류 paramsd).
TEST(LateralLearners, LocalizerSampleFrom) {
    LocalizationState loc;
    loc.flags = kLocalizationFilterValid | kLocalizationSensorsOk | kLocalizationPosenetOk;  // inputs_ok 없음
    loc.angular_velocity_calib[2] = 0.1f;
    loc.angular_velocity_calib_std[2] = 0.013f;
    loc.orientation_calib[0] = 0.02f;
    loc.orientation_std[0] = 0.004f;
    LocalizerSample s;
    ASSERT_TRUE(localizer_sample_from(loc, true, 0.07, 12.0, &s));
    // 자이로 교차검증으로 inputs_ok가 내려가도 학습 입력은 그대로다
    EXPECT_TRUE(s.pose_ok);
    EXPECT_TRUE(s.roll_ok);
    EXPECT_DOUBLE_EQ(s.t_s, 12.0);
    EXPECT_FLOAT_EQ(static_cast<float>(s.yaw_rate_std_rad_s), 0.013f);
    // 스위치가 꺼졌거나 0.5초 넘게 낡았거나 필터가 무효면 ESP12
    EXPECT_FALSE(localizer_sample_from(loc, false, 0.07, 12.0, &s));
    EXPECT_FALSE(localizer_sample_from(loc, true, 0.6, 12.0, &s));
    loc.flags &= ~kLocalizationFilterValid;
    EXPECT_FALSE(localizer_sample_from(loc, true, 0.07, 12.0, &s));
    // 자세(posenet)가 무효면 요레이트·롤 관측 조건이, 센서가 무효면 롤 관측 조건이 빠진다
    loc.flags = kLocalizationFilterValid | kLocalizationSensorsOk;
    ASSERT_TRUE(localizer_sample_from(loc, true, 0.07, 12.0, &s));
    EXPECT_FALSE(s.pose_ok);
    loc.flags = kLocalizationFilterValid | kLocalizationPosenetOk;
    ASSERT_TRUE(localizer_sample_from(loc, true, 0.07, 12.0, &s));
    EXPECT_TRUE(s.pose_ok);
    EXPECT_FALSE(s.roll_ok);
}

// 100 Hz 제어 틱과 10 Hz locationd 표본(추정 시각 + 70 ms에 도착). 요레이트는 시각에 비례하는
// 램프라 보간이 정확하다.
struct LocalizerDrive {
    LateralLearners l;
    VehicleCanState vehicle;
    double esp_yaw_left = 0.02;
    double delay_s = 0.07;
    double roll = 0.02, roll_std = 0.003;
    bool pose_ok = true, roll_ok = true;
    explicit LocalizerDrive(const SteeringParams &sp) : l(sp, "", ""), vehicle(driving_vehicle(0.0)) {}
    double yaw_slope = 0.1;
    double yaw_right(double t) const { return yaw_slope * t; }
    void tick(double now, bool use = true, bool send = true, const std::function<void(VehicleCanState *)> &edit = {}) {
        vehicle = driving_vehicle(now);
        vehicle.yaw_rate_valid = true;
        vehicle.yaw_rate_rad_s = static_cast<float>(esp_yaw_left);
        vehicle.lat_accel_valid = true;
        if (edit) edit(&vehicle);
        if (send) {
            // 지금까지 도착한 마지막 표본
            const double ts = std::floor((now - delay_s) * 10.0 + 1e-9) / 10.0;
            LocalizerSample s;
            s.t_s = ts;
            s.yaw_rate_rad_s = yaw_right(ts);
            s.yaw_rate_std_rad_s = 0.013;
            s.roll_rad = roll;
            s.roll_std_rad = roll_std;
            s.pose_ok = pose_ok;
            s.roll_ok = roll_ok;
            l.set_localizer(use, s);
        } else {
            l.set_localizer(use, LocalizerSample{});
        }
        update_learners(&l, vehicle, now, 0);
    }
};

// 표본은 그 추정 시각에 관측한다: 학습기는 kLocalizerDelayS 늦게 돌고 그 시각 값을 보간한다
TEST(LateralLearners, LocalizerSamplesObservedAtTheirTime) {
    const SteeringParams sp;
    LocalizerDrive d(sp);
    d.tick(1.0, false);
    EXPECT_FALSE(d.l.localizer_inputs());
    EXPECT_DOUBLE_EQ(d.l.last_vehicle_input().t_s, 1.0) << "ESP12면 바로 넣는다";
    int checked = 0;
    double last_t = 1.0;
    for (int i = 1; i <= 200; ++i) {
        const double now = 1.0 + 0.01 * i;
        d.tick(now);
        ASSERT_TRUE(d.l.localizer_inputs());
        const VehicleParamsInput &in = d.l.last_vehicle_input();
        if (now < 1.0 + LateralLearners::kLocalizerDelayS + 0.02) continue;
        ASSERT_LE(in.t_s, now - LateralLearners::kLocalizerDelayS + 1e-9);
        ASSERT_GT(in.t_s, last_t) << "틱 순서대로 넣는다";
        last_t = in.t_s;
        ASSERT_TRUE(in.localizer_roll_given);
        ASSERT_TRUE(in.yaw_rate_valid);
        // 받은 즉시 지금 시각으로 넣으면 0.1·(지연)만큼 늦은 값이 된다
        ASSERT_NEAR(in.yaw_rate_rad_s, -d.yaw_right(in.t_s), 1e-9) << "좌측 양수, 그 시각 값";
        ASSERT_NEAR(in.yaw_rate_std_rad_s, 0.013, 1e-12) << "locationd std(상류)";
        const TorqueEstimatorInput &tin = d.l.last_torque_input();
        ASSERT_DOUBLE_EQ(tin.t_s, in.t_s);
        ASSERT_NEAR(tin.yaw_rate_rad_s, d.yaw_right(in.t_s), 1e-9);
        ASSERT_NEAR(tin.roll_rad, 0.02, 1e-12) << "torqued는 자세 롤을 그대로";
        ASSERT_TRUE(tin.pose_valid);
        ++checked;
    }
    EXPECT_GT(checked, 150);

    // ESP12로 돌아가면 쌓인 틱을 순서대로 다 넣고 이번 틱부터 바로 넣는다
    d.tick(3.01, false);
    EXPECT_FALSE(d.l.localizer_inputs());
    EXPECT_DOUBLE_EQ(d.l.last_vehicle_input().t_s, 3.01);
    EXPECT_NEAR(d.l.last_vehicle_input().yaw_rate_rad_s, d.esp_yaw_left, 1e-6);
    EXPECT_FALSE(d.l.last_torque_input().pose_valid) << "torqued는 시작할 때의 출처(locationd) 점만 쓴다";
}

// 같은 locationd 표본을 여러 틱 읽어도 한 번만 쌓는다. 학습기 시계로 옮긴 시각(t_s)은 읽을 때마다 조금씩 달라서
// (controlsd의 두 시계를 읽는 시각 차이) locationd의 원래 시각(stamp_ns)으로 가린다.
TEST(LateralLearners, SameLocalizerSampleIsKeptOnce) {
    const SteeringParams sp;
    LateralLearners l(sp, "", "");
    LocalizerSample s;
    s.stamp_ns = 1'000'000'000ULL;
    s.t_s = 1.0;
    s.pose_ok = s.roll_ok = true;
    l.set_localizer(true, s);
    for (int tick = 1; tick < 10; ++tick) {
        s.t_s = 1.0 + 1e-7 * tick;  // 같은 표본, 다시 계산한 시각만 조금 다르다
        l.set_localizer(true, s);
    }
    EXPECT_EQ(l.localizer_sample_count(), 1u);
    s.stamp_ns += 100'000'000ULL;
    s.t_s = 1.1;
    l.set_localizer(true, s);
    EXPECT_EQ(l.localizer_sample_count(), 2u) << "새 표본";
}

// 그 시각의 표본이 없거나 자세가 무효인 틱은 ESP12 요레이트이고 torqued 점은 만들지 않는다
TEST(LateralLearners, LocalizerFallsBackToEspPerTick) {
    const SteeringParams sp;
    LocalizerDrive d(sp);
    d.pose_ok = false;
    for (int i = 0; i <= 100; ++i) d.tick(1.0 + 0.01 * i);
    EXPECT_TRUE(d.l.localizer_inputs());
    EXPECT_FALSE(d.l.last_vehicle_input().localizer_roll_given);
    EXPECT_NEAR(d.l.last_vehicle_input().yaw_rate_rad_s, d.esp_yaw_left, 1e-6);
    EXPECT_FALSE(d.l.last_torque_input().pose_valid) << "출처가 섞인 점은 쓰지 않는다";

    // 표본이 끊기면(locationd가 멈춤) 0.1초까지만 마지막 값을 잇는다
    d.pose_ok = true;
    for (int i = 101; i <= 200; ++i) d.tick(1.0 + 0.01 * i);
    EXPECT_TRUE(d.l.last_vehicle_input().localizer_roll_given);
    const double last_sample = std::floor((3.0 - d.delay_s) * 10.0 + 1e-9) / 10.0;
    for (int i = 201; i <= 260; ++i) d.tick(1.0 + 0.01 * i, true, false);
    const VehicleParamsInput &in = d.l.last_vehicle_input();
    ASSERT_GT(in.t_s, last_sample + 0.1);
    EXPECT_FALSE(in.localizer_roll_given);
    EXPECT_NEAR(in.yaw_rate_rad_s, d.esp_yaw_left, 1e-6);
}

// paramsd까지 돈다: locationd 롤이 유효하면 그 값으로, 센서가 무효면(상류는 0±10°) ESP 횡가속
// 롤로 수렴하고, 그 동안 VehicleParams가 계속 유효하다(롤 std가 1.5°에 붙지 않는다).
TEST(LateralLearners, LocalizerRollThroughParamsd) {
    const SteeringParams sp;
    LocalizerDrive d(sp);
    d.esp_yaw_left = 0.0;
    d.yaw_slope = 0.0;  // 직진
    d.roll = rad(2.0);
    d.roll_std = rad(0.2);
    for (double now = 1.0; now < 21.0 - 1e-9; now += 0.01) d.tick(now);
    EXPECT_NEAR(deg(d.l.vehicle_params().roll_rad), 2.0, 0.3) << "locationd 롤";

    // 센서 무효 60초. ESP 횡가속이 말하는 롤은 0.5°(좌측 비력 g·sin 0.5°, 반전 저장)
    const double esp_roll = rad(0.5);
    d.roll_ok = false;
    bool always_valid = true;
    for (double now = 21.0; now < 81.0 - 1e-9; now += 0.01) {
        d.tick(now, true, true,
               [&](VehicleCanState *v) { v->lat_accel_mps2 = static_cast<float>(-9.81 * std::sin(esp_roll)); });
        if (d.l.vehicle_published()) always_valid = always_valid && d.l.vehicle_params().valid;
    }
    EXPECT_TRUE(always_valid);
    EXPECT_NEAR(d.l.vehicle_params().roll_rad, esp_roll, rad(0.2)) << "ESP12 횡가속 롤";
}

// ESP12 요레이트가 무효면 구심 가속을 뺄 수 없다: 코너링 횡가속을 롤로 읽지 않는다
TEST(LateralLearners, EspRollNeedsAValidYawRate) {
    const SteeringParams sp;
    LocalizerDrive d(sp);
    d.esp_yaw_left = 0.0;
    for (double now = 1.0; now < 21.0 - 1e-9; now += 0.01) d.tick(now, false);
    ASSERT_NEAR(deg(d.l.vehicle_params().roll_rad), 0.0, 0.1);

    // 요레이트 무효 10초 동안 좌회전(좌측 비력 1 m/s², 반전 저장). 그대로 쓰면 롤 5.9°를 관측한다.
    for (double now = 21.0; now < 31.0 - 1e-9; now += 0.01) {
        d.tick(now, false, true, [](VehicleCanState *v) {
            v->yaw_rate_valid = false;
            v->lat_accel_mps2 = -1.0f;
        });
    }
    EXPECT_NEAR(deg(d.l.vehicle_params().roll_rad), 0.0, 0.3);
}

// torqued lag는 조향 지연을 따른다(상류 lateralDelay). 줄어도 이력 시각이 거꾸로 가지 않는다.
TEST(LateralLearners, TorqueLagFollowsLateralDelay) {
    const SteeringParams sp;
    LateralLearners l(sp, "", "");
    EXPECT_DOUBLE_EQ(l.torque_estimator().lag(), sp.steer_actuator_delay);
    l.set_lateral_delay(0.5);
    EXPECT_DOUBLE_EQ(l.torque_estimator().lag(), 0.5);
    VehicleCanState vehicle = driving_vehicle(1.0);
    vehicle.yaw_rate_valid = true;
    vehicle.lat_accel_valid = true;
    for (int i = 0; i < 300; ++i) {
        if (i == 150) l.set_lateral_delay(0.2);  // 0.3초 줄인다
        vehicle = driving_vehicle(1.0 + 0.01 * i);
        vehicle.yaw_rate_valid = true;
        vehicle.lat_accel_valid = true;
        update_learners(&l, vehicle, 1.0 + 0.01 * i, 100);
        ASSERT_TRUE(std::isfinite(l.torque_params().lat_accel_factor));
    }
}

TEST(LateralLearners, LearnersGlue) {
    const SteeringParams sp;
    VehicleCanState vehicle = driving_vehicle(1.0);
    vehicle.steering_angle_deg = 3.0f;
    vehicle.yaw_rate_valid = true;
    vehicle.yaw_rate_rad_s = 0.02f;
    vehicle.lat_accel_valid = true;
    vehicle.lat_accel_mps2 = -0.4f;  // 반전 저장: 좌측 비력 +0.4
    {
        LateralLearners l(sp, "", "");
        // 새 학습기는 아직 내줄 값이 없다
        ASSERT_FALSE(l.vehicle_restored());
        ASSERT_FALSE(l.vehicle_restore_rejected());
        ASSERT_EQ(l.torque_restore_status(), TorqueRestore::None);
        ASSERT_FALSE(l.live().use_vehicle);
        ASSERT_FALSE(l.live().use_torque);
        update_learners(&l, vehicle, 1.0, 100);
        // paramsd 입력의 부호·배율은 재생 도구와 같다
        ASSERT_TRUE(l.last_vehicle_input().inputs_fresh);
        ASSERT_NEAR(l.last_vehicle_input().lat_accel_mps2, 0.4, 1e-6);
        ASSERT_TRUE(l.last_torque_input().lat_active);
        // 좌회전(보낸 토크 +, ESP12 요레이트 +)은 torqued에서 둘 다 음수다(우측 양수 관례)
        {
            // torqued는 컨트롤러 좌표계라 오프셋이 torque_lat_accel_offset과 같은 방향이다
            ASSERT_LT(std::fabs(l.last_torque_input().steer_torque + 100.0 / kHyundaiSteerMax), 1e-12);
            ASSERT_EQ(l.last_torque_input().yaw_rate_rad_s, -l.last_vehicle_input().yaw_rate_rad_s);
            ASSERT_GT(l.last_vehicle_input().yaw_rate_rad_s, 0.0);
        }
        // 첫 틱에 둘 다 발행·저장한다(상류 frame 0)
        ASSERT_TRUE(l.vehicle_published());
        ASSERT_TRUE(l.torque_published());
        ASSERT_TRUE(l.vehicle_persist_due());
        ASSERT_TRUE(l.torque_persist_due());
        const LiveLateralParams live = l.live();
        // 첫 메시지는 사전값을 싣는다
        ASSERT_TRUE(live.use_vehicle);
        ASSERT_EQ(live.steer_ratio, sp.steer_ratio);
        ASSERT_TRUE(live.use_torque);
        ASSERT_EQ(live.lat_accel_factor, sp.torque_lat_accel_factor);
        ASSERT_EQ(live.friction, sp.torque_friction);
        ASSERT_EQ(live.lat_accel_offset, 0.0f);
        // 봉투가 무효인 torqued 메시지는 토크 값을 바꾸지 않는다
        VehicleCanState stale = vehicle;
        for (long i = 1; i <= 40; ++i) {
            const double now = 1.0 + 0.01 * i + 5.0;
            update_learners(&l, stale, now, 100);
        }
        // 낡은 torqued 메시지는 적용 값을 바꾸지 않는다
        ASSERT_FALSE(l.torque_params().inputs_ok);
        ASSERT_EQ(l.live().lat_accel_factor, live.lat_accel_factor);

        // 저장 → 복원
        const std::string json = l.vehicle_persist_data();
        LateralLearners restored(sp, json, l.torque_persist_data());
        // 두 학습기가 각자 저장값을 복원한다
        ASSERT_TRUE(restored.vehicle_restored());
        ASSERT_EQ(restored.torque_restore_status(), TorqueRestore::Restored);
    }
    LateralLearners bad(sp, "{ not json", std::string("junk"));
    // 거부된 저장값은 controlsd가 지우도록 알린다
    ASSERT_TRUE(bad.vehicle_restore_rejected());
    ASSERT_FALSE(bad.vehicle_restored());
    ASSERT_EQ(bad.torque_restore_status(), TorqueRestore::Corrupt);
}

}  // namespace
