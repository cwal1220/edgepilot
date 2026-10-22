#include "localization/location_estimator.h"

#include <algorithm>
#include <numeric>

#include "common/utils_math.h"

namespace {

constexpr double kEarthG = 9.81;

// locationd.py
constexpr double kAccelSanityCheck = 100.0;    // m/s²
constexpr double kRotationSanityCheck = 10.0;  // rad/s
constexpr double kTransSanityCheck = 200.0;    // m/s
constexpr double kCalibRpySanityCheck = 0.5;   // rad
constexpr double kMinStdSanityCheck = 1e-5;
constexpr double kMaxFilterRewindTime = 0.8;  // s
constexpr double kYawrateCrossErrCheckFactor = 30.0;
constexpr double kInputInvalidLimit = 2.0;
constexpr double kInputInvalidRecovery = 10.0;  // s
constexpr double kPosenetStdInitial = 10.0;
constexpr int kPosenetStdHistHalf = 20;
constexpr double kCamOdoRotStdMult = 10.0;
constexpr double kCamOdoTransStdMult = 4.0;
constexpr double kImuFrequency = 104.0;  // imud ODR
constexpr double kCameraFrequency = 20.0;
// 상류는 메시지를 도착 순서대로 넣어 카메라 한 프레임이 자이로 약 5표본만 가린다. 여기서는
// 카메라가 IMU 묶음(~100 ms) 앞에 몰려 들어가 마지막 프레임 하나가 묶음 전체(~10표본)를
// 가리므로, 교차검사 실패는 프레임당 이만큼만 센다. 그래야 상류처럼 나쁜 프레임 하나(한계
// 9.5)는 넘어간다(2026-10-02 급회전에서 프레임 하나로 11회 → inputs_ok 29초 off).
constexpr int kGyroCrossFailsPerCamera = static_cast<int>(kImuFrequency / kCameraFrequency);
constexpr double kSensorAliveS = 0.1;
// 카메라를 한 번도 못 받은 채(부팅 직후 차속을 모름) IMU만으로 돌면 속도가 묶이지 않아 자세가
// 틀어진다(30초 4~5°, 120초 40~55°, 그래도 std는 실제 오차보다 작다). 그 시간이 이보다 길면 첫 카메라
// 관측에서 필터를 처음부터 시작하고, 첫 관측 뒤 kAnchorSettleS 동안은 무효로 낸다(상류는 입력이
// 다 갖춰질 때까지 아무것도 넣지 않는다). 한 번 수렴한 뒤의 가드 구간은 30초여도 자세가 그대로라
// 해당하지 않는다.
constexpr double kImuOnlyResetS = 5.0;
constexpr double kAnchorSettleS = 3.0;
// 차속 가드(상류에 없음): 카메라 전진 속도와 CAN 차속 차이가 이보다 크면 관측을 쓰지 않는다.
// 2026-09-27 재생에서 모델/바퀴 속도 비 중앙값 0.967.
constexpr double kCamSpeedErrAbs = 2.0;  // m/s
constexpr double kCamSpeedErrRel = 0.3;
constexpr double kStoppedSpeed = 0.3;         // m/s, 이하면 정차
constexpr double kStoppedMaxRotation = 0.05;  // rad/s, 정차 중 카메라 회전 한도
constexpr double kCarSpeedMaxAgeS = 1.0;

using Vec3 = PoseKalman::Vec3;
using Mat3 = std::array<double, 9>;

// rednose sympy_helpers.euler_rotate와 같다: Rz(yaw) · Ry(pitch) · Rx(roll)
Mat3 euler_rotate(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    return {cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr};
}

Vec3 rot_to_euler(const Mat3 &r) {
    return {std::atan2(r[7], r[8]), std::asin(std::clamp(-r[6], -1.0, 1.0)), std::atan2(r[3], r[0])};
}

Mat3 mul(const Mat3 &a, const Mat3 &b) {
    Mat3 c{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) c[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
    return c;
}

Mat3 transpose(const Mat3 &a) { return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]}; }

Vec3 mul(const Mat3 &a, const Vec3 &v) {
    return {a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
            a[6] * v[0] + a[7] * v[1] + a[8] * v[2]};
}

// helpers.rotate_std와 같다: sqrt(diag(R · diag(std²) · Rᵀ))
Vec3 rotate_std(const Mat3 &r, const Vec3 &std) {
    Vec3 out{};
    for (int i = 0; i < 3; ++i) {
        double s = 0.0;
        for (int k = 0; k < 3; ++k) s += r[i * 3 + k] * r[i * 3 + k] * std[k] * std[k];
        out[i] = std::sqrt(s);
    }
    return out;
}

double norm(const Vec3 &v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

double wrap_angle(double a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

double decay_for(double limit, double frequency) {
    return std::pow(1.0 - 1.0 / (2.0 * limit), 1.0 / (kInputInvalidRecovery * frequency));
}

}  // namespace

// ---------------------------------------------------------------- PoseKalman

PoseKalman::Vec PoseKalman::initial_x() { return Vec{}; }

PoseKalman::Mat PoseKalman::initial_p() {
    const double d[kN] = {0.01 * 0.01,   0.01 * 0.01,   0.01 * 0.01,   10.0 * 10.0, 10.0 * 10.0, 10.0 * 10.0,
                          1.0,           1.0,           1.0,           0.1 * 0.1,   0.1 * 0.1,   0.1 * 0.1,
                          100.0 * 100.0, 100.0 * 100.0, 100.0 * 100.0, 0.01 * 0.01, 0.01 * 0.01, 0.01 * 0.01};
    Mat p{};
    for (int i = 0; i < kN; ++i) p[i][i] = d[i];
    return p;
}

PoseKalman::Mat PoseKalman::q() {
    const double gb = 0.005 / 100.0;
    const double d[kN] = {0.001 * 0.001, 0.001 * 0.001, 0.001 * 0.001, 0.01 * 0.01,   0.01 * 0.01,   0.01 * 0.01,
                          0.085 * 0.085, 0.085 * 0.085, 0.085 * 0.085, gb * gb,       gb * gb,       gb * gb,
                          3.0 * 3.0,     3.0 * 3.0,     3.0 * 3.0,     0.005 * 0.005, 0.005 * 0.005, 0.005 * 0.005};
    Mat m{};
    for (int i = 0; i < kN; ++i) m[i][i] = d[i];
    return m;
}

PoseKalman::Vec3 PoseKalman::default_noise(Kind kind) {
    switch (kind) {
        case Kind::Gyro:
            return {0.025 * 0.025, 0.025 * 0.025, 0.025 * 0.025};
        case Kind::Accel:
            return {0.75 * 0.75, 0.75 * 0.75, 0.75 * 0.75};
        case Kind::CameraTranslation:
            return {0.5 * 0.5, 0.5 * 0.5, 0.5 * 0.5};
        case Kind::CameraRotation:
            return {0.05 * 0.05, 0.05 * 0.05, 0.05 * 0.05};
    }
    return {};
}

void PoseKalman::init(const Vec &x, const Mat &p, double t) {
    x_ = x;
    p_ = p;
    t_ = t;
    history_.clear();
}

PoseKalman::Vec PoseKalman::f(const Vec &x, double dt) {
    Vec out = x;
    for (int i = 0; i < 3; ++i) out[kVelocity + i] = x[kVelocity + i] + dt * x[kAcceleration + i];
    const Mat3 ned_from_device = euler_rotate(x[kOrientation], x[kOrientation + 1], x[kOrientation + 2]);
    const Mat3 device_from_device_t1 =
        euler_rotate(dt * x[kAngularVelocity], dt * x[kAngularVelocity + 1], dt * x[kAngularVelocity + 2]);
    const Vec3 e = rot_to_euler(mul(ned_from_device, device_from_device_t1));
    for (int i = 0; i < 3; ++i) out[kOrientation + i] = e[i];
    return out;
}

PoseKalman::Vec3 PoseKalman::h(Kind kind, const Vec &x) {
    const Vec3 w = {x[kAngularVelocity], x[kAngularVelocity + 1], x[kAngularVelocity + 2]};
    switch (kind) {
        case Kind::Gyro:
            return {w[0] + x[kGyroBias], w[1] + x[kGyroBias + 1], w[2] + x[kGyroBias + 2]};
        case Kind::CameraRotation:
            return w;
        case Kind::CameraTranslation:
            return {x[kVelocity], x[kVelocity + 1], x[kVelocity + 2]};
        case Kind::Accel: {
            const Mat3 device_from_ned =
                transpose(euler_rotate(x[kOrientation], x[kOrientation + 1], x[kOrientation + 2]));
            const Vec3 g = mul(device_from_ned, Vec3{0.0, 0.0, -kEarthG});
            const Vec3 v = {x[kVelocity], x[kVelocity + 1], x[kVelocity + 2]};
            const Vec3 cross = {w[1] * v[2] - w[2] * v[1], w[2] * v[0] - w[0] * v[2], w[0] * v[1] - w[1] * v[0]};
            Vec3 out{};
            for (int i = 0; i < 3; ++i) out[i] = g[i] + x[kAcceleration + i] + cross[i] + x[kAccelBias + i];
            return out;
        }
    }
    return {};
}

// 야코비안은 0이 아닌 열만 수치 미분한다. f에서 비선형인 것은 자세 행(자세·각속도 열)뿐이고
// 속도는 가속도에 선형, 나머지는 항등이다. h는 가속도계만 자세·각속도·속도에 비선형이고
// 나머지는 상태를 그대로 고른다. 보드(A53 2코어)에서 되감기가 초당 수천 번 갱신을 다시
// 하므로, 18열 전부를 미분하고 조밀 행렬로 곱하던 것보다 수 배 가볍다. 결과는 같다.
void PoseKalman::predict(double dt) {  // dt > 0: apply가 시각이 앞으로 갈 때만 부른다
    constexpr double kEps = 1e-6;
    // F = I + (자세 행: 자세·각속도 열 수치 미분 − I) + (속도 행: 가속도 열 dt)
    double fo[3][6];  // 자세 행 × {자세 0..2, 각속도 6..8}
    constexpr int kCols[6] = {kOrientation,     kOrientation + 1,     kOrientation + 2,
                              kAngularVelocity, kAngularVelocity + 1, kAngularVelocity + 2};
    for (int c = 0; c < 6; ++c) {
        Vec xp = x_, xm = x_;
        xp[kCols[c]] += kEps;
        xm[kCols[c]] -= kEps;
        const Vec fp = f(xp, dt), fm = f(xm, dt);
        for (int i = 0; i < 3; ++i) fo[i][c] = wrap_angle(fp[kOrientation + i] - fm[kOrientation + i]) / (2 * kEps);
    }
    x_ = f(x_, dt);
    // FP: 자세 행만 새로 계산하고, 속도 행은 P[v] + dt·P[a], 나머지 행은 P 그대로
    Mat fp = p_;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < kN; ++j) {
            double v = 0.0;
            for (int c = 0; c < 6; ++c) v += fo[i][c] * p_[kCols[c]][j];
            fp[kOrientation + i][j] = v;
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < kN; ++j) fp[kVelocity + i][j] += dt * p_[kAcceleration + i][j];
    // P = (FP) Fᵀ: 같은 방식으로 열에 적용한다
    p_ = fp;
    for (int i = 0; i < kN; ++i) {
        for (int r = 0; r < 3; ++r) {
            double v = 0.0;
            for (int c = 0; c < 6; ++c) v += fp[i][kCols[c]] * fo[r][c];
            p_[i][kOrientation + r] = v;
        }
        for (int r = 0; r < 3; ++r) p_[i][kVelocity + r] += dt * fp[i][kAcceleration + r];
    }
    const Mat qq = q();
    for (int i = 0; i < kN; ++i)
        for (int j = 0; j < kN; ++j) p_[i][j] += dt * qq[i][j];
}

void PoseKalman::update(Kind kind, const Vec3 &z, const Vec3 &r) {
    constexpr double kEps = 1e-6;
    std::array<std::array<double, kN>, 3> hj{};
    const auto select = [&hj](int base) {
        for (int i = 0; i < 3; ++i) hj[i][base + i] = 1.0;
    };
    switch (kind) {
        case Kind::Gyro:
            select(kAngularVelocity);
            select(kGyroBias);
            break;
        case Kind::CameraRotation:
            select(kAngularVelocity);
            break;
        case Kind::CameraTranslation:
            select(kVelocity);
            break;
        case Kind::Accel:
            select(kAcceleration);
            select(kAccelBias);
            for (int j = kOrientation; j < kAngularVelocity + 3; ++j) {
                Vec xp = x_, xm = x_;
                xp[j] += kEps;
                xm[j] -= kEps;
                const Vec3 a = h(kind, xp), b = h(kind, xm);
                for (int i = 0; i < 3; ++i) hj[i][j] = (a[i] - b[i]) / (2 * kEps);
            }
            break;
    }
    const Vec3 hx = h(kind, x_);
    // PHᵀ (18x3), S = H P Hᵀ + R (3x3)
    std::array<std::array<double, 3>, kN> pht{};
    for (int i = 0; i < kN; ++i)
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < kN; ++k)
                if (hj[c][k] != 0.0) s += p_[i][k] * hj[c][k];
            pht[i][c] = s;
        }
    double s[3][3];
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) {
            double v = a == b ? r[a] : 0.0;
            for (int k = 0; k < kN; ++k) v += hj[a][k] * pht[k][b];
            s[a][b] = v;
        }
    const double det = s[0][0] * (s[1][1] * s[2][2] - s[1][2] * s[2][1]) -
                       s[0][1] * (s[1][0] * s[2][2] - s[1][2] * s[2][0]) +
                       s[0][2] * (s[1][0] * s[2][1] - s[1][1] * s[2][0]);
    // S = H P Hᵀ + R(R > 0)은 양의 정부호라 det > 0이다
    const double si[3][3] = {
        {(s[1][1] * s[2][2] - s[1][2] * s[2][1]) / det, (s[0][2] * s[2][1] - s[0][1] * s[2][2]) / det,
         (s[0][1] * s[1][2] - s[0][2] * s[1][1]) / det},
        {(s[1][2] * s[2][0] - s[1][0] * s[2][2]) / det, (s[0][0] * s[2][2] - s[0][2] * s[2][0]) / det,
         (s[0][2] * s[1][0] - s[0][0] * s[1][2]) / det},
        {(s[1][0] * s[2][1] - s[1][1] * s[2][0]) / det, (s[0][1] * s[2][0] - s[0][0] * s[2][1]) / det,
         (s[0][0] * s[1][1] - s[0][1] * s[1][0]) / det}};
    std::array<std::array<double, 3>, kN> k{};
    for (int i = 0; i < kN; ++i)
        for (int c = 0; c < 3; ++c) k[i][c] = pht[i][0] * si[0][c] + pht[i][1] * si[1][c] + pht[i][2] * si[2][c];
    const Vec3 y = {z[0] - hx[0], z[1] - hx[1], z[2] - hx[2]};
    for (int i = 0; i < kN; ++i) x_[i] += k[i][0] * y[0] + k[i][1] * y[1] + k[i][2] * y[2];
    // P ← P − K (H P) = P − K S Kᵀ. P가 대칭이라 H P = (P Hᵀ)ᵀ. 반올림으로 대칭이 깨지지
    // 않도록 위 삼각형만 계산해 복사한다(Joseph 형식과 같은 값, 곱셈은 1/12).
    for (int i = 0; i < kN; ++i)
        for (int j = i; j < kN; ++j) {
            const double v = p_[i][j] - (k[i][0] * pht[j][0] + k[i][1] * pht[j][1] + k[i][2] * pht[j][2]);
            p_[i][j] = v;
            p_[j][i] = v;
        }
}

void PoseKalman::apply(double t, Kind kind, const Vec3 &z, const Vec3 &r) {
    if (std::isnan(t_))
        t_ = t;
    else if (t > t_) {
        predict(t - t_);
        t_ = t;
    }
    update(kind, z, r);
}

bool PoseKalman::predict_and_observe(double t, std::initializer_list<Observation> observations) {
    if (!std::isnan(t_) && t_ - t > max_rewind_s_) return false;
    if (std::isnan(t_) || t >= t_) {
        for (const Observation &o : observations) {
            history_.push_back({t, o.kind, o.z, o.r, t_, x_, p_});
            apply(t, o.kind, o.z, o.r);
        }
    } else {
        // 되감기: t보다 뒤인 첫 관측 직전 상태로 돌아가 새 관측들을 넣고, 뒤의 관측을 다시 적용한다. 직전 상태는
        // 아래 루프가 채운다.
        auto it = std::find_if(history_.begin(), history_.end(), [t](const Entry &e) { return e.t > t; });
        if (it == history_.end()) return false;
        t_ = it->t_before;
        x_ = it->x_before;
        p_ = it->p_before;
        const size_t index = static_cast<size_t>(it - history_.begin());
        size_t at = index;
        for (const Observation &o : observations)
            history_.insert(history_.begin() + static_cast<std::ptrdiff_t>(at++), {t, o.kind, o.z, o.r, t_, x_, p_});
        for (size_t i = index; i < history_.size(); ++i) {
            Entry &e = history_[i];
            e.t_before = t_;
            e.x_before = x_;
            e.p_before = p_;
            apply(e.t, e.kind, e.z, e.r);
        }
    }
    while (!history_.empty() && history_.front().t < t_ - max_rewind_s_) history_.pop_front();
    return true;
}

// ---------------------------------------------------------------- LocationEstimator

// MaixCAM2 IMU 칩이 카메라 대비 기울어 붙어 있다(2026-09-27, 2026-10-01 녹화의 선회 중 자이로
// 회전축 대 카메라 pose 회전축):
//   pitch(IMU 앞이 들림): 축 기울기 차 +2.15/+2.37°, Wahba +1.96/+2.22° → +2.2°
//   roll(IMU 오른쪽이 내려감): +0.20/+1.38°, Wahba +0.57/+1.81° → +1.0°(모델 pose의 롤 성분
//     오차가 커서 ±0.6° 정도 불확실)
//   yaw는 수직축 선회로 관측되지 않아 0.
// 보정하지 않으면 평지에서 roll·pitch가 약 +1°/+2.2°로 나온다.
const double LocationEstimator::kDefaultImuExtrinsicRpy[3] = {1.0 * kPi / 180.0, 2.2 * kPi / 180.0, 0.0};

LocationEstimator::LocationEstimator() : kf_(kMaxFilterRewindTime) {
    set_imu_extrinsic(kDefaultImuExtrinsicRpy);
    posenet_stds_.fill(kPosenetStdInitial);
    const double freq[kServiceCount] = {kImuFrequency, kImuFrequency, kCameraFrequency};
    for (int s = 0; s < kServiceCount; ++s) {
        const double limit = std::round(kInputInvalidLimit * (freq[s] / 20.0));
        invalid_threshold_[s] = limit - 0.5;
        invalid_decay_[s] = decay_for(limit, freq[s]);
    }
}

void LocationEstimator::note_result(Service service, bool success) {
    if (success)
        invalid_[service] *= invalid_decay_[service];
    else
        invalid_[service] += 1.0;
}

bool LocationEstimator::timestamp_ok(double t) const {
    return std::isnan(kf_.t()) || kf_.t() - t <= kMaxFilterRewindTime;
}

void LocationEstimator::finite_check(double t) {
    bool finite = true;
    for (double v : kf_.x()) finite = finite && std::isfinite(v);
    for (const auto &row : kf_.p())
        for (double v : row) finite = finite && std::isfinite(v);
    if (!finite) kf_.init(PoseKalman::initial_x(), PoseKalman::initial_p(), t);
}

void LocationEstimator::handle_imu(double t, const float accel_chip[3], const float gyro_chip[3]) {
    // MaixCAM2 LSM6DSOW 칩 축: y 위, z 뒤, x 오른쪽(2026-09-27 녹화로 확인: 정차 중 중력이 +y,
    // 가속 때 −z, 좌회전 원심 반응이 −x, 자이로 y가 CAN 요레이트와 상관 0.93·배율 1.00).
    // 기기 좌표계(x 앞, y 오른쪽, z 아래)로: (−z, x, −y).
    // 그 뒤 칩이 카메라 대비 기운 만큼 돌린다(set_imu_extrinsic).
    const Vec3 gyro = mul(device_from_imu_, Vec3{-gyro_chip[2], gyro_chip[0], -gyro_chip[1]});
    const Vec3 accel = mul(device_from_imu_, Vec3{-accel_chip[2], accel_chip[0], -accel_chip[1]});
    seen_imu_ = true;
    if (first_imu_t_ < 0.0) first_imu_t_ = t;
    last_imu_t_ = t;

    if (!timestamp_ok(t)) {
        ++counters_.gyro_timestamp;
        ++counters_.accel_timestamp;
        note_result(kGyro, false);
        note_result(kAccel, false);
        return;
    }
    const double bias_z = kf_.x()[PoseKalman::kGyroBias + 2];
    const bool gyro_valid =
        std::fabs((gyro[2] - bias_z) - camodo_yawrate_[0]) < kYawrateCrossErrCheckFactor * camodo_yawrate_[1];
    if (norm(gyro) >= kRotationSanityCheck || !gyro_valid) {
        ++(gyro_valid ? counters_.gyro_sanity : counters_.gyro_cross_check);
        if (gyro_valid || gyro_cross_fails_++ < kGyroCrossFailsPerCamera) note_result(kGyro, false);
    } else {
        const bool ok =
            kf_.predict_and_observe(t, PoseKalman::Kind::Gyro, gyro, PoseKalman::default_noise(PoseKalman::Kind::Gyro));
        ++(ok ? counters_.gyro_ok : counters_.gyro_filter);
        note_result(kGyro, ok);
    }
    if (norm(accel) >= kAccelSanityCheck) {
        ++counters_.accel_sanity;
        note_result(kAccel, false);
    } else {
        const bool ok = kf_.predict_and_observe(t, PoseKalman::Kind::Accel, accel,
                                                PoseKalman::default_noise(PoseKalman::Kind::Accel));
        ++(ok ? counters_.accel_ok : counters_.accel_filter);
        note_result(kAccel, ok);
    }
    finite_check(t);
}

void LocationEstimator::set_imu_extrinsic(const double rpy[3]) {
    device_from_imu_ = euler_rotate(rpy[0], rpy[1], rpy[2]);
}

void LocationEstimator::handle_calibration(const double rpy[3]) {
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(rpy[i]) || std::fabs(rpy[i]) > kCalibRpySanityCheck) return;
    device_from_calib_ = euler_rotate(rpy[0], rpy[1], rpy[2]);
}

void LocationEstimator::handle_camera_odometry(double t_capture, const float trans[3], const float rot[3],
                                               const float trans_std[3], const float rot_std[3]) {
    const double t = t_capture - kCamOdoPoseDelay;
    if (!timestamp_ok(t)) {
        ++counters_.camera_timestamp;
        note_result(kCamera, false);
        return;
    }
    const Vec3 rot_device = mul(device_from_calib_, Vec3{rot[0], rot[1], rot[2]});
    const Vec3 trans_device = mul(device_from_calib_, Vec3{trans[0], trans[1], trans[2]});
    Vec3 rot_calib_std = {rot_std[0], rot_std[1], rot_std[2]};
    Vec3 trans_calib_std = {trans_std[0], trans_std[1], trans_std[2]};
    if (norm(rot_device) > kRotationSanityCheck || norm(trans_device) > kTransSanityCheck ||
        *std::min_element(rot_calib_std.begin(), rot_calib_std.end()) <= kMinStdSanityCheck ||
        *std::min_element(trans_calib_std.begin(), trans_calib_std.end()) <= kMinStdSanityCheck ||
        norm(rot_calib_std) > 10 * kRotationSanityCheck || norm(trans_calib_std) > 10 * kTransSanityCheck ||
        !std::isfinite(norm(rot_device) + norm(trans_device))) {
        ++counters_.camera_sanity;
        note_result(kCamera, false);
        return;
    }
    // 차속 가드: 모델이 차속과 다른 움직임을 말하면(정차 중 가짜 움직임 등) 관측을 버린다.
    // 차속을 모르면(부팅 직후 controlsd·CAN 전, CAN 끊김) 대조할 수 없으니 쓰지 않는다
    // (2026-10-02 부팅 30초 동안 가짜 20 m/s로 롤 −70°·피치 −54°까지 틀어지고 자이로 거부가
    // 쌓여 7분 가까이 inputs_ok가 떨어졌다). 입력 이상으로 세지 않고(inputs_ok 유지), 교차검사
    // 기준도 풀어 자이로가 버려지지 않게 한다.
    bool guard = !car_speed_ok(t);
    if (!guard) {
        const double speed_err = std::fabs(static_cast<double>(trans[0]) - car_speed_);
        const bool fake_rotation = car_speed_ <= kStoppedSpeed && norm(rot_device) > kStoppedMaxRotation;
        guard = speed_err > std::max(kCamSpeedErrAbs, kCamSpeedErrRel * car_speed_) || fake_rotation;
    }
    if (guard) {
        ++counters_.camera_speed_guard;
        camera_guarded_ = true;
        camodo_yawrate_ = {0.0, 10.0};
        gyro_cross_fails_ = 0;
        return;
    }
    camera_guarded_ = false;
    if (!seen_camera_) {
        if (first_imu_t_ >= 0.0 && t - first_imu_t_ > kImuOnlyResetS)
            kf_.init(PoseKalman::initial_x(), PoseKalman::initial_p(), t);
        first_camera_t_ = t;
    }
    seen_camera_ = true;
    std::rotate(posenet_stds_.begin(), posenet_stds_.begin() + 1, posenet_stds_.end());
    posenet_stds_.back() = trans_calib_std[0];

    // 시간 상관 잡음 때문에 칼만 필터가 과신하지 않도록 키운다.
    for (double &v : rot_calib_std) v *= kCamOdoRotStdMult;
    for (double &v : trans_calib_std) v *= kCamOdoTransStdMult;
    const Vec3 rot_device_std = rotate_std(device_from_calib_, rot_calib_std);
    const Vec3 trans_device_std = rotate_std(device_from_calib_, trans_calib_std);
    Vec3 rot_noise{}, trans_noise{};
    for (int i = 0; i < 3; ++i) {
        rot_noise[i] = rot_device_std[i] * rot_device_std[i];
        trans_noise[i] = trans_device_std[i] * trans_device_std[i];
    }
    // 회전과 이동은 같은 시각이라 한 번에 넣는다(되감기 한 번)
    const bool ok = kf_.predict_and_observe(t, {{PoseKalman::Kind::CameraRotation, rot_device, rot_noise},
                                                {PoseKalman::Kind::CameraTranslation, trans_device, trans_noise}});
    camodo_yawrate_ = {rot_device[2], rot_device_std[2]};
    gyro_cross_fails_ = 0;
    ++(ok ? counters_.camera_ok : counters_.camera_filter);
    note_result(kCamera, ok);
    finite_check(t);
}

bool LocationEstimator::car_speed_ok(double t) const {
    return car_speed_valid_ && car_speed_t_ >= 0.0 && std::fabs(t - car_speed_t_) <= kCarSpeedMaxAgeS;
}

uint32_t LocationEstimator::invalid_service_mask() const {
    uint32_t mask = 0;
    for (int s = 0; s < kServiceCount; ++s)
        if (invalid_[s] >= invalid_threshold_[s]) mask |= 1U << s;
    return mask;
}

LivePoseEstimate LocationEstimator::estimate(double now) const {
    LivePoseEstimate out;
    const auto &x = kf_.x();
    const auto &p = kf_.p();
    out.t = kf_.t();
    for (int i = 0; i < 3; ++i) {
        out.orientation_ned[i] = x[PoseKalman::kOrientation + i];
        out.velocity_device[i] = x[PoseKalman::kVelocity + i];
        out.angular_velocity_device[i] = x[PoseKalman::kAngularVelocity + i];
        out.acceleration_device[i] = x[PoseKalman::kAcceleration + i];
        out.orientation_ned_std[i] = std::sqrt(p[PoseKalman::kOrientation + i][PoseKalman::kOrientation + i]);
        out.velocity_device_std[i] = std::sqrt(p[PoseKalman::kVelocity + i][PoseKalman::kVelocity + i]);
        out.angular_velocity_device_std[i] =
            std::sqrt(p[PoseKalman::kAngularVelocity + i][PoseKalman::kAngularVelocity + i]);
        out.acceleration_device_std[i] = std::sqrt(p[PoseKalman::kAcceleration + i][PoseKalman::kAcceleration + i]);
    }
    out.filter_valid =
        seen_imu_ && seen_camera_ && std::isfinite(kf_.t()) && kf_.t() - first_camera_t_ >= kAnchorSettleS;
    bool inputs_ok = true;
    for (int s = 0; s < kServiceCount; ++s) inputs_ok = inputs_ok && invalid_[s] < invalid_threshold_[s];
    out.inputs_ok = inputs_ok;
    out.sensors_ok = last_imu_t_ >= 0.0 && now - last_imu_t_ < kSensorAliveS;
    const double old_mean =
        std::accumulate(posenet_stds_.begin(), posenet_stds_.begin() + kPosenetStdHistHalf, 0.0) / kPosenetStdHistHalf;
    const double new_mean =
        std::accumulate(posenet_stds_.begin() + kPosenetStdHistHalf, posenet_stds_.end(), 0.0) / kPosenetStdHistHalf;
    const bool std_spike = new_mean / old_mean > 4.0 && new_mean > 7.0;
    out.posenet_ok = !std_spike || car_speed_ <= 5.0;
    return out;
}

CalibratedPose calibrate_pose(const LivePoseEstimate &pose, const double calib_rpy[3]) {
    const Mat3 device_from_calib = euler_rotate(calib_rpy[0], calib_rpy[1], calib_rpy[2]);
    const Mat3 calib_from_device = transpose(device_from_calib);
    CalibratedPose out;
    const Mat3 ned_from_device =
        euler_rotate(pose.orientation_ned[0], pose.orientation_ned[1], pose.orientation_ned[2]);
    out.orientation = rot_to_euler(mul(ned_from_device, device_from_calib));
    out.angular_velocity = mul(calib_from_device, Vec3{pose.angular_velocity_device[0], pose.angular_velocity_device[1],
                                                       pose.angular_velocity_device[2]});
    out.angular_velocity_std =
        rotate_std(calib_from_device, Vec3{pose.angular_velocity_device_std[0], pose.angular_velocity_device_std[1],
                                           pose.angular_velocity_device_std[2]});
    out.acceleration = mul(calib_from_device,
                           Vec3{pose.acceleration_device[0], pose.acceleration_device[1], pose.acceleration_device[2]});
    return out;
}
