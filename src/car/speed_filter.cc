#include "car/speed_filter.h"

#include <cmath>
#include <limits>

namespace {

// opendbc CarStateBase: A = [[1, DT_CTRL], [0, 1]], C = [1, 0], Q = [[0, 0], [0, 100]], R = 0.3
constexpr double kDtCtrl = 0.01;
constexpr double kAccelProcessNoise = 100.0;
constexpr double kSpeedMeasurementNoise = 0.3;
constexpr double kRestartGapMps = 2.0;

}  // namespace

// 상류 get_kalman_gain(iterations=100): 리카티 식을 반복해 정상 상태 이득을 구한다.
SpeedFilter::SpeedFilter() {
  double p00 = 0.0, p01 = 0.0, p10 = 0.0, p11 = 0.0;
  for (int i = 0; i < 100; ++i) {
    // P = A P Aᵀ + dt Q
    const double a00 = p00 + kDtCtrl * (p01 + p10) + kDtCtrl * kDtCtrl * p11;
    const double a01 = p01 + kDtCtrl * p11;
    const double a10 = p10 + kDtCtrl * p11;
    const double a11 = p11 + kDtCtrl * kAccelProcessNoise;
    // K = P Cᵀ / (C P Cᵀ + R), P = (I − K C) P
    const double s = a00 + kSpeedMeasurementNoise;
    gain_v_ = a00 / s;
    gain_a_ = a10 / s;
    p00 = (1.0 - gain_v_) * a00;
    p01 = (1.0 - gain_v_) * a01;
    p10 = a10 - gain_a_ * a00;
    p11 = a11 - gain_a_ * a01;
  }
}

double SpeedFilter::update(double v_ego_raw) {
  if (!std::isfinite(v_ego_raw)) {
    started_ = false;
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!started_ || std::fabs(v_ego_raw - v_ego_) > kRestartGapMps) {
    v_ego_ = v_ego_raw;
    a_ego_ = 0.0;
    started_ = true;
  }
  // 상류 KF1D.update: x = (A − K C) x + K z
  const double v_ego = (1.0 - gain_v_) * v_ego_ + kDtCtrl * a_ego_ + gain_v_ * v_ego_raw;
  a_ego_ = -gain_a_ * v_ego_ + a_ego_ + gain_a_ * v_ego_raw;
  v_ego_ = v_ego;
  return v_ego_;
}
