#include "model/flow_calibration.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "common/utils_math.h"

namespace {

constexpr int kMinPoints = 50;
constexpr int kMinInliers = 40;
constexpr double kMaxRotation = 0.5 * kPi / 180.0;  // 창(0.4초) 동안. 곧은 길에서는 0.1° 안팎이다
constexpr double kMaxAngle = 10.0 * kPi / 180.0;    // 이보다 크면 풀이가 엇나간 것
constexpr double kSigmaPx = 1.0;                    // 첫 풀이의 Cauchy 척도, 맞는 점의 기준(2배)
constexpr double kMinScalePx = 0.1;                 // 다시 풀 때 강건 척도의 하한
constexpr int kIterations = 8;

// 곧은 길: 40 km/h 이상, 요레이트 0.25°/s 이하(100 km/h에서 곡률 1.4e-4), 깜빡이·desire 없음
constexpr float kMinSpeed = 40.0f / 3.6f;
constexpr float kMaxYawRate = 0.25f * static_cast<float>(kPi) / 180.0f;

void rodrigues(const double w[3], double r[9]) {
    const double th = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (th < 1e-12) {
        for (int i = 0; i < 9; ++i) r[i] = i % 4 == 0 ? 1.0 : 0.0;
        return;
    }
    const double kx = w[0] / th, ky = w[1] / th, kz = w[2] / th;
    const double s = std::sin(th), c = 1.0 - std::cos(th);
    // I + sin·K + (1 - cos)·K²
    r[0] = 1.0 + c * (-ky * ky - kz * kz);
    r[1] = -s * kz + c * kx * ky;
    r[2] = s * ky + c * kx * kz;
    r[3] = s * kz + c * kx * ky;
    r[4] = 1.0 + c * (-kx * kx - kz * kz);
    r[5] = -s * kx + c * ky * kz;
    r[6] = -s * ky + c * kx * kz;
    r[7] = s * kx + c * ky * kz;
    r[8] = 1.0 + c * (-kx * kx - ky * ky);
}

// 파라미터 p = (a, b, wx, wy, wz)에서 Sampson으로 정규화한 에피폴라 잔차(정규화 영상 단위). E = R(w)[m]x,
// m = (a, b, 1). x1, x2는 (x, y, 1) 정규화 좌표를 x, y만 담는다.
void residuals(const double p[5], const std::vector<double> &x1, const std::vector<double> &x2, double *e) {
    double r[9];
    rodrigues(p + 2, r);
    const double m0 = p[0], m1 = p[1], m2 = 1.0;
    // [m]x
    const double mx[9] = {0.0, -m2, m1, m2, 0.0, -m0, -m1, m0, 0.0};
    double ess[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ess[i * 3 + j] = r[i * 3 + 0] * mx[0 * 3 + j] + r[i * 3 + 1] * mx[1 * 3 + j] + r[i * 3 + 2] * mx[2 * 3 + j];
    const size_t n = x1.size() / 2;
    for (size_t i = 0; i < n; ++i) {
        const double ax = x1[2 * i], ay = x1[2 * i + 1], bx = x2[2 * i], by = x2[2 * i + 1];
        const double e0 = ess[0] * ax + ess[1] * ay + ess[2];  // E x1
        const double e1 = ess[3] * ax + ess[4] * ay + ess[5];
        const double e2 = ess[6] * ax + ess[7] * ay + ess[8];
        const double t0 = ess[0] * bx + ess[3] * by + ess[6];  // Eᵀ x2
        const double t1 = ess[1] * bx + ess[4] * by + ess[7];
        const double num = bx * e0 + by * e1 + e2;
        const double den = e0 * e0 + e1 * e1 + t0 * t0 + t1 * t1;
        e[i] = num / std::sqrt(std::max(den, 1e-18));
    }
}

// 5x5 대칭 양정치 연립방정식(작은 능선 포함)을 부분 피벗 가우스 소거로 푼다
bool solve5(double a[25], double b[5], double x[5]) {
    for (int c = 0; c < 5; ++c) {
        int piv = c;
        for (int r = c + 1; r < 5; ++r)
            if (std::fabs(a[r * 5 + c]) > std::fabs(a[piv * 5 + c])) piv = r;
        if (std::fabs(a[piv * 5 + c]) < 1e-300) return false;
        if (piv != c) {
            for (int k = 0; k < 5; ++k) std::swap(a[c * 5 + k], a[piv * 5 + k]);
            std::swap(b[c], b[piv]);
        }
        for (int r = c + 1; r < 5; ++r) {
            const double f = a[r * 5 + c] / a[c * 5 + c];
            for (int k = c; k < 5; ++k) a[r * 5 + k] -= f * a[c * 5 + k];
            b[r] -= f * b[c];
        }
    }
    for (int r = 4; r >= 0; --r) {
        double s = b[r];
        for (int k = r + 1; k < 5; ++k) s -= a[r * 5 + k] * x[k];
        x[r] = s / a[r * 5 + r];
    }
    return true;
}

float median_of(std::vector<float> v) {
    if (v.empty()) return 0.0f;
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
    const float hi = v[mid];
    if (v.size() % 2) return hi;
    return 0.5f * (hi + *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid)));
}

}  // namespace

namespace {

// 가중 가우스-뉴턴. use가 비어 있으면 모든 점을, 아니면 use[i]인 점만 쓴다. 잔차 척도 sigma의 Cauchy 가중.
// 수치 미분(5개 파라미터)이라 점 300개 x 8회에 1~2 ms(A53)다.
bool gauss_newton(double p[5], const std::vector<double> &x1, const std::vector<double> &x2,
                  const std::vector<char> &use, double sigma, int iterations) {
    const size_t n = x1.size() / 2;
    const double step[5] = {1e-6, 1e-6, 1e-7, 1e-7, 1e-7};
    std::vector<double> e(n), ep(n), jac(5 * n);
    for (int it = 0; it < iterations; ++it) {
        residuals(p, x1, x2, e.data());
        for (int c = 0; c < 5; ++c) {
            double q[5] = {p[0], p[1], p[2], p[3], p[4]};
            q[c] += step[c];
            residuals(q, x1, x2, ep.data());
            for (size_t i = 0; i < n; ++i) jac[i * 5 + c] = (ep[i] - e[i]) / step[c];
        }
        double a[25] = {}, g[5] = {};
        for (size_t i = 0; i < n; ++i) {
            if (!use.empty() && !use[i]) continue;
            const double z = e[i] / sigma;
            const double w = 1.0 / (1.0 + z * z);
            const double *ji = &jac[i * 5];
            for (int r = 0; r < 5; ++r) {
                g[r] += w * ji[r] * e[i];
                for (int c = r; c < 5; ++c) a[r * 5 + c] += w * ji[r] * ji[c];
            }
        }
        for (int r = 0; r < 5; ++r) {
            a[r * 5 + r] += 1e-12;
            for (int c = 0; c < r; ++c) a[r * 5 + c] = a[c * 5 + r];
            g[r] = -g[r];
        }
        double d[5];
        if (!solve5(a, g, d)) return false;
        for (int c = 0; c < 5; ++c) p[c] += d[c];
        if (std::max(std::fabs(d[0]), std::fabs(d[1])) < 1e-7) break;
    }
    return true;
}

}  // namespace

FlowSolution solve_flow_direction(const FlowIntrinsics &k, const float *start_xy, const float *end_xy, int n) {
    FlowSolution out;
    out.points = n;
    if (n < kMinPoints) return out;
    {
        std::vector<float> moved(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i)
            moved[i] = std::hypot(end_xy[2 * i] - start_xy[2 * i], end_xy[2 * i + 1] - start_xy[2 * i + 1]);
        out.parallax_px = median_of(std::move(moved));
        if (!(out.parallax_px >= kFlowMinParallaxPx)) return out;  // 움직임이 작으면 방향이 서지 않는다
    }
    std::vector<double> x1(2 * static_cast<size_t>(n)), x2(2 * static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        x1[2 * i] = (start_xy[2 * i] - k.cx) / k.fx;
        x1[2 * i + 1] = (start_xy[2 * i + 1] - k.cy) / k.fy;
        x2[2 * i] = (end_xy[2 * i] - k.cx) / k.fx;
        x2[2 * i + 1] = (end_xy[2 * i + 1] - k.cy) / k.fy;
    }
    const double sigma = kSigmaPx / k.fx;
    double p[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<char> use;
    if (!gauss_newton(p, x1, x2, use, sigma, kIterations)) return out;
    // 크게 빗나간 추적이 섞이면(IVE 경로는 왕복 검사가 없다) 1 px 척도의 Cauchy 가중만으로는 창마다 흩어짐이
    // 15배 커진다(합성 점, 이상치 10%). 잔차의 강건 척도(1.4826·MAD, 0.1 px 이상)를 재서 그 3배를 넘는 점을
    // 빼고, 그 척도로 다시 푼다. 두 번.
    std::vector<double> e(static_cast<size_t>(n)), mag(static_cast<size_t>(n));
    for (int round = 0; round < 2; ++round) {
        residuals(p, x1, x2, e.data());
        for (int i = 0; i < n; ++i) mag[i] = std::fabs(e[i]);
        std::vector<double> sorted = mag;
        std::nth_element(sorted.begin(), sorted.begin() + n / 2, sorted.end());
        const double scale = std::max(1.4826 * sorted[static_cast<size_t>(n / 2)], kMinScalePx / k.fx);
        use.assign(static_cast<size_t>(n), 0);
        int kept = 0;
        for (int i = 0; i < n; ++i) {
            use[i] = mag[i] < 3.0 * scale;
            kept += use[i];
        }
        if (kept < kMinInliers) break;
        if (!gauss_newton(p, x1, x2, use, scale, kIterations)) return out;
    }
    residuals(p, x1, x2, e.data());
    int inliers = 0;
    for (int i = 0; i < n; ++i) inliers += std::fabs(e[i]) < 2.0 * sigma ? 1 : 0;
    out.inliers = inliers;
    out.rotation_rad = std::sqrt(p[2] * p[2] + p[3] * p[3] + p[4] * p[4]);
    out.yaw_rad = std::atan(p[0]);
    out.pitch_rad = -std::atan(p[1]);
    out.valid = std::isfinite(out.yaw_rad) && std::isfinite(out.pitch_rad) && inliers >= kMinInliers &&
                out.rotation_rad < kMaxRotation && std::fabs(out.yaw_rad) < kMaxAngle &&
                std::fabs(out.pitch_rad) < kMaxAngle;
    return out;
}

int pick_flow_corners(const int16_t *response, int width, int height, int scale, int row_offset, double quality,
                      int max_points, float *xy) {
    constexpr int kCell = 8, kBorder = 3, kMinDist2 = 25;
    if (width < kCell || height < kCell || max_points <= 0) return 0;
    const int gw = width / kCell, gh = height / kCell;
    std::vector<int> best_v(static_cast<size_t>(gw) * gh, -1), best_i(static_cast<size_t>(gw) * gh, -1);
    int vmax = 0;
    for (int y = kBorder; y < gh * kCell && y < height - kBorder; ++y) {
        const int16_t *row = response + static_cast<size_t>(y) * width;
        int *bv = &best_v[static_cast<size_t>(y / kCell) * gw];
        int *bi = &best_i[static_cast<size_t>(y / kCell) * gw];
        for (int x = kBorder; x < gw * kCell && x < width - kBorder; ++x) {
            const int v = row[x];
            if (v > bv[x / kCell]) {
                bv[x / kCell] = v;
                bi[x / kCell] = y * width + x;
            }
            vmax = std::max(vmax, v);
        }
    }
    const int thr = std::max(1, static_cast<int>(quality * vmax));
    std::vector<int> cells;
    for (size_t c = 0; c < best_v.size(); ++c)
        if (best_v[c] >= thr) cells.push_back(static_cast<int>(c));
    std::sort(cells.begin(), cells.end(), [&](int a, int b) { return best_v[a] > best_v[b]; });
    std::vector<int> kept(best_v.size(), -1);  // 칸 -> 고른 화소
    int n = 0;
    for (int c : cells) {
        const int px = best_i[c] % width, py = best_i[c] / width, cx = c % gw, cy = c / gw;
        bool far = true;
        for (int dy = -1; dy <= 1 && far; ++dy)
            for (int dx = -1; dx <= 1 && far; ++dx) {
                const int nx = cx + dx, ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= gw || ny >= gh) continue;
                const int q = kept[static_cast<size_t>(ny) * gw + nx];
                if (q < 0) continue;
                const int ddx = q % width - px, ddy = q / width - py;
                far = ddx * ddx + ddy * ddy >= kMinDist2;
            }
        if (!far) continue;
        kept[c] = best_i[c];
        xy[2 * n] = (static_cast<float>(px) + 0.5f) * static_cast<float>(scale) - 0.5f;
        xy[2 * n + 1] = (static_cast<float>(py + row_offset) + 0.5f) * static_cast<float>(scale) - 0.5f;
        if (++n >= max_points) break;
    }
    return n;
}

bool flow_gate_open(const FlowGateInput &in) {
    return in.vehicle_fresh && !in.blinker && !in.desire && std::isfinite(in.v_ego) && std::isfinite(in.curvature) &&
           in.v_ego >= kMinSpeed && std::fabs(in.curvature * in.v_ego) <= kMaxYawRate;
}

void FlowCalibrator::reset() {
    block_yaw_.clear();
    block_pitch_.clear();
    med_yaw_.clear();
    med_pitch_.clear();
    blocks_since_restore_ = -1;
    state_ = FlowCalibrationState{};
    state_.status = static_cast<uint32_t>(FlowCalibrationStatus::Collecting);
}

bool FlowCalibrator::restore(const FlowBlocks &blocks) {
    if (blocks.yaw.size() != blocks.pitch.size()) return false;
    for (size_t i = 0; i < blocks.yaw.size(); ++i)
        if (!std::isfinite(blocks.yaw[i]) || !std::isfinite(blocks.pitch[i]) || std::fabs(blocks.yaw[i]) >= kMaxAngle ||
            std::fabs(blocks.pitch[i]) >= kMaxAngle)
            return false;
    reset();
    const size_t skip = blocks.yaw.size() > static_cast<size_t>(kFlowMaxBlocks)
                            ? blocks.yaw.size() - static_cast<size_t>(kFlowMaxBlocks)
                            : 0;
    med_yaw_.assign(blocks.yaw.begin() + static_cast<std::ptrdiff_t>(skip), blocks.yaw.end());
    med_pitch_.assign(blocks.pitch.begin() + static_cast<std::ptrdiff_t>(skip), blocks.pitch.end());
    state_.windows = blocks.windows;
    state_.blocks = static_cast<uint32_t>(med_yaw_.size());
    state_.mount_changes = blocks.mount_changes;
    if (!med_yaw_.empty()) blocks_since_restore_ = 0;
    update_estimate();
    return true;
}

FlowBlocks FlowCalibrator::blocks() const {
    FlowBlocks out;
    out.yaw.assign(med_yaw_.begin(), med_yaw_.end());
    out.pitch.assign(med_pitch_.begin(), med_pitch_.end());
    out.windows = state_.windows;
    out.mount_changes = state_.mount_changes;
    return out;
}

void FlowCalibrator::add(const FlowSolution &window, int tracks, uint64_t end_frame_id) {
    state_.window_frame_id = end_frame_id;
    state_.window_yaw = static_cast<float>(window.yaw_rad);
    state_.window_pitch = static_cast<float>(window.pitch_rad);
    state_.window_rotation = static_cast<float>(window.rotation_rad);
    state_.window_parallax = static_cast<float>(window.parallax_px);
    state_.window_tracks = static_cast<uint32_t>(tracks);
    state_.window_inliers = static_cast<uint32_t>(window.inliers);
    ++state_.windows;
    block_yaw_.push_back(state_.window_yaw);
    block_pitch_.push_back(state_.window_pitch);
    if (static_cast<int>(block_yaw_.size()) < kFlowBlockWindows) return;

    med_yaw_.push_back(median_of(block_yaw_));
    med_pitch_.push_back(median_of(block_pitch_));
    block_yaw_.clear();
    block_pitch_.clear();
    while (static_cast<int>(med_yaw_.size()) > kFlowMaxBlocks) {
        med_yaw_.pop_front();
        med_pitch_.pop_front();
    }
    ++state_.blocks;
    if (blocks_since_restore_ >= 0) ++blocks_since_restore_;
    detect_mount_change();
    update_estimate();
}

void FlowCalibrator::detect_mount_change() {
    const int n = static_cast<int>(med_yaw_.size()), recent = kFlowMountCheckBlocks;
    if (n < kFlowMinBlocks + recent) return;
    double old_yaw = 0.0, old_pitch = 0.0;
    for (int i = 0; i < n - recent; ++i) {
        old_yaw += med_yaw_[static_cast<size_t>(i)];
        old_pitch += med_pitch_[static_cast<size_t>(i)];
    }
    old_yaw /= n - recent;
    old_pitch /= n - recent;
    // 복원한 뒤 처음 새 블록 셋이 차면 복원한 블록과 촘촘한 기준으로 한 번 견준다
    const bool after_boot = blocks_since_restore_ == recent;
    const double shift = (after_boot ? kFlowBootShiftDeg : kFlowMountShiftDeg) * kPi / 180.0;
    const double agree = (after_boot ? kFlowBootAgreeDeg : kFlowMountAgreeDeg) * kPi / 180.0;
    // 같은 쪽으로 shift보다 멀고 서로 agree 안이어야 한다(한 축이면 충분하다)
    auto moved = [&](const std::deque<float> &med, double old) {
        double lo = 1e9, hi = -1e9;
        for (int i = n - recent; i < n; ++i) {
            const double d = med[static_cast<size_t>(i)] - old;
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        return hi - lo <= agree && (lo > shift || hi < -shift);
    };
    if (!moved(med_yaw_, old_yaw) && !moved(med_pitch_, old_pitch)) return;
    med_yaw_.erase(med_yaw_.begin(), med_yaw_.end() - recent);
    med_pitch_.erase(med_pitch_.begin(), med_pitch_.end() - recent);
    ++state_.mount_changes;
}

void FlowCalibrator::update_estimate() {
    state_.estimate_blocks = static_cast<uint32_t>(med_yaw_.size());
    if (med_yaw_.empty()) {
        state_.status = static_cast<uint32_t>(FlowCalibrationStatus::Collecting);
        return;
    }
    double sy = 0.0, sp = 0.0;
    for (size_t i = 0; i < med_yaw_.size(); ++i) {
        sy += med_yaw_[i];
        sp += med_pitch_[i];
    }
    const double nb = static_cast<double>(med_yaw_.size());
    state_.yaw = static_cast<float>(sy / nb);
    state_.pitch = static_cast<float>(sp / nb);
    // 블록 중앙값의 표본 표준편차. 범위는 블록 하나만 튀어도 30블록(4분) 동안 커진다.
    double vy = 0.0, vp = 0.0;
    for (size_t i = 0; i < med_yaw_.size(); ++i) {
        vy += (med_yaw_[i] - state_.yaw) * (med_yaw_[i] - state_.yaw);
        vp += (med_pitch_[i] - state_.pitch) * (med_pitch_[i] - state_.pitch);
    }
    state_.sd_yaw = nb > 1.0 ? static_cast<float>(std::sqrt(vy / (nb - 1.0))) : 0.0f;
    state_.sd_pitch = nb > 1.0 ? static_cast<float>(std::sqrt(vp / (nb - 1.0))) : 0.0f;
    state_.status =
        static_cast<uint32_t>(static_cast<int>(med_yaw_.size()) >= kFlowMinBlocks ? FlowCalibrationStatus::Estimate
                                                                                  : FlowCalibrationStatus::Collecting);
}
