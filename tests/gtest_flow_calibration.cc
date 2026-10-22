// 광류 보정기의 하드웨어 무관 부분: 이동 방향 풀이(알려진 장면을 투영한 합성 점), 코너 고르기, 곧은 길
// 조건, 블록 집계, 그리고 가짜 엔진으로 FlowTracker의 프레임 잇기와 버림. IVE 부분은 보드 리허설에서
// 확인한다(docs/rehearsal.md).
#include "model/flow_calibration.h"
#include "model/flow_tracker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
const FlowIntrinsics kK{754.16, 753.9, 626.7533, 368.4};  // MaixCAM2 1280x720

void rotation(const double w[3], double r[9]) {
    const double th = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    const double kx = w[0] / th, ky = w[1] / th, kz = w[2] / th, s = std::sin(th), c = 1.0 - std::cos(th);
    const double m[9] = {1 + c * (-ky * ky - kz * kz), -s * kz + c * kx * ky,        s * ky + c * kx * kz,
                         s * kz + c * kx * ky,         1 + c * (-kx * kx - kz * kz), -s * kx + c * ky * kz,
                         -s * ky + c * kx * kz,        s * kx + c * ky * kz,         1 + c * (-kx * kx - ky * ky)};
    for (int i = 0; i < 9; ++i) r[i] = m[i];
}

// 카메라가 yaw·pitch 방향으로 dist m 가고 w만큼 도는 동안 앞의 정지한 점들이 화면에서 움직인 자리.
// 점은 카메라 좌표(x 오른쪽, y 아래, z 앞)로 뿌린다. 끝 점에 noise_px 잡음, outlier_frac만큼 엉뚱한 점.
void synth_tracks(double yaw, double pitch, double dist, const double w[3], double noise_px, double outlier_frac,
                  int want, unsigned seed, std::vector<float> *a, std::vector<float> *b) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(-15.0, 15.0), uy(-1.0, 3.0), uz(8.0, 80.0), u01(0.0, 1.0);
    std::normal_distribution<double> noise(0.0, noise_px);
    const double m[3] = {std::tan(yaw), -std::tan(pitch), 1.0};
    const double norm = std::sqrt(m[0] * m[0] + m[1] * m[1] + 1.0);
    const double c[3] = {dist * m[0] / norm, dist * m[1] / norm, dist / norm};
    double r[9];
    rotation(w, r);
    a->clear();
    b->clear();
    while (static_cast<int>(a->size()) < 2 * want) {
        const double p[3] = {ux(rng), uy(rng), uz(rng)};
        const double q0[3] = {p[0] - c[0], p[1] - c[1], p[2] - c[2]};
        const double q[3] = {r[0] * q0[0] + r[1] * q0[1] + r[2] * q0[2], r[3] * q0[0] + r[4] * q0[1] + r[5] * q0[2],
                             r[6] * q0[0] + r[7] * q0[1] + r[8] * q0[2]};
        if (q[2] < 1.0) continue;
        const double u1 = kK.fx * p[0] / p[2] + kK.cx, v1 = kK.fy * p[1] / p[2] + kK.cy;
        double u2 = kK.fx * q[0] / q[2] + kK.cx, v2 = kK.fy * q[1] / q[2] + kK.cy;
        if (u1 < 0 || u1 > 1280 || v1 < 0 || v1 > 720 || u2 < 0 || u2 > 1280 || v2 < 0 || v2 > 720) continue;
        if (u01(rng) < outlier_frac) {
            u2 += 30.0 * (u01(rng) - 0.5);
            v2 += 30.0 * (u01(rng) - 0.5);
        } else {
            u2 += noise(rng);
            v2 += noise(rng);
        }
        a->push_back(static_cast<float>(u1));
        a->push_back(static_cast<float>(v1));
        b->push_back(static_cast<float>(u2));
        b->push_back(static_cast<float>(v2));
    }
}

TEST(FlowSolve, RecoversDirectionAndRotation) {
    const double w[3] = {0.0004, -0.0012, 0.0003};
    std::vector<float> a, b;
    synth_tracks(-0.6 * kDeg, -1.0 * kDeg, 5.0, w, 0.05, 0.1, 300, 7, &a, &b);
    const FlowSolution s = solve_flow_direction(kK, a.data(), b.data(), 300);
    ASSERT_TRUE(s.valid);
    EXPECT_NEAR(s.yaw_rad / kDeg, -0.6, 0.02);
    EXPECT_NEAR(s.pitch_rad / kDeg, -1.0, 0.02);
    EXPECT_NEAR(s.rotation_rad, std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]), 1e-4);
    EXPECT_GE(s.inliers, 250);  // 엉뚱한 30개 정도는 맞지 않는다
    EXPECT_LE(s.inliers, 285);
}

TEST(FlowSolve, SignsFollowTheCalibrationConvention) {
    // 오른쪽·위로 가면 yaw·pitch가 양수(calibration rpy와 같은 규약, 10-10 데이터에서 보정기와 맞춘 방향)
    const double w[3] = {0.0, 0.0, 1e-9};
    std::vector<float> a, b;
    synth_tracks(0.8 * kDeg, 0.5 * kDeg, 6.0, w, 0.0, 0.0, 200, 3, &a, &b);
    const FlowSolution s = solve_flow_direction(kK, a.data(), b.data(), 200);
    ASSERT_TRUE(s.valid);
    EXPECT_NEAR(s.yaw_rad / kDeg, 0.8, 1e-3);
    EXPECT_NEAR(s.pitch_rad / kDeg, 0.5, 1e-3);
}

TEST(FlowSolve, StaticSceneIsRejectedByParallax) {
    // 서 있는 장면: 끝 점 = 시작 점 + 0.05 px 잡음. 어떤 방향이든 식을 맞춰 무작위 방향이 통과했다(데스크 테스트).
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> ux(20.0, 1260.0), uy(100.0, 600.0);
    std::normal_distribution<double> noise(0.0, 0.05);
    std::vector<float> a, b;
    for (int i = 0; i < 300; ++i) {
        const double x = ux(rng), y = uy(rng);
        a.push_back(static_cast<float>(x));
        a.push_back(static_cast<float>(y));
        b.push_back(static_cast<float>(x + noise(rng)));
        b.push_back(static_cast<float>(y + noise(rng)));
    }
    const FlowSolution s = solve_flow_direction(kK, a.data(), b.data(), 300);
    EXPECT_FALSE(s.valid);
    EXPECT_LT(s.parallax_px, 0.2);
    EXPECT_EQ(s.points, 300);
}

TEST(FlowSolve, ReportsParallax) {
    const double w[3] = {0.0002, 0.0001, 0.0001};
    std::vector<float> a, b;
    synth_tracks(-0.3 * kDeg, -0.5 * kDeg, 4.4, w, 0.05, 0.0, 300, 11, &a, &b);  // 40 km/h, 0.4초
    const FlowSolution s = solve_flow_direction(kK, a.data(), b.data(), 300);
    ASSERT_TRUE(s.valid);
    EXPECT_GT(s.parallax_px, kFlowMinParallaxPx);
    EXPECT_NEAR(s.yaw_rad / kDeg, -0.3, 0.05);
}

TEST(FlowSolve, RejectsTooFewPointsAndLargeRotation) {
    const double small[3] = {0.0, 0.0002, 0.0};
    std::vector<float> a, b;
    synth_tracks(0.0, 0.0, 5.0, small, 0.05, 0.0, 60, 11, &a, &b);
    EXPECT_FALSE(solve_flow_direction(kK, a.data(), b.data(), 49).valid);
    EXPECT_TRUE(solve_flow_direction(kK, a.data(), b.data(), 60).valid);
    const double turn[3] = {0.0, 0.02, 0.0};  // 1.1°: 곧은 길이 아니다
    synth_tracks(0.0, 0.0, 5.0, turn, 0.05, 0.0, 200, 13, &a, &b);
    EXPECT_FALSE(solve_flow_direction(kK, a.data(), b.data(), 200).valid);
}

TEST(FlowCorners, StrongestPerCellSpacedAndScaled) {
    constexpr int w = 64, h = 32;  // 8 x 4 칸
    std::vector<int16_t> resp(w * h, 1);
    auto at = [&](int x, int y, int v) { resp[static_cast<size_t>(y) * w + x] = static_cast<int16_t>(v); };
    at(10, 10, 1000);  // 칸 (1,1)
    at(12, 9, 900);    // 같은 칸: 약하다
    at(17, 10, 800);   // 이웃 칸 (2,1)이지만 (10,10)과 7화소 떨어져 있다
    at(20, 20, 50);    // 문턱(최댓값의 10%) 아래
    at(41, 13, 700);   // 칸 (5,1)
    at(42, 16, 600);   // 이웃 칸 (5,2), (41,13)과 3.2화소: 건너뛴다
    float xy[16];
    const int n = pick_flow_corners(resp.data(), w, h, 2, 10, 0.1, 8, xy);
    ASSERT_EQ(n, 3);
    // 강한 순, 원본 좌표 = (x + 0.5)·2 - 0.5, (y + 10 + 0.5)·2 - 0.5
    EXPECT_FLOAT_EQ(xy[0], 20.5f);
    EXPECT_FLOAT_EQ(xy[1], 40.5f);
    EXPECT_FLOAT_EQ(xy[2], 34.5f);
    EXPECT_FLOAT_EQ(xy[3], 40.5f);
    EXPECT_FLOAT_EQ(xy[4], 82.5f);
    EXPECT_FLOAT_EQ(xy[5], 46.5f);
    EXPECT_EQ(pick_flow_corners(resp.data(), w, h, 2, 10, 0.1, 2, xy), 2);
}

TEST(FlowGate, OnlyStraightAndFast) {
    FlowGateInput in;
    in.v_ego = 100.0f / 3.6f;
    in.curvature = 1.0e-4f;
    in.vehicle_fresh = true;
    EXPECT_TRUE(flow_gate_open(in));
    in.curvature = 2.0e-4f;  // 100 km/h에서 0.32°/s
    EXPECT_FALSE(flow_gate_open(in));
    in.v_ego = 50.0f / 3.6f;  // 50 km/h에서는 0.16°/s
    EXPECT_TRUE(flow_gate_open(in));
    in.v_ego = 35.0f / 3.6f;
    EXPECT_FALSE(flow_gate_open(in));
    in.v_ego = 60.0f / 3.6f;
    in.blinker = true;
    EXPECT_FALSE(flow_gate_open(in));
    in.blinker = false;
    in.desire = true;
    EXPECT_FALSE(flow_gate_open(in));
    in.desire = false;
    in.vehicle_fresh = false;
    EXPECT_FALSE(flow_gate_open(in));
}

FlowSolution window(double yaw_deg, double pitch_deg) {
    FlowSolution s;
    s.valid = true;
    s.yaw_rad = yaw_deg * kDeg;
    s.pitch_rad = pitch_deg * kDeg;
    s.rotation_rad = 0.001;
    s.inliers = 100;
    return s;
}

TEST(FlowCalibratorTest, BlockMediansAndStatus) {
    FlowCalibrator cal;
    EXPECT_EQ(cal.state().status, 1U);
    uint64_t frame = 100;
    // 블록마다 중앙값이 -0.1, -0.2, -0.3°가 되게, 한 개씩 크게 튄 창을 섞는다
    for (int blk = 0; blk < kFlowMinBlocks; ++blk) {
        for (int i = 0; i < kFlowBlockWindows; ++i) {
            const double yaw = (i == 3) ? 5.0 : -0.1 * (blk + 1);
            cal.add(window(yaw, -1.0), 120, frame += 9);
        }
        EXPECT_EQ(cal.state().blocks, static_cast<uint32_t>(blk + 1));
        EXPECT_EQ(cal.state().status, blk + 1 >= kFlowMinBlocks ? 2U : 1U);
    }
    const FlowCalibrationState st = cal.state();
    EXPECT_EQ(st.windows, static_cast<uint32_t>(kFlowMinBlocks * kFlowBlockWindows));
    EXPECT_NEAR(st.yaw / kDeg, -0.2, 1e-5);
    EXPECT_NEAR(st.pitch / kDeg, -1.0, 1e-5);
    EXPECT_NEAR(st.sd_yaw / kDeg, 0.1, 1e-5);  // -0.1, -0.2, -0.3의 표본 표준편차
    EXPECT_EQ(st.window_frame_id, frame);
    EXPECT_NEAR(st.window_yaw / kDeg, -0.3, 1e-5);
    EXPECT_EQ(st.window_tracks, 120U);
    EXPECT_EQ(st.window_inliers, 100U);
}

TEST(FlowCalibratorTest, KeepsTheLatestBlocks) {
    FlowCalibrator cal;
    for (int blk = 0; blk < kFlowMaxBlocks + 5; ++blk)
        for (int i = 0; i < kFlowBlockWindows; ++i) cal.add(window(blk < 5 ? 3.0 : -0.5, 0.0), 100, 1);
    // 처음 다섯 블록(3°)은 밀려났다
    EXPECT_EQ(cal.state().blocks, static_cast<uint32_t>(kFlowMaxBlocks + 5));
    EXPECT_NEAR(cal.state().yaw / kDeg, -0.5, 1e-5);
    EXPECT_NEAR(cal.state().sd_yaw, 0.0, 1e-7);
    cal.reset();
    EXPECT_EQ(cal.state().windows, 0U);
    EXPECT_EQ(cal.state().status, 1U);
}

void add_block(FlowCalibrator *cal, double yaw_deg, double pitch_deg) {
    for (int i = 0; i < kFlowBlockWindows; ++i) cal->add(window(yaw_deg, pitch_deg), 100, 1);
}

TEST(FlowCalibratorTest, RestoreRoundTrip) {
    FlowCalibrator cal;
    for (int blk = 0; blk < 5; ++blk) add_block(&cal, -0.1 * (blk + 1), -1.0 + 0.05 * blk);
    const FlowBlocks saved = cal.blocks();
    ASSERT_EQ(saved.yaw.size(), 5U);
    EXPECT_NEAR(saved.yaw.front() / kDeg, -0.1, 1e-5) << "오래된 것부터";
    EXPECT_EQ(saved.windows, static_cast<uint32_t>(5 * kFlowBlockWindows));

    FlowCalibrator restored;
    ASSERT_TRUE(restored.restore(saved));
    const FlowCalibrationState a = cal.state(), b = restored.state();
    EXPECT_EQ(b.status, 2U);
    EXPECT_EQ(b.estimate_blocks, 5U);
    EXPECT_EQ(b.blocks, 5U);
    EXPECT_EQ(b.windows, a.windows);
    EXPECT_FLOAT_EQ(b.yaw, a.yaw);
    EXPECT_FLOAT_EQ(b.pitch, a.pitch);
    EXPECT_FLOAT_EQ(b.sd_yaw, a.sd_yaw);

    FlowBlocks many;
    for (int i = 0; i < kFlowMaxBlocks + 10; ++i) {
        many.yaw.push_back(static_cast<float>((i < 10 ? 2.0 : -0.4) * kDeg));
        many.pitch.push_back(static_cast<float>(-1.0 * kDeg));
    }
    ASSERT_TRUE(restored.restore(many));
    EXPECT_EQ(restored.state().estimate_blocks, static_cast<uint32_t>(kFlowMaxBlocks)) << "최근 30블록만";
    EXPECT_NEAR(restored.state().yaw / kDeg, -0.4, 1e-5);

    FlowBlocks bad = saved;
    bad.pitch.pop_back();
    EXPECT_FALSE(restored.restore(bad)) << "길이가 다르다";
    bad = saved;
    bad.yaw[2] = std::nanf("");
    EXPECT_FALSE(restored.restore(bad));
    bad = saved;
    bad.yaw[0] = static_cast<float>(12.0 * kDeg);
    EXPECT_FALSE(restored.restore(bad)) << "풀이 한계(10°) 밖";
    EXPECT_NEAR(restored.state().yaw / kDeg, -0.4, 1e-5) << "실패하면 그대로다";
}

TEST(FlowCalibratorTest, MountChangeKeepsTheRecentBlocks) {
    FlowCalibrator cal;
    for (int blk = 0; blk < 10; ++blk) add_block(&cal, -0.2, -1.0);
    add_block(&cal, 1.0, -1.0);
    add_block(&cal, 1.1, -1.0);
    EXPECT_EQ(cal.state().mount_changes, 0U) << "두 블록으로는 아직";
    EXPECT_EQ(cal.state().estimate_blocks, 12U);
    add_block(&cal, 0.9, -1.0);
    EXPECT_EQ(cal.state().mount_changes, 1U);
    EXPECT_EQ(cal.state().estimate_blocks, 3U);
    EXPECT_NEAR(cal.state().yaw / kDeg, 1.0, 1e-5);
    EXPECT_EQ(cal.state().status, 2U);
    EXPECT_EQ(cal.state().blocks, 13U) << "blocks는 누적이다";
    EXPECT_EQ(cal.blocks().mount_changes, 1U) << "저장 내용에도 남는다";

    FlowCalibrator pitch;
    for (int blk = 0; blk < 6; ++blk) add_block(&pitch, 0.0, -1.0);
    for (int blk = 0; blk < 3; ++blk) add_block(&pitch, 0.0, 0.3 + 0.1 * blk);
    EXPECT_EQ(pitch.state().mount_changes, 1U) << "pitch만 움직여도";

    FlowCalibrator small;
    for (int blk = 0; blk < 6; ++blk) add_block(&small, 0.0, -1.0);
    for (int blk = 0; blk < 3; ++blk) add_block(&small, 0.8, -1.0);
    EXPECT_EQ(small.state().mount_changes, 0U) << "1°보다 작은 이동은 평균이 천천히 따라간다";
    EXPECT_EQ(small.state().estimate_blocks, 9U);

    FlowCalibrator scattered;
    for (int blk = 0; blk < 6; ++blk) add_block(&scattered, 0.0, -1.0);
    add_block(&scattered, 1.5, -1.0);
    add_block(&scattered, 2.5, -1.0);
    add_block(&scattered, 1.2, -1.0);
    EXPECT_EQ(scattered.state().mount_changes, 0U) << "새 블록끼리 0.5° 넘게 흩어지면 장착 변경으로 보지 않는다";
}

TEST(FlowCalibratorTest, CountsRejectedWindows) {
    FlowCalibrator cal;
    FlowSolution bad;
    bad.parallax_px = 0.3;
    cal.note_rejected(bad);
    cal.note_rejected(bad);
    EXPECT_EQ(cal.state().rejected_windows, 2U);
    EXPECT_NEAR(cal.state().window_parallax, 0.3, 1e-6);
    cal.reset();
    EXPECT_EQ(cal.state().rejected_windows, 0U);
}

// 가짜 광류 엔진: 띠 안 격자에 코너 봉우리를 두고, 점은 주점 둘레로 한 프레임에 1%씩 퍼진다(정면으로 곧게 가는
// 카메라). 풀이는 이동 방향 yaw 0·pitch 0을 낸다. push·track을 실패시키거나 센다.
class FakeFlowEngine : public FlowEngine {
public:
    explicit FakeFlowEngine(int width) : width_(width), response_(static_cast<size_t>(width / 2) * kBandRows, 0) {
        for (int y = 8; y < kBandRows - 8; y += 12)
            for (int x = 8; x < width / 2 - 8; x += 12) response_[static_cast<size_t>(y) * (width / 2) + x] = 1000;
    }
    bool push(unsigned long long) override {
        ++pushes;
        return push_ok;
    }
    const int16_t *corner_response() override { return response_.data(); }
    bool track(const float *xy, int n, float *out_xy, uint8_t *ok, bool backward) override {
        ++tracks;
        const double scale = backward ? 1.0 / 1.01 : 1.01;
        for (int i = 0; i < n; ++i) {
            out_xy[2 * i] = static_cast<float>(kK.cx + (xy[2 * i] - kK.cx) * scale);
            out_xy[2 * i + 1] = static_cast<float>(kK.cy + (xy[2 * i + 1] - kK.cy) * scale);
            ok[i] = 1;
        }
        return true;
    }
    int width() const override { return width_; }

    bool push_ok = true;
    int pushes = 0;
    int tracks = 0;

private:
    int width_;
    std::vector<int16_t> response_;
};

struct TrackerRig {
    FakeFlowEngine *engine = nullptr;
    std::unique_ptr<FlowTracker> tracker;
    TrackerRig() {
        auto fake = std::make_unique<FakeFlowEngine>(1280);
        engine = fake.get();
        tracker = std::make_unique<FlowTracker>(kK, 1280, 720, std::move(fake));
    }
    // 곧고 빠른 프레임 하나. valid가 거짓이면 링 슬롯이 그동안 덮어써진 것이다.
    void frame(uint64_t id, bool valid = true, float v_ego = 25.0f) {
        FlowFrame f;
        f.y_phys = 0x1000;
        f.stride = 1280;
        f.frame_id = id;
        f.t_s = 0.05 * static_cast<double>(id);
        f.gate.v_ego = v_ego;
        f.gate.vehicle_fresh = true;
        f.still_valid = [valid] { return valid; };
        tracker->update(f);
    }
};

// 이어진 프레임으로 창 하나(8걸음)를 끝내고 이동 방향을 낸다
TEST(FlowTrackerTest, ConsecutiveFramesFinishAWindow) {
    TrackerRig rig;
    for (uint64_t id = 1; id <= 1 + kFlowWindowSteps; ++id) rig.frame(id);
    const FlowTracker::Stats &s = rig.tracker->stats();
    EXPECT_EQ(s.windows_done, 1u);
    EXPECT_EQ(s.windows_lost, 0u);
    EXPECT_EQ(s.frames, static_cast<uint64_t>(1 + kFlowWindowSteps));
    EXPECT_EQ(rig.engine->tracks, kFlowWindowSteps) << "앞차가 없으면 걸음마다 한 번만 따라간다";
    const FlowCalibrationState st = rig.tracker->calibrator().state();
    EXPECT_EQ(st.windows, 1u);
    // 풀이의 정밀도는 FlowSolve가 본다. 여기서는 띠 안 점만으로도 정면(0.3° 안)이 나오는지만 본다
    EXPECT_NEAR(st.window_yaw, 0.0, 0.005);
    EXPECT_NEAR(st.window_pitch, 0.0, 0.005);
}

// frame_id가 이어지지 않으면 진행 중인 창을 버리고 그 프레임에서 새로 시작한다(한 걸음 이동이 한계를 넘을 수
// 있다). 녹화를 재생하는 도구도 이 규칙을 따르도록 녹화된 frame_id를 준다.
TEST(FlowTrackerTest, AFrameGapDropsTheWindow) {
    TrackerRig rig;
    for (uint64_t id = 1; id <= 4; ++id) rig.frame(id);
    rig.frame(6);
    EXPECT_EQ(rig.tracker->stats().windows_lost, 1u);
    for (uint64_t id = 7; id < 6 + kFlowWindowSteps; ++id) rig.frame(id);
    EXPECT_EQ(rig.tracker->stats().windows_done, 0u) << "새 창은 6번에서 시작했다";
    rig.frame(6 + kFlowWindowSteps);
    EXPECT_EQ(rig.tracker->stats().windows_done, 1u);
}

// 링 슬롯이 덮어써진 프레임과 엔진이 실패한 프레임은 그 걸음을 버리고, 다음 프레임은 이어지지 않은 것으로 본다
TEST(FlowTrackerTest, OverwrittenOrFailedFramesAreDropped) {
    TrackerRig rig;
    for (uint64_t id = 1; id <= 3; ++id) rig.frame(id);
    rig.frame(4, false);
    EXPECT_EQ(rig.tracker->stats().windows_lost, 1u);
    for (uint64_t id = 5; id <= 7; ++id) rig.frame(id);
    rig.engine->push_ok = false;
    rig.frame(8);
    EXPECT_EQ(rig.tracker->stats().windows_lost, 2u);
    rig.engine->push_ok = true;
    for (uint64_t id = 9; id <= 9 + kFlowWindowSteps; ++id) rig.frame(id);
    EXPECT_EQ(rig.tracker->stats().windows_done, 1u);
    EXPECT_EQ(rig.tracker->stats().frames, 3u + 3u + 1u + kFlowWindowSteps) << "실패한 두 프레임은 세지 않는다";
}

// 곧은 길이 아니고 앞차도 없으면 엔진을 부르지 않고, 열린 창은 버린다
TEST(FlowTrackerTest, ClosedGateLeavesTheEngineIdle) {
    TrackerRig rig;
    rig.frame(1, true, 0.0f);
    rig.frame(2, true, 0.0f);
    EXPECT_EQ(rig.engine->pushes, 0);
    for (uint64_t id = 3; id <= 5; ++id) rig.frame(id);
    rig.frame(6, true, 0.0f);
    EXPECT_EQ(rig.tracker->stats().windows_lost, 1u);
    EXPECT_EQ(rig.engine->pushes, 3);
}

}  // namespace
