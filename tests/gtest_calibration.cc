// 온라인 보정을 openpilot 참조식과 대조한다: OnlineCalibrator ↔ calibrationd.py, calibration_service의 저장·복원·
// 초기화와 광류 출처. 입력 워프의 투영 행렬은 gtest_model_input_transform이 본다.
#include "common/app_config.h"
#include "common/utils_math.h"
#include "model/calibration_online.h"
#include "model/calibration_service.h"
#include "model/flow_calibration.h"

#include <gtest/gtest.h>

#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846264338327950288;
// openpilot calibrationd.py 상수
constexpr double kMinSpeedFilter = 15.0 * 0.44704;
constexpr double kMaxVelAngleStd = 0.25 * kPi / 180.0;
constexpr double kMaxYawRateFilter = 2.0 * kPi / 180.0;
constexpr double kMaxAllowedYawSpread = 2.0 * kPi / 180.0;
constexpr double kMaxAllowedPitchSpread = 4.0 * kPi / 180.0;
constexpr double kSmoothCycles = 10.0;
constexpr double kPitchMin = -0.09074112085129739;
constexpr double kPitchMax = 0.17;  // upstream PITCH_LIMITS (mici 아님)
constexpr double kYawMin = -0.06912048084718224;
constexpr double kYawMax = 0.06912048084718235;
constexpr double kSanityMargin = 0.005;
const double kMaxHeightStd = std::exp(-3.5);
constexpr double kHeightInit = 1.22;
constexpr int kBlockSize = 100;
constexpr int kInputsNeeded = 5;
constexpr int kInputsWanted = 50;

void matmul3d(const double *a, const double *b, double *out) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

// rot = Rz(yaw)·Ry(pitch)·Rx(roll), openpilot rot_from_euler과 같은 순서
void rot_from_euler_ref(const double rpy[3], double *rot) {
    const double cr = std::cos(rpy[0]);
    const double sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]);
    const double sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]);
    const double sy = std::sin(rpy[2]);

    // clang-format off
    const double rx[9] = {
        1.0, 0.0, 0.0,
        0.0, cr, -sr,
        0.0, sr, cr,
    };
    const double ry[9] = {
        cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp,
    };
    const double rz[9] = {
        cy, -sy, 0.0,
        sy, cy, 0.0,
        0.0, 0.0, 1.0,
    };
    // clang-format on

    double tmp[9];
    matmul3d(ry, rx, tmp);
    matmul3d(rz, tmp, rot);
}

void euler_from_rot_ref(const double *rot, double rpy[3]) {
    rpy[0] = std::atan2(rot[2 * 3 + 1], rot[2 * 3 + 2]);
    rpy[1] = std::asin(std::max(-1.0, std::min(1.0, -rot[2 * 3 + 0])));
    rpy[2] = std::atan2(rot[1 * 3 + 0], rot[0 * 3 + 0]);
}

void compose_rpy_ref(const double base[3], const double observed[3], double out[3]) {
    double base_rot[9];
    double observed_rot[9];
    double composed[9];
    rot_from_euler_ref(base, base_rot);
    rot_from_euler_ref(observed, observed_rot);
    matmul3d(base_rot, observed_rot, composed);
    euler_from_rot_ref(composed, out);
}

bool finite3(const double v[3]) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }

// openpilot calibrationd.py의 Calibrator를 double로 옮긴 참조 구현.
struct RefCalibrator {
    double rpys[kInputsWanted][3] = {};
    double heights[kInputsWanted] = {};
    double height = kHeightInit;
    double rpy[3] = {};
    double spread[3] = {};
    double old_rpy[3] = {};
    double old_rpy_weight = 0.0;
    int idx = 0;
    int block_idx = 0;
    int valid_blocks = 0;
    bool seeded = false;  // upstream: 첫 블록으로 워프를 옮겼다(새 보정의 첫 블록은 세지 않는다)
    CalibrationStatus status = CalibrationStatus::Uncalibrated;
    uint64_t accepted = 0;
    uint64_t rejected = 0;

    void reset(const double init[3], int blocks, const double *smooth_from = nullptr,
               double height_init = kHeightInit) {
        valid_blocks = std::max(0, std::min(blocks, kInputsWanted));
        height = std::isfinite(height_init) ? height_init : kHeightInit;
        for (int b = 0; b < kInputsWanted; ++b) heights[b] = height;
        for (int i = 0; i < 3; ++i) rpy[i] = std::isfinite(init[i]) ? init[i] : 0.0;
        for (int b = 0; b < kInputsWanted; ++b) {
            for (int i = 0; i < 3; ++i) rpys[b][i] = rpy[i];
        }
        idx = 0;
        block_idx = 0;
        if (smooth_from) {
            for (int i = 0; i < 3; ++i) old_rpy[i] = smooth_from[i];
            old_rpy_weight = 1.0;
        } else {
            old_rpy[0] = old_rpy[1] = old_rpy[2] = 0.0;
            old_rpy_weight = 0.0;
        }
        update_status();
    }

    void smooth_rpy(double out[3]) const {
        if (old_rpy_weight > 0.0) {
            for (int i = 0; i < 3; ++i) out[i] = old_rpy_weight * old_rpy[i] + (1.0 - old_rpy_weight) * rpy[i];
        } else {
            for (int i = 0; i < 3; ++i) out[i] = rpy[i];
        }
    }

    static bool is_valid(const double rpy_in[3]) {
        return kPitchMin < rpy_in[1] && rpy_in[1] < kPitchMax && kYawMin < rpy_in[2] && rpy_in[2] < kYawMax;
    }

    static void sanity_clip(double rpy_in[3]) {
        if (!finite3(rpy_in)) {
            rpy_in[0] = 0.0;
            rpy_in[1] = 0.0;
            rpy_in[2] = 0.0;
            return;
        }
        rpy_in[1] = std::max(kPitchMin - kSanityMargin, std::min(kPitchMax + kSanityMargin, rpy_in[1]));
        rpy_in[2] = std::max(kYawMin - kSanityMargin, std::min(kYawMax + kSanityMargin, rpy_in[2]));
    }

    void update_status() {
        double sum[3] = {};
        double height_sum = 0.0;
        double min_v[3] = {};
        double max_v[3] = {};
        int valid_count = 0;

        for (int b = 0; b < valid_blocks; ++b) {
            if (b == block_idx) continue;
            if (valid_count == 0) {
                for (int i = 0; i < 3; ++i) min_v[i] = max_v[i] = rpys[b][i];
            }
            height_sum += heights[b];
            for (int i = 0; i < 3; ++i) {
                sum[i] += rpys[b][i];
                min_v[i] = std::min(min_v[i], rpys[b][i]);
                max_v[i] = std::max(max_v[i], rpys[b][i]);
            }
            ++valid_count;
        }

        if (valid_count > 0) {
            height = height_sum / valid_count;
            for (int i = 0; i < 3; ++i) {
                rpy[i] = sum[i] / valid_count;
                spread[i] = std::fabs(max_v[i] - min_v[i]);
            }
        } else {
            spread[0] = spread[1] = spread[2] = 0.0;
        }

        if (valid_blocks < kInputsNeeded) {
            if (status != CalibrationStatus::Recalibrating) status = CalibrationStatus::Uncalibrated;
        } else if (is_valid(rpy)) {
            status = CalibrationStatus::Calibrated;
        } else {
            status = CalibrationStatus::Invalid;
        }

        const bool spread_too_high = spread[1] > kMaxAllowedPitchSpread || spread[2] > kMaxAllowedYawSpread;
        if (status == CalibrationStatus::Calibrated && spread_too_high) {
            const int last_block = (block_idx + kInputsWanted - 1) % kInputsWanted;
            const double smooth_from[3] = {rpy[0], rpy[1], rpy[2]};
            const double last_rpy[3] = {rpys[last_block][0], rpys[last_block][1], rpys[last_block][2]};
            reset(last_rpy, 0, smooth_from);
            status = CalibrationStatus::Recalibrating;
        }
    }

    bool update(const PoseObservation &pose, double v_ego = 20.0) {
        old_rpy_weight = std::max(0.0, old_rpy_weight - 1.0 / kSmoothCycles);

        const double trans[3] = {pose.trans[0], pose.trans[1], pose.trans[2]};
        const double rot[3] = {pose.rot[0], pose.rot[1], pose.rot[2]};
        const double trans_std[3] = {pose.trans_std[0], pose.trans_std[1], pose.trans_std[2]};
        const bool valid_numbers = finite3(trans) && finite3(rot) && finite3(trans_std) && std::isfinite(v_ego);
        const bool straight_and_fast = valid_numbers && v_ego > kMinSpeedFilter && trans[0] > kMinSpeedFilter &&
                                       std::fabs(rot[2]) < kMaxYawRateFilter;
        const bool rpy_certain = std::atan2(trans_std[1], trans[0]) < kMaxVelAngleStd;
        const bool height_certain = pose.road_trans_std[2] < kMaxHeightStd;
        const bool certain_if_calib =
            valid_numbers && ((rpy_certain && height_certain) || (valid_blocks < kInputsNeeded));

        if (!straight_and_fast || !certain_if_calib) {
            ++rejected;
            return false;
        }

        const double observed_rpy[3] = {
            0.0,
            -std::atan2(trans[2], trans[0]),
            std::atan2(trans[1], trans[0]),
        };
        double base[3];
        double new_rpy[3];
        smooth_rpy(base);
        compose_rpy_ref(base, observed_rpy, new_rpy);
        sanity_clip(new_rpy);

        if (!finite3(new_rpy)) {
            ++rejected;
            return false;
        }

        for (int i = 0; i < 3; ++i)
            rpys[block_idx][i] =
                (idx * rpys[block_idx][i] + (kBlockSize - idx) * new_rpy[i]) / static_cast<double>(kBlockSize);
        const double new_height = std::isfinite(pose.road_trans[2]) ? pose.road_trans[2] : kHeightInit;
        heights[block_idx] =
            (idx * heights[block_idx] + (kBlockSize - idx) * new_height) / static_cast<double>(kBlockSize);

        idx = (idx + 1) % kBlockSize;
        ++accepted;
        if (idx == 0 && !seeded) {
            seeded = true;
            for (int i = 0; i < 3; ++i) rpy[i] = rpys[0][i];
        } else if (idx == 0) {
            ++block_idx;
            valid_blocks = std::min(kInputsWanted, std::max(block_idx, valid_blocks));
            block_idx %= kInputsWanted;
        }
        update_status();
        return true;
    }
};

PoseObservation make_pose(float tx = 20.0f, float ty = 0.2f, float tz = -0.4f, float yaw_rate = 0.0f,
                          float trans_std_y = 0.01f) {
    PoseObservation pose{};
    pose.trans[0] = tx;
    pose.trans[1] = ty;
    pose.trans[2] = tz;
    pose.rot[0] = 0.0f;
    pose.rot[1] = 0.0f;
    pose.rot[2] = yaw_rate;
    pose.trans_std[0] = 0.01f;
    pose.trans_std[1] = trans_std_y;
    pose.trans_std[2] = 0.01f;
    pose.rot_std[0] = 0.01f;
    pose.rot_std[1] = 0.01f;
    pose.rot_std[2] = 0.01f;
    pose.road_trans[0] = 0.0f;
    pose.road_trans[1] = 0.0f;
    pose.road_trans[2] = 1.30f;
    pose.road_trans_std[0] = pose.road_trans_std[1] = pose.road_trans_std[2] = 0.01f;
    return pose;
}

void compare_snapshot(const OnlineCalibrator::Snapshot &actual, const RefCalibrator &expected, const char *label) {
    EXPECT_EQ(actual.valid_blocks, expected.valid_blocks) << std::string(label) + " valid_blocks";
    EXPECT_EQ(actual.block_sample_count, expected.idx) << std::string(label) + " block_sample_count";
    EXPECT_EQ(static_cast<int>(actual.status), static_cast<int>(expected.status)) << std::string(label) + " status";
    EXPECT_NEAR(actual.accepted_samples, expected.accepted, 0.0) << std::string(label) + " accepted";
    EXPECT_NEAR(actual.rejected_samples, expected.rejected, 0.0) << std::string(label) + " rejected";
    EXPECT_NEAR(actual.height_m, expected.height, 1e-5) << std::string(label) + " height";
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual.rpy[i], expected.rpy[i], 1e-5) << std::string(label) + " rpy";
        EXPECT_NEAR(actual.spread[i], expected.spread[i], 1e-5) << std::string(label) + " spread";
    }
}

TEST(CalibrationEquivalence, OnlineCalibrator) {
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;

    PoseObservation low_speed = make_pose(6.0f);
    EXPECT_FALSE(actual.update(low_speed, 20.0f).accepted) << "카메라 속도가 낮은 표본은 버린다";

    PoseObservation low_vehicle_speed = make_pose(20.0f, 0.0f, 0.0f, 0.01f, 0.0f);
    EXPECT_FALSE(actual.update(low_vehicle_speed, 2.0f).accepted) << "CAN vEgo가 낮은 표본은 버린다";
    EXPECT_FALSE(ref.update(low_speed)) << "참조식도 저속 표본을 버린다";
    EXPECT_FALSE(ref.update(low_vehicle_speed, 2.0)) << "참조식도 CAN vEgo가 낮은 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "low_speed_reject");

    PoseObservation high_yaw = make_pose();
    high_yaw.rot[2] = static_cast<float>(3.0 * kPi / 180.0);
    EXPECT_FALSE(actual.update(high_yaw, 20.0f).accepted) << "요레이트가 큰 표본은 버린다";
    EXPECT_FALSE(ref.update(high_yaw)) << "참조식도 요레이트가 큰 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "high_yaw_reject");

    PoseObservation nan_pose = make_pose();
    nan_pose.trans[0] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(actual.update(nan_pose, 20.0f).accepted) << "NaN 표본은 버린다";
    EXPECT_FALSE(ref.update(nan_pose)) << "참조식도 NaN 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "nan_reject");

    const PoseObservation accepted_pose = make_pose();
    for (int i = 0; i < kBlockSize; ++i) {
        EXPECT_TRUE(actual.update(accepted_pose, 20.0f).accepted) << "블록 표본을 받아들인다";
        EXPECT_TRUE(ref.update(accepted_pose)) << "참조식도 블록 표본을 받아들인다";
    }
    compare_snapshot(actual.snapshot(), ref, "one_block");
    EXPECT_EQ(actual.snapshot().valid_blocks, 0) << "새 보정의 첫 블록은 세지 않는다";
    EXPECT_NEAR(actual.snapshot().rpy[2], std::atan2(0.2f, 20.0f), 1e-6) << "첫 블록 값으로 워프만 옮긴다";

    for (int i = 0; i < 5 * kBlockSize; ++i) {
        actual.update(accepted_pose, 20.0f);
        ref.update(accepted_pose);
    }
    compare_snapshot(actual.snapshot(), ref, "five_blocks");
    EXPECT_EQ(static_cast<int>(actual.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "블록 5개가 차면 보정 완료";

    PoseObservation uncertain = make_pose();
    uncertain.trans_std[1] = 1.0f;
    EXPECT_FALSE(actual.update(uncertain, 20.0f).accepted) << "보정 뒤에도 trans 표준편차가 큰 표본은 버린다";
    EXPECT_FALSE(ref.update(uncertain)) << "참조식도 보정 뒤 trans 표준편차가 큰 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "uncertain_after_calib");
}

// upstream처럼 road_transform z로 카메라 높이를 블록 평균하고, 보정 뒤에는 높이 표준편차가
// e^-3.5보다 큰 표본을 버린다. 장착 변경으로 다시 모을 때 높이는 HEIGHT_INIT로 돌아간다.
TEST(CalibrationEquivalence, HeightFollowsUpstream) {
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;
    EXPECT_NEAR(actual.snapshot().height_m, 1.22f, 1e-6) << "시작 높이는 HEIGHT_INIT";
    PoseObservation pose = make_pose(20.0f, 0.0f, 0.0f);
    for (int i = 0; i < 6 * kBlockSize; ++i) {  // 워프만 옮기는 첫 블록 + 5블록
        actual.update(pose, 20.0f);
        ref.update(pose);
    }
    compare_snapshot(actual.snapshot(), ref, "height_five_blocks");
    EXPECT_NEAR(actual.snapshot().height_m, 1.30f, 1e-4) << "블록 평균 높이";

    PoseObservation unsure = pose;
    unsure.road_trans_std[2] = 0.05f;
    EXPECT_FALSE(actual.update(unsure, 20.0f).accepted) << "보정 뒤 높이가 불확실한 표본은 버린다";
    EXPECT_FALSE(ref.update(unsure));
    compare_snapshot(actual.snapshot(), ref, "height_uncertain");

    OnlineCalibrator fresh;
    EXPECT_TRUE(fresh.update(unsure, 20.0f).accepted) << "보정 전에는 높이가 불확실해도 받는다";
}

// upstream reset처럼 범위를 벗어난 저장값도 받아 Invalid로 두고, 저장된 높이로 시작한다.
TEST(CalibrationEquivalence, RestoreOutOfRangeLikeUpstream) {
    OnlineCalibrator calibrator;
    const float out_of_range[3] = {0.0f, 0.0f, 0.10f};  // yaw 5.7도: 한계 0.069 rad 밖
    ASSERT_TRUE(calibrator.restore(out_of_range, 12, nullptr, 1.35f));
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Invalid));
    float out[3] = {};
    calibrator.output_rpy(out);
    EXPECT_NEAR(out[2], 0.10f, 1e-7) << "범위 밖이어도 그 rpy를 쓴다(upstream modeld도 rpyCalib를 그대로 쓴다)";
    EXPECT_NEAR(calibrator.snapshot().height_m, 1.35f, 1e-7) << "저장된 높이";
    const float nan_rpy[3] = {NAN, 0.0f, 0.0f};
    ASSERT_TRUE(calibrator.restore(nan_rpy, 3));
    calibrator.output_rpy(out);
    EXPECT_EQ(out[0], 0.0f) << "유한하지 않으면 RPY_INIT";
    EXPECT_EQ(calibrator.snapshot().valid_blocks, 3);
    const float pitch_016[3] = {0.0f, 0.16f, 0.0f};
    ASSERT_TRUE(calibrator.restore(pitch_016, 12));
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "pitch 상한은 upstream 0.17";
}

TEST(CalibrationEquivalence, MountChangeRecalibratesLikeUpstream) {
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;
    const PoseObservation still = make_pose(20.0f, 0.0f, 0.0f);  // 현재 보정과 맞는 관측
    for (int i = 0; i < 6 * kBlockSize; ++i) {
        actual.update(still, 20.0f);
        ref.update(still);
    }
    ASSERT_EQ(static_cast<int>(actual.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));

    // pitch 3도: openpilot 문턱(4도) 아래라 그대로 보정 완료로 섞인다.
    const float pitch3 = -20.0f * std::tan(static_cast<float>(3.0 * kPi / 180.0));
    const PoseObservation pitched = make_pose(20.0f, 0.0f, pitch3);
    for (int i = 0; i < kBlockSize; ++i) {
        actual.update(pitched, 20.0f);
        ref.update(pitched);
    }
    compare_snapshot(actual.snapshot(), ref, "pitch_3deg");
    EXPECT_EQ(static_cast<int>(actual.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "pitch 편차 3도는 장착 변경으로 보지 않는다";

    // yaw 2도를 넘는 편차는 장착 변경: 마지막 블록 하나에서 다시 모은다.
    OnlineCalibrator yawed_actual;
    RefCalibrator yawed_ref;
    yawed_ref.reset(zero, 0);
    for (int i = 0; i < 6 * kBlockSize; ++i) {
        yawed_actual.update(still, 20.0f);
        yawed_ref.update(still);
    }
    const float yaw3 = 20.0f * std::tan(static_cast<float>(3.0 * kPi / 180.0));
    const PoseObservation yawed = make_pose(20.0f, yaw3, 0.0f);
    bool recalibrating = false;
    for (int i = 0; i < kBlockSize && !recalibrating; ++i) {
        yawed_actual.update(yawed, 20.0f);
        yawed_ref.update(yawed);
        recalibrating = yawed_actual.snapshot().status == CalibrationStatus::Recalibrating;
    }
    compare_snapshot(yawed_actual.snapshot(), yawed_ref, "yaw_3deg");
    ASSERT_TRUE(recalibrating) << "yaw 편차가 2도를 넘으면 recalibrating";
    EXPECT_EQ(yawed_actual.snapshot().valid_blocks, 0) << "openpilot처럼 블록 0개에서 다시 시작";
    EXPECT_GT(yawed_actual.snapshot().rpy[2], deg_to_rad(2.0f)) << "새 장착의 yaw로 옮겨 간다";

    // block_idx가 0부터 다시 세므로 openpilot도 새 블록 5개가 다 차야 보정 완료다.
    for (int i = 0; i < 4 * kBlockSize; ++i) {
        yawed_actual.update(still, 20.0f);
        yawed_ref.update(still);
    }
    compare_snapshot(yawed_actual.snapshot(), yawed_ref, "recalibrating_4_blocks");
    EXPECT_EQ(static_cast<int>(yawed_actual.snapshot().status), static_cast<int>(CalibrationStatus::Recalibrating))
        << "5블록 전에는 recalibrating";
    for (int i = 0; i < kBlockSize; ++i) {
        yawed_actual.update(still, 20.0f);
        yawed_ref.update(still);
    }
    compare_snapshot(yawed_actual.snapshot(), yawed_ref, "recalibrated");
    EXPECT_EQ(static_cast<int>(yawed_actual.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "5블록이 다시 차면 보정 완료";
}

// upstream #39170: 새 보정의 첫 블록은 보정 안 된 워프(0)로 잰 것이라 크게 틀릴 수 있다. 그 블록은 워프만 옮기고
// 세지 않으므로, pitch가 5도 틀린 첫 블록이 뒤 블록들과의 편차 검사(4도)에서 장착 변경으로 오판되지 않는다.
// 첫 블록 뒤의 관측은 그때 워프에 대한 나머지(참값 1도 - 워프)라 보정은 참값으로 모인다.
TEST(CalibrationEquivalence, FirstBlockOnlyMovesTheWarp) {
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;
    const auto pitched = [](float pitch_rad) { return make_pose(20.0f, 0.0f, -20.0f * std::tan(pitch_rad)); };
    const float true_pitch = deg_to_rad(1.0f);
    const PoseObservation first = pitched(true_pitch + deg_to_rad(5.0f));  // 첫 블록 오차 5도
    for (int i = 0; i < kBlockSize; ++i) {
        actual.update(first, 20.0f);
        ref.update(first);
    }
    compare_snapshot(actual.snapshot(), ref, "seed_block");
    EXPECT_EQ(actual.snapshot().valid_blocks, 0);
    EXPECT_NEAR(actual.snapshot().rpy[1], deg_to_rad(6.0f), 1e-5) << "워프는 첫 블록 값으로 옮긴다";

    for (int i = 0; i < 5 * kBlockSize; ++i) {
        float warp[3];
        actual.output_rpy(warp);
        const PoseObservation residual = pitched(true_pitch - warp[1]);
        actual.update(residual, 20.0f);
        ref.update(residual);
    }
    compare_snapshot(actual.snapshot(), ref, "five_blocks_after_seed");
    EXPECT_EQ(static_cast<int>(actual.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "첫 블록의 5도 오차로 장착 변경을 오판하지 않는다";
    EXPECT_NEAR(actual.snapshot().rpy[1], deg_to_rad(1.0f), 1e-4) << "보정은 참값으로 모인다";
    EXPECT_LT(actual.snapshot().spread[1], deg_to_rad(0.1f));
}

ParsedModelOutput parsed_from_pose(const PoseObservation &pose) {
    ParsedModelOutput output{};
    output.valid = true;
    output.pose = pose;
    return output;
}

// 소스 트리를 더럽히지 않게 테스트마다 시스템 임시 폴더를 params 디렉터리로 쓰고 끝나면 지운다.
// 실행마다 따로 만든다(mkdtemp): 고정 이름이면 두 실행(빌드 디렉터리 둘, 병렬 실행)이 서로의 파일을 지웠다.
struct TempParamsDir {
    std::string dir;
    explicit TempParamsDir(const char *name) {
        std::string pattern =
            (std::filesystem::temp_directory_path() / (std::string("edgepilot_test_") + name + "_XXXXXX")).string();
        dir = mkdtemp(pattern.data());
        setenv("EDGEPILOT_PARAMS_DIR", dir.c_str(), 1);
    }
    ~TempParamsDir() {
        unsetenv("EDGEPILOT_PARAMS_DIR");
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
    std::string file(const char *name) const { return dir + "/" + name; }
};

// 이 코드가 쓰는 형식의 calibration.json(rad)
void seed_calibration(const std::string &path, const float rpy[3], int valid_blocks) {
    std::FILE *f = std::fopen(path.c_str(), "w");
    ASSERT_NE(f, nullptr) << path;
    std::fprintf(f,
                 "{\n  \"version\": 1,\n  \"rpy_rad\": [%.9f, %.9f, %.9f],\n  \"spread_rad\": [0, 0, 0],\n"
                 "  \"valid_blocks\": %d\n}\n",
                 rpy[0], rpy[1], rpy[2], valid_blocks);
    std::fclose(f);
}

TEST(CalibrationEquivalence, CalibrationService) {
    const TempParamsDir params("calibration_service");

    OnlineCalibrator restored_calibrator;
    const float restored_rpy[3] = {0.0f, deg_to_rad(2.0f), deg_to_rad(-0.75f)};
    EXPECT_TRUE(restored_calibrator.restore(restored_rpy, 12)) << "저장된 유효 보정값을 복원한다";
    float restored_output[3] = {};
    restored_calibrator.output_rpy(restored_output);
    EXPECT_NEAR(restored_output[1], restored_rpy[1], 1e-7) << "복원한 pitch로 보정기를 초기화한다";
    EXPECT_NEAR(restored_output[2], restored_rpy[2], 1e-7) << "복원한 yaw로 보정기를 초기화한다";
    EXPECT_EQ(restored_calibrator.snapshot().valid_blocks, 12) << "복원한 유효 블록 수";

    AppConfig auto_config;
    auto_config.calibration_auto = true;
    CalibrationService service(auto_config);

    const ParsedModelOutput output = parsed_from_pose(make_pose());
    for (int i = 0; i < kBlockSize; ++i) service.update(output, 20.0f);

    float input_rpy[3] = {};
    service.input_rpy(input_rpy);
    const OnlineCalibrator::Snapshot snapshot = service.snapshot();
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(input_rpy[i], snapshot.rpy[i], 1e-7) << "자동 모드의 input_rpy는 온라인 스냅샷을 따른다";

    const float seeded[3] = {deg_to_rad(0.4f), deg_to_rad(1.0f), deg_to_rad(-0.5f)};
    seed_calibration(params.file("calibration.json"), seeded, 5);
    CalibrationService restored(auto_config);
    float persisted_rpy[3] = {};
    restored.input_rpy(persisted_rpy);
    EXPECT_NEAR(persisted_rpy[0], seeded[0], 1e-7) << "저장된 roll을 다시 읽는다";
    EXPECT_NEAR(persisted_rpy[1], seeded[1], 1e-7) << "저장된 pitch를 다시 읽는다";
    EXPECT_NEAR(persisted_rpy[2], seeded[2], 1e-7) << "저장된 yaw를 다시 읽는다";
    EXPECT_EQ(restored.snapshot().valid_blocks, 5);
    EXPECT_EQ(static_cast<int>(restored.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "다시 읽은 보정은 보정 완료 상태다";
}

TEST(CalibrationEquivalence, ResetRequestRecalibratesFromScratch) {
    const TempParamsDir params("calibration_reset");
    const std::string calibration = params.file("calibration.json");
    const std::string reset_request = params.file("calibration_reset");
    setenv("EDGEPILOT_CALIBRATION_RESET_PATH", reset_request.c_str(), 1);

    const float seeded[3] = {0.0f, deg_to_rad(2.3f), 0.0f};
    seed_calibration(calibration, seeded, 5);

    AppConfig auto_config;
    auto_config.calibration_auto = true;
    const ParsedModelOutput no_pose{};
    {
        CalibrationService service(auto_config);
        ASSERT_EQ(service.snapshot().valid_blocks, 5) << "저장된 보정으로 시작한다";
        service.update(no_pose, 0.0f);
        EXPECT_EQ(service.snapshot().valid_blocks, 5) << "요청이 없으면 그대로다";

        std::FILE *request = std::fopen(reset_request.c_str(), "w");
        ASSERT_NE(request, nullptr);
        std::fclose(request);
        // 요청은 1초에 한 번 본다. 첫 update에서 이미 봤으므로 간격을 넘겨 다시 부른다.
        for (int i = 0; i < 12 && service.snapshot().valid_blocks != 0; ++i) {
            struct timespec pause = {0, 100'000'000};
            nanosleep(&pause, nullptr);
            service.update(no_pose, 0.0f);
        }
        EXPECT_EQ(service.snapshot().valid_blocks, 0) << "초기화하면 블록이 없다";
        EXPECT_EQ(static_cast<int>(service.snapshot().status), static_cast<int>(CalibrationStatus::Uncalibrated));
        float rpy[3] = {1.0f, 1.0f, 1.0f};
        service.input_rpy(rpy);
        EXPECT_NEAR(rpy[1], 0.0f, 1e-7) << "모델 입력도 0에서 다시 시작한다";
        EXPECT_NE(access(reset_request.c_str(), F_OK), 0) << "요청 파일은 한 번 쓰고 지운다";
    }  // 쓰기 스레드가 남은 일을 마칠 때까지 기다린다
    EXPECT_NE(access(calibration.c_str(), F_OK), 0) << "저장된 보정 파일을 지운다";
    {
        CalibrationService restarted(auto_config);
        EXPECT_EQ(restarted.snapshot().valid_blocks, 0) << "재시작해도 예전 값을 다시 읽지 않는다";
    }

    unsetenv("EDGEPILOT_CALIBRATION_RESET_PATH");
}

// 초기화 뒤 다시 수렴한 보정은 저장된다(2026-10-01: 초기화의 삭제 표시가 남아 저장 대신 파일을
// 지웠고, 재시작하면 저장소 기본값이 복원돼 laneless가 0.2 m 치우쳤다).
TEST(CalibrationEquivalence, RecalibrationAfterResetIsSaved) {
    // ctest가 병렬로 돌리므로 ResetRequestRecalibratesFromScratch와 다른 디렉터리를 쓴다.
    const TempParamsDir params("calibration_reset_save");
    const std::string calibration = params.file("calibration.json");
    const std::string reset_request = params.file("calibration_reset");
    setenv("EDGEPILOT_CALIBRATION_RESET_PATH", reset_request.c_str(), 1);

    const float seeded[3] = {0.0f, deg_to_rad(2.3f), 0.0f};
    seed_calibration(calibration, seeded, 5);

    AppConfig auto_config;
    auto_config.calibration_auto = true;
    {
        CalibrationService service(auto_config);
        const ParsedModelOutput no_pose{};
        service.update(no_pose, 0.0f);
        std::FILE *request = std::fopen(reset_request.c_str(), "w");
        ASSERT_NE(request, nullptr);
        std::fclose(request);
        for (int i = 0; i < 12 && service.snapshot().valid_blocks != 0; ++i) {
            struct timespec pause = {0, 100'000'000};
            nanosleep(&pause, nullptr);
            service.update(no_pose, 0.0f);
        }
        ASSERT_EQ(service.snapshot().valid_blocks, 0);
        // 워프만 옮기는 첫 블록 + 직진 5블록(600프레임)으로 다시 보정 완료
        ParsedModelOutput straight{};
        straight.pose = make_pose();
        for (int i = 0; i < 6 * 100; ++i) service.update(straight, 20.0f);
        ASSERT_EQ(static_cast<int>(service.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));
    }  // 쓰기 스레드가 남은 일을 마칠 때까지 기다린다
    EXPECT_EQ(access(calibration.c_str(), F_OK), 0) << "다시 수렴한 보정을 저장한다";
    {
        CalibrationService restarted(auto_config);
        EXPECT_EQ(restarted.snapshot().valid_blocks, 5) << "재시작하면 새 보정으로 시작한다";
    }

    unsetenv("EDGEPILOT_CALIBRATION_RESET_PATH");
}

// openpilot과 같이 저장된 블록 수를 그대로 쓴다: 5 미만(저장소 기본값 0)이면 rpy는 출발점일 뿐
// 미보정이고, 직진 5블록이 모여야 보정 완료다. 0이면 새 보정이라 첫 블록은 워프만 옮긴다.
TEST(CalibrationEquivalence, StoredCalibrationBelowFiveBlocksStartsUncalibrated) {
    const float stored[3] = {0.0f, deg_to_rad(-1.0f), deg_to_rad(-1.12f)};
    OnlineCalibrator calibrator;
    ASSERT_TRUE(calibrator.restore(stored, 0));
    EXPECT_EQ(calibrator.snapshot().valid_blocks, 0);
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Uncalibrated));
    float out[3] = {};
    calibrator.output_rpy(out);
    EXPECT_NEAR(out[2], stored[2], 1e-7) << "모델 입력은 저장된 rpy에서 출발한다";
    for (int i = 0; i < 6 * 100; ++i) calibrator.update(make_pose(), 20.0f);
    EXPECT_EQ(calibrator.snapshot().valid_blocks, 5);
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));

    OnlineCalibrator converged;
    ASSERT_TRUE(converged.restore(stored, 12));
    EXPECT_EQ(static_cast<int>(converged.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));

    // 서비스: 저장소 기본값 같은 파일(valid_blocks 0)로 시작하면 미보정
    const TempParamsDir params("calibration_uncalibrated");
    const std::string calibration = params.file("calibration.json");
    {
        std::FILE *f = std::fopen(calibration.c_str(), "w");
        ASSERT_NE(f, nullptr);
        std::fprintf(f,
                     "{\n  \"version\": 1,\n  \"rpy_rad\": [0, %.9f, %.9f],\n  \"spread_rad\": [0, 0, 0],\n"
                     "  \"valid_blocks\": 0\n}\n",
                     stored[1], stored[2]);
        std::fclose(f);
    }
    AppConfig auto_config;
    auto_config.calibration_auto = true;
    {
        CalibrationService service(auto_config);
        EXPECT_EQ(static_cast<int>(service.snapshot().status), static_cast<int>(CalibrationStatus::Uncalibrated));
        float rpy[3] = {};
        service.input_rpy(rpy);
        EXPECT_NEAR(rpy[2], stored[2], 1e-6);
    }
}

void write_text(const std::string &path, const std::string &text) {
    std::FILE *f = std::fopen(path.c_str(), "w");
    ASSERT_NE(f, nullptr) << path;
    std::fputs(text.c_str(), f);
    std::fclose(f);
}

// flow_calibration.json: 같은 값의 블록 n개
std::string flow_json(float yaw_deg, float pitch_deg, int blocks) {
    std::string yaw, pitch;
    for (int i = 0; i < blocks; ++i) {
        yaw += (i ? ", " : "") + std::to_string(deg_to_rad(yaw_deg));
        pitch += (i ? ", " : "") + std::to_string(deg_to_rad(pitch_deg));
    }
    return "{\n  \"version\": 1,\n  \"yaw_rad\": [" + yaw + "],\n  \"pitch_rad\": [" + pitch +
           "],\n  \"windows\": 200,\n  \"mount_changes\": 0\n}\n";
}

ParsedModelOutput no_pose_output() { return ParsedModelOutput{}; }

AppConfig auto_calibration() {
    AppConfig config;
    config.calibration_auto = true;
    return config;
}

// 광류 측정원 흉내(보드에서는 FlowWorker). 초기화를 받으면 바로 처리한 것으로 센다.
struct FakeFlowSource : FlowSource {
    FlowCalibrationState current;
    FlowBlocks blocks;
    bool blocks_ready = false;
    uint32_t resets_requested = 0;
    uint32_t resets_applied = 0;
    bool apply_resets = true;  // false면 초기화를 아직 처리하지 않은 것처럼 군다
    FlowCalibrationState state(uint32_t *applied) const override {
        if (applied) *applied = resets_applied;
        return current;
    }
    bool take_blocks(FlowBlocks *out) override {
        if (!blocks_ready) return false;
        *out = blocks;
        blocks_ready = false;
        return true;
    }
    void request_reset() override {
        ++resets_requested;
        if (!apply_resets) return;
        ++resets_applied;
        current = FlowCalibrationState{};
        current.status = 1;
    }
};

FlowCalibrationState usable_flow_state(float yaw_deg, float pitch_deg, uint32_t blocks) {
    FlowCalibrationState f;
    f.status = 2;
    f.blocks = f.estimate_blocks = blocks;
    f.yaw = deg_to_rad(yaw_deg);
    f.pitch = deg_to_rad(pitch_deg);
    f.sd_yaw = f.sd_pitch = deg_to_rad(0.1f);
    return f;
}

TEST(CalibrationFlowSource, StoredFlowAppliesWhenTheSettingIsOn) {
    const TempParamsDir params("calibration_flow_source");
    const float pose[3] = {0.0f, deg_to_rad(-0.6f), deg_to_rad(-0.5f)};
    seed_calibration(params.file("calibration.json"), pose, 5);
    write_text(params.file("flow_calibration.json"), flow_json(-0.03f, -0.2f, 6));
    const AppConfig config = auto_calibration();
    {
        CalibrationService pose_only(config, CalibrationSource::Pose);
        float rpy[3] = {};
        pose_only.input_rpy(rpy);
        EXPECT_NEAR(rpy[2], deg_to_rad(-0.5f), 1e-6) << "설정이 꺼져 있으면 포즈 보정";
        EXPECT_EQ(pose_only.source_state().source, 0u);
        EXPECT_NE(pose_only.source_state().flags & kCalibrationSourceFlowUsable, 0u) << "저장된 광류 추정은 쓸 수 있다";
        EXPECT_EQ(pose_only.flow_state().estimate_blocks, 6u);
        EXPECT_EQ(pose_only.snapshot().valid_blocks, 5) << "포즈 출처면 포즈 보정기의 블록 수";
    }
    CalibrationService flow(config, CalibrationSource::Flow);
    float rpy[3] = {};
    flow.input_rpy(rpy);
    EXPECT_NEAR(rpy[1], deg_to_rad(-0.2f), 1e-6) << "첫 프레임부터 광류";
    EXPECT_NEAR(rpy[2], deg_to_rad(-0.03f), 1e-6);
    EXPECT_NEAR(flow.projection().yaw, deg_to_rad(-0.03f), 1e-6) << "HUD 투영도 같은 값";
    const CalibrationSourceState src = flow.source_state();
    EXPECT_EQ(src.source, 1u);
    EXPECT_EQ(src.requested, 1u);
    EXPECT_NEAR(src.pose_yaw, deg_to_rad(-0.5f), 1e-6) << "포즈 보정기 자신의 값은 따로 남는다";
    const OnlineCalibrator::Snapshot applied = flow.snapshot();
    EXPECT_EQ(static_cast<int>(applied.status), static_cast<int>(CalibrationStatus::Calibrated));
    EXPECT_EQ(applied.valid_blocks, 6) << "광류 출처면 광류 추정에 든 블록 수";
    EXPECT_NEAR(applied.spread[2], flow.flow_state().sd_yaw, 1e-9) << "광류 출처면 광류 블록 중앙값 표준편차";

    FakeFlowSource source;  // 측정원이 새 추정을 보고하면 천천히 옮긴다
    source.current = flow.flow_state();
    source.current.yaw = deg_to_rad(0.3f);
    flow.attach_flow(&source);
    const ParsedModelOutput no_pose{};
    flow.update(no_pose, 0.0f, false, CalibrationSource::Flow);
    flow.input_rpy(rpy);
    EXPECT_LT(rpy[2], 0.0f) << "한 번에 옮기지 않는다";
    EXPECT_NE(flow.source_state().flags & kCalibrationSourceRamping, 0u);
    flow.attach_flow(nullptr);
}

TEST(CalibrationFlowSource, PoseLoopPausesWhileFlowDrivesTheWarp) {
    // upstream 루프는 모델이 자기 값으로 워프된 영상을 봤다고 가정하므로, 광류가 워프를 맡는 동안에는 멈춘다.
    // 광류 추정을 포즈 추정과 같게 두어 돌아갈 때 램프가 없게 한다(램프는 실제 시간으로 간다).
    const TempParamsDir params("calibration_flow_pause");
    const float pose[3] = {0.0f, 0.0f, deg_to_rad(-0.5f)};
    seed_calibration(params.file("calibration.json"), pose, 5);
    write_text(params.file("flow_calibration.json"), flow_json(-0.5f, 0.0f, 6));
    CalibrationService service(auto_calibration(), CalibrationSource::Flow);
    ASSERT_EQ(service.source_state().source, 1u);
    const float bias = deg_to_rad(-0.3f);  // 모델 pose의 치우침이 남아도
    const ParsedModelOutput biased = parsed_from_pose(make_pose(20.0f, 20.0f * std::tan(bias), 0.0f));
    for (int i = 0; i < 20 * kBlockSize; ++i) service.update(biased, 20.0f, false, CalibrationSource::Flow);
    CalibrationSourceState src = service.source_state();
    EXPECT_EQ(src.pose_valid_blocks, 5) << "광류가 워프를 맡는 동안 포즈 보정기는 블록을 모으지 않는다";
    EXPECT_NEAR(src.pose_yaw, deg_to_rad(-0.5f), 1e-7) << "멈춘 마지막 추정 그대로";
    EXPECT_NEAR(src.pose_roll, 0.0f, 1e-7);

    service.update(no_pose_output(), 0.0f, false, CalibrationSource::Pose);  // 포즈로 돌아간다(같은 값이라 바로)
    src = service.source_state();
    EXPECT_EQ(src.source, 0u);
    EXPECT_EQ(src.flags & kCalibrationSourceRamping, 0u);
    // 복원한 보정기는 upstream reset()처럼 block_idx 0부터 덮어쓰므로 valid_blocks는 5에 머문다.
    // 추정이 움직이는지 본다.
    for (int i = 0; i < 2 * kBlockSize; ++i) service.update(biased, 20.0f, false, CalibrationSource::Pose);
    EXPECT_LT(service.source_state().pose_yaw, deg_to_rad(-0.55f)) << "포즈 보정기가 다시 돈다";
}

TEST(CalibrationFlowSource, ResetClearsBothAndTellsTheSource) {
    const TempParamsDir params("calibration_flow_reset");
    const std::string reset_request = params.file("calibration_reset");
    setenv("EDGEPILOT_CALIBRATION_RESET_PATH", reset_request.c_str(), 1);
    write_text(params.file("flow_calibration.json"), flow_json(-0.03f, -0.2f, 6));
    const ParsedModelOutput no_pose{};
    FakeFlowSource source;
    source.current = usable_flow_state(-0.03f, -0.2f, 6);
    source.apply_resets = false;  // 측정원이 초기화를 처리하기 전의 상태를 먼저 본다
    {
        CalibrationService service(auto_calibration(), CalibrationSource::Flow);
        service.attach_flow(&source);
        ASSERT_EQ(service.source_state().source, 1u);
        write_text(reset_request, "");
        for (int i = 0; i < 12 && source.resets_requested == 0; ++i) {
            struct timespec pause = {0, 100'000'000};
            nanosleep(&pause, nullptr);
            service.update(no_pose, 0.0f, false, CalibrationSource::Flow);
        }
        EXPECT_EQ(source.resets_requested, 1u) << "측정원에도 알린다";
        EXPECT_EQ(service.source_state().source, 0u) << "초기화하면 포즈 0에서 다시";
        float rpy[3] = {1.0f, 1.0f, 1.0f};
        service.input_rpy(rpy);
        EXPECT_NEAR(rpy[2], 0.0f, 1e-7) << "램프 없이 바로";

        source.blocks.yaw = source.blocks.pitch = {0.0f, 0.0f, 0.0f};
        source.blocks_ready = true;
        service.update(no_pose, 0.0f, false, CalibrationSource::Flow);
        EXPECT_EQ(service.source_state().source, 0u) << "처리 전의 예전 추정은 무시한다";
        EXPECT_EQ(service.flow_state().estimate_blocks, 0u);
        EXPECT_TRUE(source.blocks_ready) << "처리 전에 끝난 블록은 받지 않는다";

        source.resets_applied = 1;  // 측정원이 초기화를 처리했다(그 사이 블록이 쌓였어도 처리 수로 가른다)
        source.current = FlowCalibrationState{};
        source.current.status = 1;
        source.current.blocks = 2;
        source.blocks_ready = false;
        service.update(no_pose, 0.0f, false, CalibrationSource::Flow);
        EXPECT_EQ(service.flow_state().status, 1u) << "처리한 뒤의 상태는 받는다";
        EXPECT_EQ(service.flow_state().blocks, 2u);
        EXPECT_EQ(source.resets_requested, 1u) << "한 번만 보낸다";
        service.attach_flow(nullptr);
    }
    EXPECT_NE(access(params.file("flow_calibration.json").c_str(), F_OK), 0) << "광류 블록 파일도 지운다";
    unsetenv("EDGEPILOT_CALIBRATION_RESET_PATH");
}

TEST(CalibrationFlowSource, ResetWaitsForDisengage) {
    const TempParamsDir params("calibration_flow_reset_engaged");
    const std::string reset_request = params.file("calibration_reset");
    setenv("EDGEPILOT_CALIBRATION_RESET_PATH", reset_request.c_str(), 1);
    write_text(params.file("flow_calibration.json"), flow_json(-0.03f, -0.2f, 6));
    const ParsedModelOutput no_pose{};
    CalibrationService service(auto_calibration(), CalibrationSource::Flow);
    write_text(reset_request, "");
    for (int i = 0; i < 12; ++i) {
        struct timespec pause = {0, 100'000'000};
        nanosleep(&pause, nullptr);
        service.update(no_pose, 0.0f, true, CalibrationSource::Flow);
    }
    EXPECT_EQ(service.source_state().source, 1u) << "결합 중에는 초기화하지 않는다";
    EXPECT_EQ(access(reset_request.c_str(), F_OK), 0) << "요청은 남겨 둔다";
    for (int i = 0; i < 12 && service.source_state().source != 0u; ++i) {
        struct timespec pause = {0, 100'000'000};
        nanosleep(&pause, nullptr);
        service.update(no_pose, 0.0f, false, CalibrationSource::Flow);
    }
    EXPECT_EQ(service.source_state().source, 0u) << "해제되면 초기화한다";
    EXPECT_NE(access(reset_request.c_str(), F_OK), 0);
    unsetenv("EDGEPILOT_CALIBRATION_RESET_PATH");
}

TEST(CalibrationFlowSource, SavesFlowBlocksFromTheSource) {
    const TempParamsDir params("calibration_flow_save");
    const ParsedModelOutput no_pose{};
    FakeFlowSource source;
    source.current = usable_flow_state(-0.15f, -1.0f, 3);
    source.blocks.yaw = {deg_to_rad(-0.1f), deg_to_rad(-0.2f), deg_to_rad(-0.15f)};
    source.blocks.pitch = {deg_to_rad(-1.0f), deg_to_rad(-1.1f), deg_to_rad(-0.9f)};
    source.blocks.windows = 60;
    source.blocks_ready = true;
    {
        CalibrationService service(auto_calibration(), CalibrationSource::Pose);
        service.attach_flow(&source);
        service.update(no_pose, 0.0f, false, CalibrationSource::Pose);  // 첫 저장은 바로 나간다
        EXPECT_FALSE(source.blocks_ready);
        service.attach_flow(nullptr);
    }
    CalibrationService restarted(auto_calibration(), CalibrationSource::Pose);
    ASSERT_EQ(restarted.stored_flow_blocks().yaw.size(), 3u);
    EXPECT_EQ(restarted.stored_flow_blocks().windows, 60u);
    EXPECT_EQ(restarted.flow_state().status, 2u);
    EXPECT_NEAR(restarted.flow_state().yaw, deg_to_rad(-0.15f), 1e-6);
    EXPECT_NEAR(restarted.flow_state().pitch, deg_to_rad(-1.0f), 1e-6);
}

}  // namespace
