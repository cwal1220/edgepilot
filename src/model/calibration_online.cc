#include "model/calibration_online.h"
#include "common/projection.h"  // rotation_from_rpy
#include "common/utils_math.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr float kMinSpeedFilter = 15.0f * 0.44704f;
constexpr float kMaxVelAngleStd = 0.25f * kPi / 180.0f;
constexpr float kMaxYawRateFilter = 2.0f * kPi / 180.0f;
// openpilot calibrationd.py: 블록 사이 편차가 이보다 크면 장착이 바뀐 것으로 본다.
constexpr float kMaxAllowedYawSpread = 2.0f * kPi / 180.0f;
constexpr float kMaxAllowedPitchSpread = 4.0f * kPi / 180.0f;
// 그때 옛 값에서 새 값으로 넘어가는 표본 수(20 Hz에서 0.5초).
constexpr float kSmoothCycles = 10.0f;
constexpr float kPitchMin = -0.09074112085129739f;
// upstream PITCH_LIMITS(mici가 아닌 장치). 2026-10 master에서 0.149 -> 0.17로 넓어졌다.
constexpr float kPitchMax = 0.17f;
constexpr float kYawMin = -0.06912048084718224f;
constexpr float kYawMax = 0.06912048084718235f;
constexpr float kSanityMargin = 0.005f;
// upstream MAX_HEIGHT_STD: 보정 뒤에는 높이(road_transform z) 표준편차가 이보다 작은 표본만 쓴다.
const float kMaxHeightStd = std::exp(-3.5f);

bool finite3(const float v[3]) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }

void euler_from_rot(const float *rot, float rpy[3]) {
    rpy[0] = std::atan2(rot[2 * 3 + 1], rot[2 * 3 + 2]);
    rpy[1] = std::asin(std::max(-1.0f, std::min(1.0f, -rot[2 * 3 + 0])));
    rpy[2] = std::atan2(rot[1 * 3 + 0], rot[0 * 3 + 0]);
}

void compose_rpy(const float base[3], const float observed[3], float out[3]) {
    float base_rot[9];
    float observed_rot[9];
    float composed[9];
    rotation_from_rpy(base[0], base[1], base[2], base_rot);
    rotation_from_rpy(observed[0], observed[1], observed[2], observed_rot);
    matmul3(base_rot, observed_rot, composed);
    euler_from_rot(composed, out);
}

}  // namespace

OnlineCalibrator::OnlineCalibrator() { reset_to_rpy(snapshot_.rpy, 0); }

OnlineCalibrator::Output OnlineCalibrator::update(const PoseObservation &pose, float v_ego) {
    Output result;
    result.snapshot = snapshot_;
    old_rpy_weight_ = std::max(0.0f, old_rpy_weight_ - 1.0f / kSmoothCycles);

    const bool straight_and_fast =
        v_ego > kMinSpeedFilter && pose.trans[0] > kMinSpeedFilter && std::fabs(pose.rot[2]) < kMaxYawRateFilter;
    const bool rpy_certain = std::atan2(pose.trans_std[1], pose.trans[0]) < kMaxVelAngleStd;
    const bool height_certain = pose.road_trans_std[2] < kMaxHeightStd;
    const bool certain_if_calib = (rpy_certain && height_certain) || snapshot_.valid_blocks < kInputsNeeded;

    if (!straight_and_fast || !certain_if_calib) {
        ++snapshot_.rejected_samples;
        result.snapshot = snapshot_;
        return result;
    }

    const float observed_rpy[3] = {
        0.0f,
        -std::atan2(pose.trans[2], pose.trans[0]),
        std::atan2(pose.trans[1], pose.trans[0]),
    };
    float base_rpy[3];
    float new_rpy[3];
    smooth_rpy(base_rpy);
    compose_rpy(base_rpy, observed_rpy, new_rpy);
    sanity_clip(new_rpy);  // 유한하지 않으면 0으로 둔다(상류 RPY_INIT)

    for (int i = 0; i < 3; ++i)
        block_rpys_[block_idx_][i] =
            (sample_idx_ * block_rpys_[block_idx_][i] + (kBlockSize - sample_idx_) * new_rpy[i]) /
            static_cast<float>(kBlockSize);
    // upstream: road_transform이 없으면 HEIGHT_INIT. 유한하지 않은 값도 같이 본다.
    const float new_height = std::isfinite(pose.road_trans[2]) ? pose.road_trans[2] : kHeightInit;
    block_heights_[block_idx_] = (sample_idx_ * block_heights_[block_idx_] + (kBlockSize - sample_idx_) * new_height) /
                                 static_cast<float>(kBlockSize);

    sample_idx_ = (sample_idx_ + 1) % kBlockSize;
    ++snapshot_.accepted_samples;
    snapshot_.block_sample_count = sample_idx_;
    result.accepted = true;

    if (sample_idx_ == 0 && !seeded_) {
        // 새 보정의 첫 블록은 보정 안 된 워프로 잰 것이라 몇 도씩 틀릴 수 있다. 워프만 그 값으로 옮기고 블록으로
        // 세지 않는다: 세면 블록이 다 찼을 때 편차 검사가 장착 변경으로 오판한다(upstream #39170, 직진 약 5초가 더
        // 든다).
        seeded_ = true;
        for (int i = 0; i < 3; ++i) snapshot_.rpy[i] = block_rpys_[0][i];
    } else if (sample_idx_ == 0) {
        ++block_idx_;
        snapshot_.valid_blocks = std::min(kInputsWanted, std::max(block_idx_, snapshot_.valid_blocks));
        block_idx_ %= kInputsWanted;
        result.block_completed = true;
    }
    update_status();

    result.snapshot = snapshot_;
    return result;
}

bool OnlineCalibrator::restore(const float rpy[3], int valid_blocks, const float spread[3], float height_m) {
    if (!rpy) return false;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float *start = finite3(rpy) ? rpy : zero;  // upstream reset: 유한하지 않으면 RPY_INIT
    // openpilot calibrationd reset(rpy_init, valid_blocks)와 같이 저장된 블록 수를 그대로 쓴다.
    // 5블록 미만(저장소 기본값은 0)이면 rpy는 출발점일 뿐 미보정이고, 직진 5블록이 모일 때까지
    // 결합이 막힌다. 예전에는 5블록으로 올려 기본값을 보정 완료로 믿었다(2026-10-01 laneless
    // 0.2 m 치우침).
    reset_to_rpy(start, std::max(0, valid_blocks), nullptr, height_m);
    // 저장된 블록이 있으면 이미 그 워프로 잰 보정이라 첫 블록을 건너뛰지 않는다(upstream __init__의 seeded)
    seeded_ = snapshot_.valid_blocks > 0;
    if (spread && finite3(spread)) {
        for (int i = 0; i < 3; ++i) snapshot_.spread[i] = std::max(0.0f, spread[i]);
    }
    return true;
}

OnlineCalibrator::Snapshot OnlineCalibrator::snapshot() const { return snapshot_; }

void OnlineCalibrator::output_rpy(float rpy[3]) const { smooth_rpy(rpy); }

void OnlineCalibrator::reset_to_rpy(const float rpy[3], int valid_blocks, const float *smooth_from, float height_m) {
    const float height = std::isfinite(height_m) ? height_m : kHeightInit;
    for (int b = 0; b < kInputsWanted; ++b) block_heights_[b] = height;
    snapshot_.height_m = height;
    const int clamped_valid_blocks = std::max(0, std::min(valid_blocks, kInputsWanted));
    for (int b = 0; b < kInputsWanted; ++b) {
        for (int i = 0; i < 3; ++i) block_rpys_[b][i] = rpy[i];
    }
    sample_idx_ = 0;
    block_idx_ = 0;
    if (smooth_from) {
        for (int i = 0; i < 3; ++i) old_rpy_[i] = smooth_from[i];
        old_rpy_weight_ = 1.0f;
    } else {
        std::memset(old_rpy_, 0, sizeof(old_rpy_));
        old_rpy_weight_ = 0.0f;
    }
    for (int i = 0; i < 3; ++i) {
        snapshot_.rpy[i] = rpy[i];
        snapshot_.spread[i] = 0.0f;
    }
    snapshot_.valid_blocks = clamped_valid_blocks;
    snapshot_.block_sample_count = 0;
    // upstream __init__의 update_status: 5블록 미만이면 미보정, 범위 밖이면 Invalid
    snapshot_.status = clamped_valid_blocks < kInputsNeeded ? CalibrationStatus::Uncalibrated
                       : is_calibration_valid(rpy)          ? CalibrationStatus::Calibrated
                                                            : CalibrationStatus::Invalid;
}

void OnlineCalibrator::update_status() {
    float sum[3] = {};
    float height_sum = 0.0f;
    float min_v[3] = {};
    float max_v[3] = {};
    int valid_count = 0;
    for (int b = 0; b < snapshot_.valid_blocks; ++b) {
        if (b == block_idx_) continue;
        if (valid_count == 0) {
            min_v[0] = max_v[0] = block_rpys_[b][0];
            min_v[1] = max_v[1] = block_rpys_[b][1];
            min_v[2] = max_v[2] = block_rpys_[b][2];
        }
        height_sum += block_heights_[b];
        for (int i = 0; i < 3; ++i) {
            const float value = block_rpys_[b][i];
            sum[i] += value;
            min_v[i] = std::min(min_v[i], value);
            max_v[i] = std::max(max_v[i], value);
        }
        ++valid_count;
    }

    if (valid_count > 0) {
        snapshot_.height_m = height_sum / static_cast<float>(valid_count);
        for (int i = 0; i < 3; ++i) {
            snapshot_.rpy[i] = sum[i] / static_cast<float>(valid_count);
            snapshot_.spread[i] = std::fabs(max_v[i] - min_v[i]);
        }
    } else {
        for (int i = 0; i < 3; ++i) snapshot_.spread[i] = 0.0f;
    }

    if (snapshot_.valid_blocks < kInputsNeeded) {
        if (snapshot_.status != CalibrationStatus::Recalibrating) snapshot_.status = CalibrationStatus::Uncalibrated;
    } else if (is_calibration_valid(snapshot_.rpy)) {
        snapshot_.status = CalibrationStatus::Calibrated;
    } else {
        snapshot_.status = CalibrationStatus::Invalid;
    }

    // 편차가 크면 장착이 바뀐 것으로 보고 마지막 블록 값에서 블록 0개로 다시 모은다(openpilot과 같이
    // recalibrating). 워프는 이미 그 값이라 첫 블록부터 센다. 5블록이 다시 찰 때까지 보정 완료가 아니므로 저장하지
    // 않고 학습기도 보정을 믿지 않는다. roll은 관측하지 않으므로 보지 않는다.
    const bool spread_too_high =
        snapshot_.spread[1] > kMaxAllowedPitchSpread || snapshot_.spread[2] > kMaxAllowedYawSpread;
    if (snapshot_.status == CalibrationStatus::Calibrated && spread_too_high) {
        const int last_block = (block_idx_ + kInputsWanted - 1) % kInputsWanted;
        float smooth_from[3] = {snapshot_.rpy[0], snapshot_.rpy[1], snapshot_.rpy[2]};
        float last_rpy[3] = {block_rpys_[last_block][0], block_rpys_[last_block][1], block_rpys_[last_block][2]};
        // upstream reset(rpys[block_idx - 1], smooth_from=rpy): 높이는 HEIGHT_INIT로 돌아간다
        reset_to_rpy(last_rpy, 0, smooth_from, kHeightInit);
        snapshot_.status = CalibrationStatus::Recalibrating;
    }
}

void OnlineCalibrator::smooth_rpy(float out[3]) const {
    if (old_rpy_weight_ > 0.0f) {
        for (int i = 0; i < 3; ++i)
            out[i] = old_rpy_weight_ * old_rpy_[i] + (1.0f - old_rpy_weight_) * snapshot_.rpy[i];
    } else {
        for (int i = 0; i < 3; ++i) out[i] = snapshot_.rpy[i];
    }
}

bool OnlineCalibrator::rpy_within_limits(const float rpy[3]) {
    return kPitchMin < rpy[1] && rpy[1] < kPitchMax && kYawMin < rpy[2] && rpy[2] < kYawMax;
}

bool OnlineCalibrator::is_calibration_valid(const float rpy[3]) const { return rpy_within_limits(rpy); }

void OnlineCalibrator::sanity_clip(float rpy[3]) const {
    if (!finite3(rpy)) {
        rpy[0] = 0.0f;
        rpy[1] = 0.0f;
        rpy[2] = 0.0f;
        return;
    }
    rpy[1] = std::max(kPitchMin - kSanityMargin, std::min(kPitchMax + kSanityMargin, rpy[1]));
    rpy[2] = std::max(kYawMin - kSanityMargin, std::min(kYawMax + kSanityMargin, rpy[2]));
}
