#pragma once

// 모델 포즈로 카메라 장착 각도를 배우는 온라인 보정기(상류 calibrationd의 이식).

#include "common/ipc_messages.h"  // CalibrationStatus
#include "common/model_output.h"  // PoseObservation(모델 pose 출력)

#include <cstdint>

class OnlineCalibrator {
public:
    struct Snapshot {
        float rpy[3] = {0.0f, 0.0f, 0.0f};
        float spread[3] = {0.0f, 0.0f, 0.0f};
        float height_m = 1.22f;  // 도로면에서 카메라까지(upstream extrinsicsCalibration.height)
        int valid_blocks = 0;
        int block_sample_count = 0;
        CalibrationStatus status = CalibrationStatus::Uncalibrated;
        uint64_t accepted_samples = 0;
        uint64_t rejected_samples = 0;
    };

    struct Output {
        Snapshot snapshot;
        bool accepted = false;
        bool block_completed = false;
    };

    OnlineCalibrator();

    Output update(const PoseObservation &pose, float v_ego);
    // 저장된 보정으로 시작한다(openpilot reset). 블록 수를 그대로 써서 5 미만이면 미보정이고, 0이면 새 보정처럼
    // 첫 블록은 워프만 옮긴다.
    // openpilot처럼 범위를 벗어난 rpy도 받아 Invalid로 두고, 유한하지 않으면 0에서 시작한다.
    // rpy 포인터가 있으면 true.
    bool restore(const float rpy[3], int valid_blocks, const float spread[3] = nullptr, float height_m = kHeightInit);
    Snapshot snapshot() const;
    void output_rpy(float rpy[3]) const;

    // 보정 완료(Calibrated)에 필요한 블록 수. HUD가 보정 진행률을 이것으로 낸다.
    static constexpr int kInputsNeeded = 5;
    // upstream PITCH_LIMITS·YAW_LIMITS 안인지(Calibrated의 조건). 광류 추정도 같은 한계를 쓴다.
    static bool rpy_within_limits(const float rpy[3]);
    static constexpr float kHeightInit = 1.22f;  // upstream HEIGHT_INIT

private:
    static constexpr int kBlockSize = 100;
    static constexpr int kInputsWanted = 50;

    void reset_to_rpy(const float rpy[3], int valid_blocks, const float *smooth_from = nullptr,
                      float height_m = kHeightInit);
    void update_status();
    void smooth_rpy(float out[3]) const;
    bool is_calibration_valid(const float rpy[3]) const;
    void sanity_clip(float rpy[3]) const;

    float block_rpys_[kInputsWanted][3] = {};
    float block_heights_[kInputsWanted] = {};
    int sample_idx_ = 0;
    int block_idx_ = 0;
    float old_rpy_[3] = {};
    float old_rpy_weight_ = 0.0f;
    // 첫 블록으로 워프를 옮겼다(upstream seeded). 새 보정에서는 그 블록을 세지 않는다.
    bool seeded_ = false;
    Snapshot snapshot_;
};
