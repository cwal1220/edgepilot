#pragma once

// openpilot master supercombo의 시간축 입력. 공개 ONNX는 이 큐들을 그래프 안에
// 갖고 있지만 NPU 코어에서는 떼어냈으므로(tools/model/axmodel/extract_core.py) 런타임이
// 같은 규약으로 관리한다. 모든 큐는 최신 값이 마지막 슬롯이고 매 프레임 한 칸씩
// 앞으로 밀린다(특징 이력과 이미지는 복사 없이 링 인덱스만 돈다).
//
// 크기는 Smoked Cheddar(openpilot e170f08, 2026-11-24, sha256 1fde7dcb…)의 그래프를 따른다. 이전 출시
// 모델(65a08adc, desire 100틱·특징 96틱)과 그 그래프로 만든 학생 모델(43, 45)은 이 크기와 맞지 않아 로드에서
// 거부된다.
//  - desire: 20 Hz rising-edge 펄스 132틱. 모델에는 4틱씩 max-pool한 33x8(5 Hz).
//  - 특징: 직전 128틱의 hidden_state(512). 모델에는 0,4,...,124번 슬롯 32개(6.4초). 이번
//    프레임 출력은 실행 뒤 마지막(127번) 슬롯에 넣으므로, 모델은 자기 출력을 4프레임
//    뒤(124번 슬롯에 이를 때)부터 본다.
//  - 이미지: 탑마다 최근 5프레임(6평면 YUV). 모델에는 가장 오래된 것(t-4)과
//    최신(t)을 이어 붙인 12평면.
// axengine에 의존하지 않아 호스트 검사가 규약을 본다.

#include "common/model_output.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

class SupercomboTemporalState {
public:
    static constexpr int kDesireHistoryTicks = 132;
    static constexpr int kDesireTurnLeft = 1;  // 상류 DESIRES의 turnLeft, turnRight
    static constexpr int kDesireTurnRight = 2;
    static constexpr int kDesirePool = 4;
    static constexpr int kDesireInputTicks = kDesireHistoryTicks / kDesirePool;
    static constexpr int kFeatureHistoryTicks = 128;
    static constexpr int kFeatureStride = 4;
    static constexpr int kFeatureInputTicks = kFeatureHistoryTicks / kFeatureStride;
    static constexpr size_t kHiddenOffset = model_output_layout::kFeatureOffset;

    SupercomboTemporalState()
        : desire_(kDesireLen, 0.0f),
          prev_desire_(kDesireLen, 0.0f),
          desire_history_(kDesireHistoryTicks * kDesireLen, 0.0f),
          feature_history_(kFeatureHistoryTicks * kModelFeatureLen, 0.0f) {}

    // openpilot 방식 rising-edge 펄스: desire가 바뀐 틱에만 1. 0(없음)은 항상 0. 회전 desire(1, 2)가 아니게 되면
    // 이력의 회전 펄스를 지운다: 회전을 마친 뒤에도 5초 동안 모델이 회전하려 하지 않게 한다(차선 변경 펄스는
    // 상류처럼 남긴다). tools/model/axmodel/make_m2_data.py의 desire_step이 같은 규칙이다(check_recording_reader가
    // 대조한다).
    void set_desire(int desire) {
        for (int i = 1; i < static_cast<int>(desire_.size()); ++i) {
            const float current = i == desire ? 1.0f : 0.0f;
            desire_[i] = current - prev_desire_[i] > 0.99f ? current : 0.0f;
            prev_desire_[i] = current;
        }
        if (desire != kDesireTurnLeft && desire != kDesireTurnRight) {
            for (int t = 0; t < kDesireHistoryTicks; ++t) {
                desire_history_[t * kDesireLen + kDesireTurnLeft] = 0.0f;
                desire_history_[t * kDesireLen + kDesireTurnRight] = 0.0f;
            }
        }
    }

    // 오래된 틱을 앞으로 밀고 마지막 슬롯에 현재 펄스를 넣는다. 매 프레임 실행 직전.
    void push_desire_pulse() {
        std::memmove(desire_history_.data(), desire_history_.data() + kDesireLen,
                     sizeof(float) * kDesireLen * (kDesireHistoryTicks - 1));
        std::memcpy(desire_history_.data() + kDesireLen * (kDesireHistoryTicks - 1), desire_.data(),
                    sizeof(float) * kDesireLen);
    }

    // 모델 입력 desire [33, 8]: 연속한 4틱마다 원소별 최댓값.
    void desire_input(float *out) const {
        for (int t = 0; t < kDesireInputTicks; ++t) {
            for (int d = 0; d < kDesireLen; ++d) {
                float value = desire_history_[(t * kDesirePool) * kDesireLen + d];
                for (int k = 1; k < kDesirePool; ++k)
                    value = std::max(value, desire_history_[(t * kDesirePool + k) * kDesireLen + d]);
                out[t * kDesireLen + d] = value;
            }
        }
    }

    // 모델 입력 features_buffer [32, 512]: 특징 이력의 0,4,...,124번 슬롯(0이 가장 오래된 것).
    void feature_input(float *out) const {
        for (int t = 0; t < kFeatureInputTicks; ++t)
            std::memcpy(out + t * kModelFeatureLen, feature_slot(t * kFeatureStride), sizeof(float) * kModelFeatureLen);
    }

    // 실행 직후 raw 출력(kModelOutputFloats)의 hidden_state를 특징 이력 마지막 슬롯에 넣는다. 이력은 128칸 링이라
    // 가장 오래된 칸에 쓰고 머리만 옮긴다(예전에는 프레임마다 194 KB를 memmove로 밀었다).
    void push_feature_history(const float *raw_output) {
        std::memcpy(feature_history_.data() + static_cast<size_t>(feature_head_) * kModelFeatureLen,
                    raw_output + kHiddenOffset, sizeof(float) * kModelFeatureLen);
        feature_head_ = (feature_head_ + 1) % kFeatureHistoryTicks;
    }

    const std::vector<float> &desire_history() const { return desire_history_; }
    // 특징 이력을 시간 순(가장 오래된 것부터)으로 복사한다(시험용).
    std::vector<float> feature_history() const {
        std::vector<float> ordered(feature_history_.size());
        for (int age = 0; age < kFeatureHistoryTicks; ++age)
            std::memcpy(ordered.data() + static_cast<size_t>(age) * kModelFeatureLen, feature_slot(age),
                        sizeof(float) * kModelFeatureLen);
        return ordered;
    }

private:
    // age 0 = 가장 오래된 칸, kFeatureHistoryTicks-1 = 최신. feature_head_는 다음에 쓸 칸(= 가장 오래된 칸).
    const float *feature_slot(int age) const {
        return feature_history_.data() +
               static_cast<size_t>((feature_head_ + age) % kFeatureHistoryTicks) * kModelFeatureLen;
    }

    std::vector<float> desire_;
    std::vector<float> prev_desire_;
    std::vector<float> desire_history_;
    std::vector<float> feature_history_;
    int feature_head_ = 0;
};

// 한 탑(road 또는 wide)의 최근 5프레임 이미지. 새 프레임은 newest_slot()에 직접
// 워프해 넣고 commit()으로 한 칸 민다. 복사 없이 링 인덱스만 돈다.
class ModelImageHistory {
public:
    static constexpr int kFrames = 5;
    static constexpr int kPlaneW = 256;
    static constexpr int kPlaneH = 128;
    static constexpr size_t kFrameBytes = 6 * kPlaneW * kPlaneH;

    ModelImageHistory() : frames_(kFrames * kFrameBytes, 0) {}

    // 다음 commit()에서 최신이 될 칸. 워프가 여기에 6평면을 쓴다.
    uint8_t *newest_slot() { return frames_.data() + static_cast<size_t>(head_) * kFrameBytes; }
    void commit() { head_ = (head_ + 1) % kFrames; }

    const uint8_t *oldest() const { return frame(0); }
    const uint8_t *newest() const { return frame(kFrames - 1); }

    // 모델 입력 12평면: 가장 오래된 프레임(t-4) 다음 최신(t).
    void model_input(uint8_t *out) const {
        std::memcpy(out, oldest(), kFrameBytes);
        std::memcpy(out + kFrameBytes, newest(), kFrameBytes);
    }

private:
    // age 0 = 가장 오래된 칸, kFrames-1 = 최신. head_는 다음에 쓸 칸(= 가장 오래된 칸).
    const uint8_t *frame(int age) const {
        return frames_.data() + static_cast<size_t>((head_ + age) % kFrames) * kFrameBytes;
    }

    std::vector<uint8_t> frames_;
    int head_ = 0;
};
