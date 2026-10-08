#ifndef MODEL_TEMPORAL_H
#define MODEL_TEMPORAL_H

/* openpilot master supercombo의 시간축 입력. 공개 ONNX는 이 큐들을 그래프 안에
 * 갖고 있지만 NPU 코어에서는 떼어냈으므로(tools/model/axmodel/extract_core.py) 런타임이
 * 같은 규약으로 관리한다. 모든 큐는 최신 값이 마지막 슬롯이고 매 프레임 한 칸씩
 * 앞으로 밀린다.
 *
 *  - desire: 20 Hz rising-edge 펄스 100틱. 모델에는 4틱씩 max-pool한 25x8(5 Hz).
 *  - 특징: 직전 96틱의 hidden_state(512). 모델에는 0,4,...,92번 슬롯 24개. 이번
 *    프레임 출력은 실행 뒤에 넣으므로 모델은 자기 출력을 다음 프레임부터 본다.
 *  - 이미지: 탑마다 최근 5프레임(6평면 YUV). 모델에는 가장 오래된 것(t-4)과
 *    최신(t)을 이어 붙인 12평면.
 * axengine에 의존하지 않아 호스트 검사가 규약을 본다. */

#include "common/model_output.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

class SupercomboTemporalState
{
public:
    static constexpr int kDesireHistoryTicks = 100;
    static constexpr int kDesirePool = 4;
    static constexpr int kDesireInputTicks = kDesireHistoryTicks / kDesirePool;
    static constexpr int kFeatureHistoryTicks = 96;
    static constexpr int kFeatureStride = 4;
    static constexpr int kFeatureInputTicks = kFeatureHistoryTicks / kFeatureStride;
    static constexpr size_t kHiddenOffset = model_output_layout::kFeatureOffset;

    SupercomboTemporalState()
        : desire_(kDesireLen, 0.0f),
          prev_desire_(kDesireLen, 0.0f),
          traffic_convention_{1.0f, 0.0f},
          desire_history_(kDesireHistoryTicks * kDesireLen, 0.0f),
          feature_history_(kFeatureHistoryTicks * kModelFeatureLen, 0.0f)
    {
    }

    /* openpilot 방식 rising-edge 펄스: desire가 바뀐 틱에만 1. 0(없음)은 항상 0. */
    void set_desire(int desire)
    {
        for (int i = 1; i < static_cast<int>(desire_.size()); ++i) {
            const float current = i == desire ? 1.0f : 0.0f;
            desire_[i] = current - prev_desire_[i] > 0.99f ? current : 0.0f;
            prev_desire_[i] = current;
        }
    }

    // 오래된 틱을 앞으로 밀고 마지막 슬롯에 현재 펄스를 넣는다. 매 프레임 실행 직전.
    void push_desire_pulse()
    {
        std::memmove(desire_history_.data(), desire_history_.data() + kDesireLen,
                     sizeof(float) * kDesireLen * (kDesireHistoryTicks - 1));
        std::memcpy(desire_history_.data() + kDesireLen * (kDesireHistoryTicks - 1),
                    desire_.data(), sizeof(float) * kDesireLen);
    }

    // 모델 입력 desire [25, 8]: 연속한 4틱마다 원소별 최댓값.
    void desire_input(float *out) const
    {
        for (int t = 0; t < kDesireInputTicks; ++t) {
            for (int d = 0; d < kDesireLen; ++d) {
                float value = desire_history_[(t * kDesirePool) * kDesireLen + d];
                for (int k = 1; k < kDesirePool; ++k)
                    value = std::max(value, desire_history_[(t * kDesirePool + k) * kDesireLen + d]);
                out[t * kDesireLen + d] = value;
            }
        }
    }

    // 모델 입력 features_buffer [24, 512]: 특징 이력의 0,4,...,92번 슬롯.
    void feature_input(float *out) const
    {
        for (int t = 0; t < kFeatureInputTicks; ++t)
            std::memcpy(out + t * kModelFeatureLen,
                        feature_history_.data() + (t * kFeatureStride) * kModelFeatureLen,
                        sizeof(float) * kModelFeatureLen);
    }

    // 실행 직후 raw 출력의 hidden_state를 특징 이력 마지막 슬롯에 넣는다.
    bool push_feature_history(const float *raw_output, size_t count)
    {
        if (count < kHiddenOffset + kModelFeatureLen) return false;
        std::memmove(feature_history_.data(), feature_history_.data() + kModelFeatureLen,
                     sizeof(float) * kModelFeatureLen * (kFeatureHistoryTicks - 1));
        std::memcpy(feature_history_.data() + kModelFeatureLen * (kFeatureHistoryTicks - 1),
                    raw_output + kHiddenOffset, sizeof(float) * kModelFeatureLen);
        return true;
    }

    const std::vector<float> &desire_history() const { return desire_history_; }
    const std::vector<float> &feature_history() const { return feature_history_; }
    const std::vector<float> &traffic_convention() const { return traffic_convention_; }

private:
    std::vector<float> desire_;
    std::vector<float> prev_desire_;
    std::vector<float> traffic_convention_;
    std::vector<float> desire_history_;
    std::vector<float> feature_history_;
};

/* 한 탑(road 또는 wide)의 최근 5프레임 이미지. 새 프레임은 newest_slot()에 직접
 * 워프해 넣고 commit()으로 한 칸 민다. 복사 없이 링 인덱스만 돈다. */
class ModelImageHistory
{
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
    void model_input(uint8_t *out) const
    {
        std::memcpy(out, oldest(), kFrameBytes);
        std::memcpy(out + kFrameBytes, newest(), kFrameBytes);
    }

private:
    // age 0 = 가장 오래된 칸, kFrames-1 = 최신. head_는 다음에 쓸 칸(= 가장 오래된 칸).
    const uint8_t *frame(int age) const
    {
        return frames_.data() + static_cast<size_t>((head_ + age) % kFrames) * kFrameBytes;
    }

    std::vector<uint8_t> frames_;
    int head_ = 0;
};

#endif
