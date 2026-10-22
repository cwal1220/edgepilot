// supercombo(openpilot master) raw 출력 파서와 시간축 입력 규약(desire 펄스·풀링,
// 특징 이력, 이미지 이력).
#include "common/model_output.h"
#include "model/model_temporal.h"

#include "model/model_output_assembly.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

using namespace model_output_layout;

// openpilot master 출력 레이아웃 검사.
TEST(ModelOutputParser, Master) {
    std::vector<float> raw(kModelOutputFloats, 0.0f);

    raw[kPlanOffset + 7 * kPlanWidth + 0] = 23.0f;
    raw[kPlanOffset + 7 * kPlanWidth + 1] = 0.75f;

    const int lane = 1;
    raw[kLaneOffset + lane * kTrajectorySize * 2 + 5 * 2] = -1.75f;
    raw[kLaneProbOffset + lane * 2 + 1] = 2.0f;
    raw[kLaneOffset + kLaneLineSize + lane * kTrajectorySize * 2] = std::log(0.2f);
    // 시간 오프셋 1(2 s)의 lead만 존재
    raw[kLeadOffset + 1 * kLeadTrajLen * kLeadElementSize] = 42.0f;
    raw[kLeadProbOffset + 0] = -3.0f;
    raw[kLeadProbOffset + 1] = 3.0f;

    raw[kDesireStateOffset + 3] = 5.0f;
    for (int i = 0; i < 3; ++i) {
        raw[kPoseOffset + i] = 20.0f + i;
        raw[kPoseOffset + 6 + i] = std::log(0.05f);
    }
    // openpilot Meta: GAS_PRESS = meta[31:55:4](로짓). 2초 칸만 크게
    raw[kMetaOffset + kMetaGasPressIndex + 1 * kMetaPressStride] = 2.0f;
    raw[kMetaOffset + 33] = 4.0f;  // 같은 칸 줄의 왼쪽 깜빡이 예측: 페달로 읽히면 안 된다
    const ParsedModelOutput parsed = ModelOutputParser::parse(raw.data());
    ParsedLeadPoint lead;
    float lead_prob = 0.0f;
    ASSERT_TRUE(parsed.valid);
    ASSERT_NEAR(parsed.plan.points[7].x, 23.0f, 1e-6f);
    ASSERT_NEAR(parsed.plan.points[7].y, 0.75f, 1e-6f);
    ASSERT_NEAR(parsed.lanes[1].points[5].y, -1.75f, 1e-6f);
    ASSERT_NEAR(parsed.lanes[1].probability, 1.0f / (1.0f + std::exp(-2.0f)), 1e-6f);
    ASSERT_NEAR(parsed.lanes[1].std, 0.2f, 1e-6f);
    ASSERT_FALSE(parsed.leads.primary(0, 0.5f, &lead)) << "0 s lead는 확률이 낮다";
    ASSERT_TRUE(parsed.leads.primary(1, 0.5f, &lead, &lead_prob));
    ASSERT_NEAR(lead.x, 42.0f, 1e-6f);
    ASSERT_GT(lead_prob, 0.9f);
    ASSERT_GT(parsed.meta.desire_state[3], parsed.meta.desire_state[0]);
    ASSERT_NEAR(parsed.meta.gas_press[1], 1.0f / (1.0f + std::exp(-2.0f)), 1e-6f);
    ASSERT_NEAR(parsed.meta.gas_press[0], 0.5f, 1e-6f) << "로짓 0은 확률 0.5";
    ASSERT_NEAR(parsed.pose.trans[2], 22.0f, 1e-6f);
    ASSERT_NEAR(parsed.pose.trans_std[0], 0.05f, 1e-6f);
}

// 회전 desire(1, 2)는 회전이 끝나면(다른 desire가 오면) 이력에서 지운다. 차선 변경 펄스는 상류처럼 남긴다.
TEST(ModelTemporal, TurnDesireClearsWhenTheTurnEnds) {
    SupercomboTemporalState state;
    const auto &desire = state.desire_history();
    const size_t last = (SupercomboTemporalState::kDesireHistoryTicks - 1) * kDesireLen;
    state.set_desire(3);
    state.push_desire_pulse();
    state.set_desire(1);
    state.push_desire_pulse();
    ASSERT_EQ(desire[last + 1], 1.0f);
    for (int i = 0; i < 10; ++i) {
        state.set_desire(1);
        state.push_desire_pulse();
    }
    ASSERT_EQ(desire[last - 10 * kDesireLen + 1], 1.0f) << "회전 중에는 펄스가 이력에 남는다";
    state.set_desire(0);
    state.push_desire_pulse();
    float turn = 0.0f, lane_change = 0.0f;
    for (int t = 0; t < SupercomboTemporalState::kDesireHistoryTicks; ++t) {
        turn = std::max({turn, desire[t * kDesireLen + 1], desire[t * kDesireLen + 2]});
        lane_change = std::max(lane_change, desire[t * kDesireLen + 3]);
    }
    ASSERT_EQ(turn, 0.0f) << "회전을 마치면 이력의 회전 펄스를 지운다";
    ASSERT_EQ(lane_change, 1.0f) << "차선 변경 펄스는 남긴다";
}

// 정해진 desire 수열 뒤 이력에 남는 펄스(최신 틱부터 센 칸, desire 번호). make_m2_data.py의 desire_step도 같은 값을
// 낸다(check_recording_reader.py가 같은 수열과 기준값으로 본다). 둘이 갈리면 eval 기준과 양자화 표본이 보드 입력과
// 달라진다.
TEST(ModelTemporal, DesireSequenceMatchesTheToolQueue) {
    SupercomboTemporalState state;
    for (int desire : {0, 3, 3, 0, 1, 1, 1, 0, 0, 2, 2, 3, 0, 1, 1, 4, 4, 0, 1, 1, 2}) {
        state.set_desire(desire);
        state.push_desire_pulse();
    }
    const auto &history = state.desire_history();
    std::vector<std::pair<int, int>> pulses;
    for (int back = 0; back < SupercomboTemporalState::kDesireHistoryTicks; ++back)
        for (int i = 0; i < kDesireLen; ++i)
            if (history[(SupercomboTemporalState::kDesireHistoryTicks - 1 - back) * kDesireLen + i] != 0.0f)
                pulses.emplace_back(back, i);
    EXPECT_EQ(pulses, (std::vector<std::pair<int, int>>{{0, 2}, {2, 1}, {5, 4}, {9, 3}, {19, 3}}));
}

// desire: rising-edge 펄스, 100틱 이력 밀기, 4틱 max-pool.
TEST(ModelTemporal, Desire) {
    SupercomboTemporalState state;
    const auto &desire = state.desire_history();
    const size_t last = (SupercomboTemporalState::kDesireHistoryTicks - 1) * kDesireLen;
    state.set_desire(3);
    state.push_desire_pulse();
    // 새 desire는 제 칸에 펄스가 뜨고 0번 칸에는 뜨지 않는다
    ASSERT_EQ(desire[last + 3], 1.0f);
    ASSERT_EQ(desire[last + 0], 0.0f);
    state.set_desire(3);
    state.push_desire_pulse();
    // 유지된 desire는 펄스가 한 번만 뜨고 그 펄스는 한 틱 뒤로 밀린다
    ASSERT_EQ(desire[last + 3], 0.0f);
    ASSERT_EQ(desire[last - kDesireLen + 3], 1.0f);

    // 펄스는 마지막 풀(틱 96..99)에 들어 있고, 4틱 더 밀면 한 풀 앞으로 간다
    std::vector<float> pooled(SupercomboTemporalState::kDesireInputTicks * kDesireLen);
    state.desire_input(pooled.data());
    const size_t last_pool = (SupercomboTemporalState::kDesireInputTicks - 1) * kDesireLen;
    ASSERT_EQ(pooled[last_pool + 3], 1.0f);
    ASSERT_EQ(pooled[last_pool - kDesireLen + 3], 0.0f);
    for (int i = 0; i < 4; ++i) {
        state.set_desire(3);
        state.push_desire_pulse();
    }
    state.desire_input(pooled.data());
    ASSERT_EQ(pooled[last_pool + 3], 0.0f);
    ASSERT_EQ(pooled[last_pool - kDesireLen + 3], 1.0f);
}

// 특징: hidden_state 512개가 마지막 슬롯에 들어가고, 모델 입력은 0,4,...,92번
// 슬롯이라 이번 출력은 3프레임을 더 밀어야 입력 마지막 행에 나타난다.
TEST(ModelTemporal, Features) {
    static_assert(SupercomboTemporalState::kHiddenOffset == 1064, "hidden state offset moved");
    SupercomboTemporalState state;
    std::vector<float> raw(kModelOutputFloats, 0.0f), zeros(kModelOutputFloats, 0.0f);
    for (int i = 0; i < kModelFeatureLen; ++i)
        raw[SupercomboTemporalState::kHiddenOffset + i] = static_cast<float>(i + 1);
    state.push_feature_history(raw.data());
    const auto &features = state.feature_history();
    const size_t newest = (SupercomboTemporalState::kFeatureHistoryTicks - 1) * kModelFeatureLen;
    ASSERT_EQ(features[newest], 1.0f);
    ASSERT_EQ(features[newest + kModelFeatureLen - 1], 512.0f);

    std::vector<float> input(SupercomboTemporalState::kFeatureInputTicks * kModelFeatureLen);
    const size_t last_row = (SupercomboTemporalState::kFeatureInputTicks - 1) * kModelFeatureLen;
    for (int pushes = 0; pushes < 3; ++pushes) {
        state.feature_input(input.data());
        ASSERT_EQ(input[last_row], 0.0f) << pushes << "프레임 뒤에는 아직 입력에 없다";
        state.push_feature_history(zeros.data());
    }
    state.feature_input(input.data());
    ASSERT_EQ(input[last_row], 1.0f);
    ASSERT_EQ(input[last_row + kModelFeatureLen - 1], 512.0f);

    // 이력은 96칸 링이다: 여러 바퀴 돈 뒤에도 입력 행은 오래된 것부터 4칸 간격이다
    for (int k = 1; k <= 200; ++k) {
        std::fill(raw.begin() + SupercomboTemporalState::kHiddenOffset,
                  raw.begin() + SupercomboTemporalState::kHiddenOffset + kModelFeatureLen, static_cast<float>(k));
        state.push_feature_history(raw.data());
    }
    state.feature_input(input.data());
    for (int t = 0; t < SupercomboTemporalState::kFeatureInputTicks; ++t)
        ASSERT_EQ(input[t * kModelFeatureLen], static_cast<float>(105 + 4 * t)) << "행 " << t;
    EXPECT_EQ(state.feature_history().back(), 200.0f) << "시간 순 사본의 마지막은 최신";
}

// 이미지: 5프레임 링, 모델 입력은 t-4 다음 t.
TEST(ModelTemporal, ImageHistory) {
    ModelImageHistory history;
    std::vector<uint8_t> input(2 * ModelImageHistory::kFrameBytes);
    for (int frame = 1; frame <= 7; ++frame) {
        std::memset(history.newest_slot(), frame, ModelImageHistory::kFrameBytes);
        history.commit();
    }
    history.model_input(input.data());
    ASSERT_EQ(input[0], 3) << "7프레임 뒤 가장 오래된 칸은 3번";
    ASSERT_EQ(input[ModelImageHistory::kFrameBytes - 1], 3);
    ASSERT_EQ(input[ModelImageHistory::kFrameBytes], 7);
    ASSERT_EQ(input.back(), 7);
}

// 헤드를 나눈 axmodel 출력(split_outputs.py)을 모으면 2576 레이아웃의 모든 칸이 정확히 한 번씩
// 제자리에 온다. 각 출력에 "가야 할 칸 번호"를 채워 확인한다.
TEST(ModelOutputParser, SplitOutputsReassembleLayout) {
    using namespace model_output_assembly;
    std::vector<float> raw(kModelOutputFloats, -1.0f);
    std::vector<int> hits(kModelOutputFloats, 0);
    int total = 0;
    for (const Part &part : kParts) {
        std::vector<float> src(part.count);
        for (int i = 0; i < part.count; ++i) {
            int target = part.offset + i;
            if (part.piece == Piece::PlanMotion)
                target =
                    part.offset + (i / kPlanMotionWidth) * model_output_assembly::kPlanWidth + i % kPlanMotionWidth;
            else if (part.piece == Piece::PlanOrient)
                target = part.offset + (i / kPlanOrientWidth) * model_output_assembly::kPlanWidth + kPlanMotionWidth +
                         i % kPlanOrientWidth;
            src[i] = static_cast<float>(target);
            ++hits[target];
        }
        place(part, src.data(), raw.data());
        total += part.count;
    }
    EXPECT_EQ(total, kModelOutputFloats - 2) << "패딩 두 칸을 뺀 전부";
    for (int i = 0; i < kModelOutputFloats - 2; ++i) {
        ASSERT_EQ(hits[i], 1) << "칸 " << i;
        ASSERT_EQ(raw[i], static_cast<float>(i)) << "칸 " << i;
    }
    // plan yaw(knot 20, 열 11)는 방향 출력에서 온다
    EXPECT_EQ(raw[model_output_layout::kPlanOffset + 20 * 15 + 11],
              static_cast<float>(model_output_layout::kPlanOffset + 20 * 15 + 11));
}

}  // namespace
