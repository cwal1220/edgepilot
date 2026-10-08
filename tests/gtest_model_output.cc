/* supercombo(openpilot master) raw 출력 파서와 시간축 입력 규약(desire 펄스·풀링,
 * 특징 이력, 이미지 이력). SCODMP1 덤프(EDGEPILOT_RAW_DUMP)를 인자로 주면 테스트
 * 대신 첫 프레임을 파싱해 출력한다. */
#include "common/model_output.h"
#include "model/model_temporal.h"

#include "model/model_output_assembly.h"

#include <gtest/gtest.h>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

namespace {

using namespace model_output_layout;

/* openpilot master 출력 레이아웃 검사. */
TEST(ModelOutputParser, Master)
{
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
    // openpilot Meta: GAS_PRESS = meta[31:55:4], BRAKE_PRESS = meta[32:55:4](로짓). 2초 칸만 크게
    raw[kMetaOffset + kMetaGasPressIndex + 1 * kMetaPressStride] = 2.0f;
    raw[kMetaOffset + kMetaBrakePressIndex + 0 * kMetaPressStride] = 3.0f;
    raw[kMetaOffset + 33] = 4.0f;  // 같은 칸 줄의 왼쪽 깜빡이 예측: 페달로 읽히면 안 된다
    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    ParsedLeadPoint lead;
    float lead_prob = 0.0f;
    ASSERT_TRUE(parsed.valid);
    ASSERT_EQ(parsed.plan.best_index, 0);
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
    ASSERT_NEAR(parsed.meta.brake_press[0], 1.0f / (1.0f + std::exp(-3.0f)), 1e-6f);
    for (int i = 1; i < kMetaPressHorizons; ++i)
        ASSERT_NEAR(parsed.meta.brake_press[i], 0.5f, 1e-6f) << i;
    ASSERT_TRUE(parsed.has_pose);
    ASSERT_NEAR(parsed.pose.trans[2], 22.0f, 1e-6f);
    ASSERT_NEAR(parsed.pose.trans_std[0], 0.05f, 1e-6f);
    ASSERT_TRUE(ModelOutputParser::parse(std::vector<float>(kModelOutputFloats + 2, 0.0f)).valid)
        << "더 긴 버퍼는 받아준다";
    ASSERT_FALSE(ModelOutputParser::parse(std::vector<float>(100, 0.0f)).valid);
}

/* desire: rising-edge 펄스, 100틱 이력 밀기, 4틱 max-pool. */
TEST(ModelTemporal, Desire)
{
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
    ASSERT_EQ(state.traffic_convention(), (std::vector<float>{1.0f, 0.0f}));
}

/* 특징: hidden_state 512개가 마지막 슬롯에 들어가고, 모델 입력은 0,4,...,92번
 * 슬롯이라 이번 출력은 3프레임을 더 밀어야 입력 마지막 행에 나타난다. */
TEST(ModelTemporal, Features)
{
    static_assert(SupercomboTemporalState::kHiddenOffset == 1064, "hidden state offset moved");
    SupercomboTemporalState state;
    std::vector<float> raw(kModelOutputFloats, 0.0f), zeros(kModelOutputFloats, 0.0f);
    for (int i = 0; i < kModelFeatureLen; ++i)
        raw[SupercomboTemporalState::kHiddenOffset + i] = static_cast<float>(i + 1);
    ASSERT_TRUE(state.push_feature_history(raw.data(), raw.size()));
    const auto &features = state.feature_history();
    const size_t newest = (SupercomboTemporalState::kFeatureHistoryTicks - 1) * kModelFeatureLen;
    ASSERT_EQ(features[newest], 1.0f);
    ASSERT_EQ(features[newest + kModelFeatureLen - 1], 512.0f);

    std::vector<float> input(SupercomboTemporalState::kFeatureInputTicks * kModelFeatureLen);
    const size_t last_row = (SupercomboTemporalState::kFeatureInputTicks - 1) * kModelFeatureLen;
    for (int pushes = 0; pushes < 3; ++pushes) {
        state.feature_input(input.data());
        ASSERT_EQ(input[last_row], 0.0f) << pushes << "프레임 뒤에는 아직 입력에 없다";
        ASSERT_TRUE(state.push_feature_history(zeros.data(), zeros.size()));
    }
    state.feature_input(input.data());
    ASSERT_EQ(input[last_row], 1.0f);
    ASSERT_EQ(input[last_row + kModelFeatureLen - 1], 512.0f);
    ASSERT_FALSE(state.push_feature_history(raw.data(), 100)) << "짧은 raw 출력은 거부한다";
}

/* 이미지: 5프레임 링, 모델 입력은 t-4 다음 t. */
TEST(ModelTemporal, ImageHistory)
{
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

/* 헤드를 나눈 axmodel 출력(split_outputs.py)을 모으면 2576 레이아웃의 모든 칸이 정확히 한 번씩
 * 제자리에 온다. 각 출력에 "가야 할 칸 번호"를 채워 확인한다. */
TEST(ModelOutputParser, SplitOutputsReassembleLayout)
{
    using namespace model_output_assembly;
    std::vector<float> raw(kModelOutputFloats, -1.0f);
    std::vector<int> hits(kModelOutputFloats, 0);
    int total = 0;
    for (const Part &part : kParts) {
        std::vector<float> src(part.count);
        for (int i = 0; i < part.count; ++i) {
            int target = part.offset + i;
            if (part.piece == Piece::PlanMotion)
                target = part.offset + (i / kPlanMotionWidth) * model_output_assembly::kPlanWidth + i % kPlanMotionWidth;
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

template <typename T>
bool read_exact(std::ifstream &file, T *value)
{
    file.read(reinterpret_cast<char *>(value), sizeof(T));
    return file.gcount() == static_cast<std::streamsize>(sizeof(T));
}

} // namespace

// 인자가 없으면 테스트, SCODMP1 덤프 경로를 주면 첫 프레임을 파싱해 출력한다.
int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    if (argc == 1) return RUN_ALL_TESTS();
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " [SCODMP1 raw dump]\n";
        return 2;
    }

    std::ifstream file(argv[1], std::ios::binary);
    if (!file) {
        std::perror("open raw dump");
        return 1;
    }

    char magic[8]{};
    file.read(magic, sizeof(magic));
    if (file.gcount() != static_cast<std::streamsize>(sizeof(magic)) ||
        std::memcmp(magic, "SCODMP1", 7) != 0) {
        std::cerr << "bad raw dump magic\n";
        return 1;
    }

    uint32_t raw_size = 0;
    uint32_t frame_count = 0;
    if (!read_exact(file, &raw_size) || !read_exact(file, &frame_count) || raw_size == 0) {
        std::cerr << "bad raw dump header\n";
        return 1;
    }

    std::vector<float> raw(raw_size);
    file.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(raw.size() * sizeof(float)));
    if (file.gcount() != static_cast<std::streamsize>(raw.size() * sizeof(float))) {
        std::cerr << "short first frame\n";
        return 1;
    }

    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    if (!parsed.valid || !parsed.plan.valid || !parsed.has_pose) {
        std::cerr << "parser did not produce required plan/pose outputs\n";
        return 1;
    }

    ParsedLeadPoint lead;
    const bool have_lead = parsed.leads.primary(0, 0.0f, &lead);
    std::cout << "raw_size=" << raw_size
              << " frames=" << frame_count
              << " plan_best=" << parsed.plan.best_index
              << " plan_prob=" << parsed.plan.probability
              << " lane0_prob=" << parsed.lanes[0].probability
              << " pose_vx=" << parsed.pose.trans[0]
              << " lead_valid=" << (have_lead ? 1 : 0);
    if (have_lead)
        std::cout << " lead_x=" << lead.x << " lead_y=" << lead.y;
    std::cout << "\n";
    return 0;
}
