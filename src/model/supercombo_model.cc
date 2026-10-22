#include "model/supercombo_model.h"

#include "model/model_output_assembly.h"

#include "common/utils_process.h"
#include "common/utils_time.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <string>

namespace {

const char *const kInputNames[] = {
    "input_imgs", "big_input_imgs", "desire", "features_buffer", "traffic_convention",
};

std::string shape_string(const std::vector<int> &shape) {
    std::string s = "[";
    for (size_t i = 0; i < shape.size(); ++i) s += (i ? "," : "") + std::to_string(shape[i]);
    return s + "]";
}

}  // namespace

SupercomboModel::SupercomboModel(const char *model_file, const AppConfig &config)
    : session_(model_file),
      profile_(config.profile),
      input_transform_(config, ModelFrame::MedModel),
      big_input_transform_(config, ModelFrame::SmallBigModel),
      gdc_(static_cast<int>(config.nv12_width), static_cast<int>(config.nv12_height)) {
    // master 코어 계약을 강제한다. 다른 axmodel은 조용히 오해석되지 않도록
    // 이름·모양·자료형이 하나라도 다르면 거부한다.
    const struct {
        size_t count;
        AxEngineSession::DataType dtype;
        size_t element_bytes;
    } expected[kInputCount] = {
        {2 * ModelImageHistory::kFrameBytes, AxEngineSession::DataType::UInt8, 1},
        {2 * ModelImageHistory::kFrameBytes, AxEngineSession::DataType::UInt8, 1},
        {SupercomboTemporalState::kDesireInputTicks * kDesireLen, AxEngineSession::DataType::Float32, 4},
        {SupercomboTemporalState::kFeatureInputTicks * kModelFeatureLen, AxEngineSession::DataType::Float32, 4},
        {2, AxEngineSession::DataType::Float32, 4},
    };
    if (session_.inputs().size() != kInputCount)
        throw std::runtime_error(
            "not an openpilot master supercombo axmodel: " + std::to_string(session_.inputs().size()) + " inputs");
    for (int i = 0; i < kInputCount; ++i) {
        index_[i] = session_.input_index(kInputNames[i]);
        if (index_[i] < 0) throw std::runtime_error(std::string("axmodel lacks input ") + kInputNames[i]);
        const auto &t = session_.inputs()[index_[i]];
        // 패딩된 버퍼(nSize > 원소 수 x 크기)는 연속 배열로 채울 수 없으므로 거부한다.
        if (t.count() != expected[i].count || t.dtype != expected[i].dtype ||
            t.bytes != expected[i].count * expected[i].element_bytes)
            throw std::runtime_error(std::string("axmodel input ") + kInputNames[i] + " is " + shape_string(t.shape) +
                                     ", not the master contract");
    }
    // 출력은 헤드를 나눈 출력들(split_outputs.py)이다. 이름으로 찾아 제자리에 모은다.
    using model_output_assembly::kParts;
    if (session_.outputs().size() != static_cast<size_t>(model_output_assembly::kPartCount))
        throw std::runtime_error("axmodel has " + std::to_string(session_.outputs().size()) + " outputs, expected " +
                                 std::to_string(model_output_assembly::kPartCount));
    for (int p = 0; p < model_output_assembly::kPartCount; ++p) {
        const auto &part = kParts[p];
        int found = -1;
        for (size_t i = 0; i < session_.outputs().size(); ++i)
            if (session_.outputs()[i].name == part.name) found = static_cast<int>(i);
        if (found < 0) throw std::runtime_error(std::string("axmodel lacks output ") + part.name);
        const auto &out = session_.outputs()[found];
        if (out.count() != static_cast<size_t>(part.count) || out.dtype != AxEngineSession::DataType::Float32)
            throw std::runtime_error(std::string("axmodel output ") + part.name + " is " + shape_string(out.shape) +
                                     ", expected " + std::to_string(part.count) + " float");
        output_parts_[p] = found;
    }

    // traffic convention은 이 차에서 상수(우측 통행 {1, 0})라 한 번만 쓴다.
    const float right_hand_traffic[2] = {1.0f, 0.0f};
    std::memcpy(session_.input<float>(index_[kTraffic]), right_hand_traffic, sizeof(right_hand_traffic));
    std::fprintf(stderr, "Supercombo openpilot-master axmodel (pulsar2 %s), warp on GDC (AX IVPS)\n",
                 session_.tools_version().c_str());
}

void SupercomboModel::set_desire(int desire) { temporal_.set_desire(desire); }

void SupercomboModel::set_camera_mount(float offset_m, float height_m) {
    input_transform_.set_camera_mount(offset_m, height_m);
    big_input_transform_.set_camera_mount(offset_m, height_m);
}

void SupercomboModel::set_input_calibration(const float rpy[3]) {
    input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
    big_input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
}

uint64_t SupercomboModel::profile_stamp() const { return profile_ ? monotonic_now_ns() : 0; }

bool SupercomboModel::run_frame_phys(unsigned long long src_phys, std::vector<float> &raw_output,
                                     const std::function<bool()> &source_still_valid) {
    const uint64_t warp_start_ns = profile_stamp();
    float med[9], sbig[9];
    input_transform_.projection_matrix(med);
    big_input_transform_.projection_matrix(sbig);
    if (!gdc_.warp(src_phys, med, sbig, road_history_.newest_slot(), wide_history_.newest_slot()) ||
        !source_still_valid())
        return false;
    road_history_.commit();
    wide_history_.commit();
    return infer(warp_start_ns, profile_stamp(), raw_output);
}

bool SupercomboModel::infer(uint64_t warp_start_ns, uint64_t warp_end_ns, std::vector<float> &raw_output) {
    road_history_.model_input(session_.input<uint8_t>(index_[kRoad]));
    wide_history_.model_input(session_.input<uint8_t>(index_[kWide]));
    temporal_.push_desire_pulse();
    temporal_.desire_input(session_.input<float>(index_[kDesire]));
    temporal_.feature_input(session_.input<float>(index_[kFeatures]));
    const uint64_t inputs_end_ns = profile_stamp();

    if (!session_.run()) {
        // 이미지·desire 큐는 이미 한 칸 나갔으므로 특징 큐도 빈 칸으로 한 칸 밀어
        // 세 큐의 시점을 맞춘다.
        const std::vector<float> empty(kModelOutputFloats, 0.0f);
        temporal_.push_feature_history(empty.data());
        return false;
    }
    const uint64_t npu_end_ns = profile_stamp();

    raw_output.resize(kModelOutputFloats);  // 끝의 패딩 두 칸은 처음 늘릴 때의 0 그대로다
    for (int i = 0; i < model_output_assembly::kPartCount; ++i)
        model_output_assembly::place(model_output_assembly::kParts[i], session_.output<float>(output_parts_[i]),
                                     raw_output.data());
    temporal_.push_feature_history(raw_output.data());
    const uint64_t outputs_end_ns = profile_stamp();

    if (profile_) {
        const uint64_t stamps[] = {warp_start_ns, warp_end_ns, inputs_end_ns, npu_end_ns, outputs_end_ns};
        for (int i = 0; i < 4; ++i) profile_ms_[i] += (stamps[i + 1] - stamps[i]) / 1e6;
        if (++profile_frames_ == kProfileFrames) {
            std::fprintf(stderr, "profile ms: warp %.2f inputs %.2f npu %.2f outputs %.2f\n",
                         profile_ms_[0] / kProfileFrames, profile_ms_[1] / kProfileFrames,
                         profile_ms_[2] / kProfileFrames, profile_ms_[3] / kProfileFrames);
            profile_frames_ = 0;
            std::fill(std::begin(profile_ms_), std::end(profile_ms_), 0.0);
        }
    }
    return true;
}
