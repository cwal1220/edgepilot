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

std::string shape_string(const std::vector<int> &shape)
{
    std::string s = "[";
    for (size_t i = 0; i < shape.size(); ++i)
        s += (i ? "," : "") + std::to_string(shape[i]);
    return s + "]";
}

} // namespace

SupercomboModel::SupercomboModel(const char *model_file, const AppConfig &config)
    : session_(model_file),
      profile_(config.profile),
      input_transform_(config, ModelFrame::MedModel),
      big_input_transform_(config, ModelFrame::SmallBigModel)
{
    /* master 코어 계약을 강제한다. 다른 axmodel은 조용히 오해석되지 않도록
     * 이름·모양·자료형이 하나라도 다르면 거부한다. */
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
        throw std::runtime_error("not an openpilot master supercombo axmodel: " +
                                 std::to_string(session_.inputs().size()) + " inputs");
    for (int i = 0; i < kInputCount; ++i) {
        index_[i] = session_.input_index(kInputNames[i]);
        if (index_[i] < 0) throw std::runtime_error(std::string("axmodel lacks input ") + kInputNames[i]);
        const auto &t = session_.inputs()[index_[i]];
        // 패딩된 버퍼(nSize > 원소 수 x 크기)는 연속 배열로 채울 수 없으므로 거부한다.
        if (t.count() != expected[i].count || t.dtype != expected[i].dtype ||
            t.bytes != expected[i].count * expected[i].element_bytes)
            throw std::runtime_error(std::string("axmodel input ") + kInputNames[i] + " is " +
                                     shape_string(t.shape) + ", not the master contract");
    }
    /* 출력은 두 가지를 받는다: 예전의 단일 [1,2576](모든 값이 한 눈금으로 양자화됨), 또는
     * 헤드를 나눈 출력들(split_outputs.py, 이름으로 찾아 제자리에 모은다). */
    if (session_.outputs().size() == 1) {
        const auto &out = session_.outputs()[0];
        if (out.count() != kModelOutputFloats || out.dtype != AxEngineSession::DataType::Float32)
            throw std::runtime_error("axmodel output is " + shape_string(out.shape) +
                                     ", expected [1," + std::to_string(kModelOutputFloats) + "] float");
    } else {
        using model_output_assembly::kParts;
        if (session_.outputs().size() != static_cast<size_t>(model_output_assembly::kPartCount))
            throw std::runtime_error("axmodel has " + std::to_string(session_.outputs().size()) +
                                     " outputs, expected 1 or " +
                                     std::to_string(model_output_assembly::kPartCount));
        for (const auto &part : kParts) {
            int found = -1;
            for (size_t i = 0; i < session_.outputs().size(); ++i)
                if (session_.outputs()[i].name == part.name) found = static_cast<int>(i);
            if (found < 0) throw std::runtime_error(std::string("axmodel lacks output ") + part.name);
            const auto &out = session_.outputs()[found];
            if (out.count() != static_cast<size_t>(part.count) ||
                out.dtype != AxEngineSession::DataType::Float32)
                throw std::runtime_error(std::string("axmodel output ") + part.name + " is " +
                                         shape_string(out.shape) + ", expected " +
                                         std::to_string(part.count) + " float");
            output_parts_.push_back(found);
        }
    }

    // traffic convention은 이 차에서 상수라 한 번만 쓴다.
    std::memcpy(session_.input<float>(index_[kTraffic]), temporal_.traffic_convention().data(),
                sizeof(float) * 2);
    std::fprintf(stderr, "Supercombo openpilot-master axmodel (pulsar2 %s)\n",
                 session_.tools_version().c_str());
    if (!env_flag("EDGEPILOT_WARP_CPU", false)) {
        try {
            gdc_.reset(new GdcWarp(static_cast<int>(config.nv12_width), static_cast<int>(config.nv12_height)));
            std::fprintf(stderr, "Supercombo warp backend=gdc (AX IVPS)\n");
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Supercombo warp backend=cpu (gdc unavailable: %s)\n", e.what());
        }
    }
}

SupercomboModel::~SupercomboModel()
{
    {
        std::lock_guard<std::mutex> lock(wide_mutex_);
        wide_stop_ = true;
    }
    wide_cv_.notify_all();
    if (wide_thread_.joinable()) wide_thread_.join();
}

void SupercomboModel::wide_worker()
{
    uint64_t served = 0;
    for (;;) {
        const uint8_t *src;
        int w, h;
        {
            std::unique_lock<std::mutex> lock(wide_mutex_);
            wide_cv_.wait(lock, [&] { return wide_stop_ || wide_request_ != served; });
            if (wide_stop_) return;
            served = wide_request_;
            src = wide_src_;
            w = wide_w_;
            h = wide_h_;
        }
        bool ok = true;
        try {
            big_input_transform_.nv12_to_yuv6_warped(src, w, h, wide_history_.newest_slot());
        } catch (const std::exception &e) {
            // 스레드 밖으로 새면 std::terminate. 요청한 쪽이 실패로 받게 한다.
            std::fprintf(stderr, "Supercombo wide warp failed: %s\n", e.what());
            ok = false;
        }
        {
            std::lock_guard<std::mutex> lock(wide_mutex_);
            wide_ok_ = ok;
            wide_done_ = served;
        }
        wide_cv_.notify_all();
    }
}

void SupercomboModel::set_desire(int desire)
{
    temporal_.set_desire(desire);
}

void SupercomboModel::set_chroma_vu(bool vu)
{
    chroma_vu_ = vu;
    input_transform_.set_chroma_vu(vu);
    big_input_transform_.set_chroma_vu(vu);
}

void SupercomboModel::set_camera_mount(float offset_m, float height_m)
{
    input_transform_.set_camera_mount(offset_m, height_m);
    big_input_transform_.set_camera_mount(offset_m, height_m);
}

void SupercomboModel::set_input_calibration(const float rpy[3])
{
    input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
    big_input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
}

uint8_t *SupercomboModel::input_buffer(int src_w, int src_h)
{
    if (!gdc_ || chroma_vu_ || src_w != gdc_->src_width() || src_h != gdc_->src_height()) return nullptr;
    return gdc_->source();
}

bool SupercomboModel::run_frame_preloaded(int src_w, int src_h, std::vector<float> &raw_output)
{
    if (!input_buffer(src_w, src_h)) return false;
    const uint64_t t0 = monotonic_now_ns();
    float med[9], sbig[9];
    input_transform_.projection_matrix(med);
    big_input_transform_.projection_matrix(sbig);
    if (!gdc_->warp(med, sbig, road_history_.newest_slot(), wide_history_.newest_slot())) return false;
    road_history_.commit();
    wide_history_.commit();
    return infer(t0, monotonic_now_ns(), raw_output);
}

bool SupercomboModel::run_frame_phys(unsigned long long src_phys, int src_w, int src_h,
                                     std::vector<float> &raw_output,
                                     const std::function<bool()> &source_still_valid)
{
    if (!input_buffer(src_w, src_h)) return false;
    const uint64_t t0 = monotonic_now_ns();
    float med[9], sbig[9];
    input_transform_.projection_matrix(med);
    big_input_transform_.projection_matrix(sbig);
    if (!gdc_->warp_phys(src_phys, med, sbig, road_history_.newest_slot(), wide_history_.newest_slot()) ||
        !source_still_valid())
        return false;
    road_history_.commit();
    wide_history_.commit();
    return infer(t0, monotonic_now_ns(), raw_output);
}

bool SupercomboModel::run_frame_nv12(const uint8_t *nv12, int src_w, int src_h,
                                     std::vector<float> &raw_output)
{
    if (uint8_t *buffer = input_buffer(src_w, src_h)) {
        std::memcpy(buffer, nv12, static_cast<size_t>(src_w) * src_h * 3 / 2);
        return run_frame_preloaded(src_w, src_h, raw_output);
    }
    // GDC를 쓸 수 없는 프레임(예: NV21)이 처음 오면 그때 wide 워프 스레드를 띄운다.
    if (!wide_thread_.joinable()) wide_thread_ = std::thread(&SupercomboModel::wide_worker, this);
    const uint64_t t0 = monotonic_now_ns();
    uint64_t request;
    {
        std::lock_guard<std::mutex> lock(wide_mutex_);
        wide_src_ = nv12;
        wide_w_ = src_w;
        wide_h_ = src_h;
        request = ++wide_request_;
    }
    wide_cv_.notify_all();
    bool road_ok = true;
    try {
        input_transform_.nv12_to_yuv6_warped(nv12, src_w, src_h, road_history_.newest_slot());
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Supercombo road warp failed: %s\n", e.what());
        road_ok = false;
    }
    bool wide_ok;
    {
        std::unique_lock<std::mutex> lock(wide_mutex_);
        wide_cv_.wait(lock, [&] { return wide_done_ == request; });
        wide_ok = wide_ok_;
    }
    if (!road_ok || !wide_ok) return false;
    road_history_.commit();
    wide_history_.commit();
    return infer(t0, monotonic_now_ns(), raw_output);
}

bool SupercomboModel::infer(uint64_t t0, uint64_t t1, std::vector<float> &raw_output)
{
    road_history_.model_input(session_.input<uint8_t>(index_[kRoad]));
    wide_history_.model_input(session_.input<uint8_t>(index_[kWide]));
    temporal_.push_desire_pulse();
    temporal_.desire_input(session_.input<float>(index_[kDesire]));
    temporal_.feature_input(session_.input<float>(index_[kFeatures]));
    const uint64_t t2 = monotonic_now_ns();

    if (!session_.run()) {
        /* 이미지·desire 큐는 이미 한 칸 나갔으므로 특징 큐도 빈 칸으로 한 칸 밀어
         * 세 큐의 시점을 맞춘다. */
        std::vector<float> empty(kModelOutputFloats, 0.0f);
        temporal_.push_feature_history(empty.data(), empty.size());
        return false;
    }
    const uint64_t t3 = monotonic_now_ns();

    raw_output.resize(kModelOutputFloats);
    if (output_parts_.empty()) {
        std::memcpy(raw_output.data(), session_.output<float>(0), sizeof(float) * kModelOutputFloats);
    } else {
        std::fill(raw_output.begin(), raw_output.end(), 0.0f);  // 끝의 패딩 두 칸
        for (int i = 0; i < model_output_assembly::kPartCount; ++i)
            model_output_assembly::place(model_output_assembly::kParts[i],
                                         session_.output<float>(output_parts_[i]), raw_output.data());
    }
    temporal_.push_feature_history(raw_output.data(), raw_output.size());
    const uint64_t t4 = monotonic_now_ns();

    if (profile_) {
        const uint64_t stamps[] = {t0, t1, t2, t3, t4};
        for (int i = 0; i < 4; ++i) profile_ms_[i] += (stamps[i + 1] - stamps[i]) / 1e6;
        if (++profile_frames_ == 30) {
            std::fprintf(stderr, "\nprofile ms: warp %.2f inputs %.2f npu %.2f outputs %.2f\n",
                         profile_ms_[0] / 30, profile_ms_[1] / 30, profile_ms_[2] / 30,
                         profile_ms_[3] / 30);
            profile_frames_ = 0;
            std::fill(std::begin(profile_ms_), std::end(profile_ms_), 0.0);
        }
    }
    return true;
}
