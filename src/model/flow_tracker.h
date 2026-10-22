#pragma once

// 광류 작업의 한 프레임(호스트에서도 빌드한다). FlowTracker가 프레임마다 엔진(model/flow_engine)으로 축소 단을 한
// 번 만들고, 광류 보정의 측정 창(model/flow_calibration)과 앞차 확대율의 창(model/lead_flow)의 점을 한 번의
// LK로 함께 따라간다. 앞차 점은 앞뒤 검사를 위해 거꾸로 한 번 더 따라간다. 보정은 차가 곧고 빠를 때, 앞차는
// 모델 앞차가 있을 때만 일하고, 둘 다 없으면 엔진을 쓰지 않는다. 카메라 frame_id가 이어지지 않거나 링 슬롯이
// 덮어써지면 그 걸음을 버린다. modeld는 FlowWorker(model/flow_worker)의 스레드에서 돌린다.
// docs/flow-calibration.md.

#include "model/flow_calibration.h"
#include "model/flow_engine.h"
#include "model/lead_flow.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

// 카메라 프레임 하나와 그때의 조건
struct FlowFrame {
    // NV12/NV21 프레임의 Y 평면(CMM, 행 간격 stride, UV는 바로 뒤). 원본 단은 복사하지 않으므로 다음 프레임의
    // LK가 끝날 때까지 그대로여야 한다. still_valid가 있으면 축소 단을 만든 뒤와 LK 뒤에 불러(이전 프레임
    // 것도), 그동안 링 슬롯이 덮어써졌으면 그 걸음을 버린다.
    unsigned long long y_phys = 0;
    const uint8_t *y_virt = nullptr;  // 같은 평면의 CPU 매핑(앞차 점·템플릿). 없으면 앞차를 따라가지 않는다
    // y_virt가 캐시 매핑이면 CPU가 [offset, offset + bytes)를 읽기 전에 부른다(보드: 캐시 무효화)
    std::function<void(size_t offset, size_t bytes)> invalidate;
    int stride = 0;
    uint64_t frame_id = 0;
    double t_s = 0.0;    // 촬영 시각(s)
    FlowGateInput gate;  // 광류 보정의 곧은 길 조건
    LeadFlowInput lead;  // 직전 모델 출력의 앞차와 워프에 쓴 보정
    std::function<bool()> still_valid;
};

class FlowTracker {
public:
    // engine은 보드에서 IveFlow(만들 수 없으면 std::runtime_error), 시험에서는 가짜다.
    FlowTracker(const FlowIntrinsics &k, int width, int height, std::unique_ptr<FlowEngine> engine);
    ~FlowTracker();

    void update(const FlowFrame &f);
    const FlowCalibrator &calibrator() const { return calibrator_; }
    LeadFlowState lead_state() const { return lead_.state(); }
    // 저장해 둔 블록으로 시작한다(FlowCalibrator::restore). 진행 중인 창은 그대로다.
    bool restore(const FlowBlocks &blocks) { return calibrator_.restore(blocks); }
    // 보정 초기화: 블록과 진행 중인 창을 버린다(앞차 창은 그대로다).
    void reset_calibration();

    struct Stats {
        uint64_t frames = 0;        // IVE로 처리한 프레임
        uint64_t windows_done = 0;  // 끝까지 간 보정 창
        uint64_t windows_lost = 0;  // 조건이 닫히거나 점·프레임을 잃어 버린 보정 창
        uint64_t rejected = 0;      // 끝까지 갔지만 풀이가 맞지 않은 보정 창
        uint64_t lead_frames = 0;   // 앞차 점을 따라간 프레임
        uint64_t lead_rates = 0;    // 그중 확대율을 낸 프레임
        double wall_ms = 0.0;       // 처리한 프레임의 벽시계 합
        double cpu_ms = 0.0;        // 같은 프레임의 스레드 CPU 합
    };
    const Stats &stats() const { return stats_; }

private:
    void start_window();
    void advance_window(uint64_t frame_id, int tracked);

    FlowIntrinsics k_;
    std::unique_ptr<FlowEngine> engine_;
    FlowCalibrator calibrator_;
    LeadFlowTracker lead_;
    Stats stats_;
    bool in_window_ = false;
    bool have_last_ = false;            // 직전 프레임을 넣었다(LK의 이전 프레임)
    std::function<bool()> last_valid_;  // 직전 프레임의 링 슬롯이 그대로인지
    uint64_t last_frame_ = 0;
    int steps_ = 0;
    int n_ = 0;
    std::vector<float> start_, cur_;  // 보정 창 시작 점, 지금 위치(원본 좌표)
    std::vector<uint8_t> alive_;
    std::vector<int> live_;                  // 이번 걸음에 따라간 보정 점의 번호
    std::vector<float> step_in_, step_out_;  // 보정 점 다음에 앞차 점
    std::vector<uint8_t> step_ok_;
    std::vector<float> back_;  // 앞차 점을 거꾸로 따라간 위치(직전 프레임)
    std::vector<uint8_t> back_ok_;
    std::vector<float> fit_a_, fit_b_;
};
