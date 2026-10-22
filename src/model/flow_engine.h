#pragma once

// 광류 작업(model/flow_tracker)의 하드웨어 면: 프레임마다 축소 단을 만들고, 측정 창 시작 프레임의 코너 응답을 내고,
// 점을 한 프레임씩 따라간다. 보드는 IVE·VPP(platform/maixcam2/maix_ive_flow의 IveFlow)이고, 호스트 시험은 가짜를
// 넣어 FlowTracker의 프레임 잇기와 버림을 본다.

#include <cstdint>

class FlowEngine {
public:
    static constexpr int kBandTop = 88;  // 코너를 찾는 1/2 단의 띠(하늘·보닛 제외, 16의 배수 높이)
    static constexpr int kBandRows = 240;
    static constexpr int kMaxPoints = 500;  // track 한 번에 따라가는 점의 한계(IVE LK·CornerDet)

    virtual ~FlowEngine() = default;

    // NV12 프레임(Y 평면 y_phys, 행 간격은 폭)으로 축소 단을 만든다. 성공하면 이 프레임이 '현재', 앞의
    // 현재 프레임이 '이전'이 된다. 원본 단은 복사하지 않으므로 두 프레임의 버퍼는 다음 track()이 끝날 때까지
    // 그대로여야 한다(링 슬롯이면 호출자가 seqlock으로 확인한다).
    virtual bool push(unsigned long long y_phys) = 0;
    // 현재 프레임 1/2 단 띠(kBandRows행, 폭 width/2)의 코너 응답. 다음 호출까지 유효하다. 실패하면 nullptr.
    virtual const int16_t *corner_response() = 0;
    // 이전 프레임의 점 n개(원본 좌표)를 현재 프레임으로 따라간다. backward면 거꾸로 현재 프레임의 점을 이전
    // 프레임으로 따라간다(앞뒤 검사). ok[i]가 0이면 놓쳤거나 한계를 넘었다. 하드웨어 호출이 실패하면 false.
    virtual bool track(const float *xy, int n, float *out_xy, uint8_t *ok, bool backward = false) = 0;
    virtual int width() const = 0;
};
