#pragma once

// 광류 작업(model/flow_tracker: 광류 보정과 앞차 확대율)의 하드웨어 부분(FlowEngine). 프레임마다 축소 단(1/2,
// 1/4)을 VPP 한 번으로 만들고(modeld가 GDC 때문에 쥐고만 있던 VPP 작업 큐), 측정 창 시작 프레임에서 IVE로
// Shi-Tomasi 응답을 내고, IVE 피라미드 LK로 점을 한 프레임씩(앞뒤 검사는 거꾸로도) 따라간다. 원본 단은 링 슬롯을
// 복사하지 않고 그대로 읽는다. CPU는 호출과 좌표 변환만 한다(프레임당 약 1 ms).
//
// IVE는 헤더 문서와 다르게 동작한다(2026-10 보드 실측, docs/flow-calibration.md).
//  - LK 시작점은 가장 거친 단의 좌표로 준다(원본 좌표 / 2^(단 수 - 1)). 출력은 원본 좌표다.
//  - 전체 이동량은 ±8 px에서 잘리고, 음수 이동은 +16 px로 나온다. 그래서 한 프레임씩 따라가고, 되돌리고,
//    kMaxStepPx를 넘는 점은 버린다. bUseInitFlow로도 범위는 늘지 않는다.
//  - CornerDet(ST)의 점 목록은 늘 비어 있다. 응답 영상은 5x5 창 최소 고유값과 정확히 같으므로 점은
//    pick_flow_corners가 고른다. Sobel은 U16C1 출력만 받지만 값은 부호가 있다(CornerDet에는 S16C1로 준다).
//  - AX_IVE_Query(block)는 작업이 안 끝났어도 몇 ms 뒤 AX_ERR_IVE_QUERY_TIMEOUT(0x80150054)을 낸다.
//    그러면 다시 묻는다. 묻지 않고 넘어가면 다음 작업들이 줄줄이 실패한다.
//  - IVPS 동기 API는 AX_IVPS_Init을 부른 프로세스마다 TDP·VPP 작업 큐를 한 쌍씩 잡는데 보드 전체에
//    넷뿐이다(camerad·modeld·recordd·overlayd가 다 쓴다). 다섯째 프로세스는 BUSY라 축소 단을 만들지 못한다.
//  - VPP는 YUV400 출력을 받지 않아 축소 단은 NV12로 만든다(Y만 쓴다).
// AX_SYS_Init·AX_IVPS_Init은 같은 프로세스(modeld의 AxEngineSession·GdcWarp)가 해 둔 뒤에 만든다. AX 헤더는
// 이 파일 밖으로 새지 않는다.

#include "model/flow_engine.h"

#include <cstddef>
#include <cstdint>
#include <memory>

// 띠(kBandTop, kBandRows)와 점 한계(kMaxPoints)는 FlowEngine의 것이다.
class IveFlow : public FlowEngine {
public:
    static constexpr int kLevels = 3;          // LK 피라미드(원본, 1/2, 1/4)
    static constexpr float kMaxStepPx = 7.5f;  // 한 프레임 이동 한계(하드웨어는 8 px에서 자른다)

    // 크기는 1280x720 같은 캡처 크기다(가장 작은 단의 폭이 16화소 배수). 실패하면 std::runtime_error.
    IveFlow(int width, int height);
    ~IveFlow() override;
    IveFlow(const IveFlow &) = delete;
    IveFlow &operator=(const IveFlow &) = delete;

    // FlowEngine::push. Y 평면 바로 뒤에 UV가 있는 NV12(행 간격은 폭)다.
    bool push(unsigned long long y_phys) override;
    const int16_t *corner_response() override;
    bool track(const float *xy, int n, float *out_xy, uint8_t *ok, bool backward = false) override;

    int width() const override { return width_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    int width_ = 0;
};
