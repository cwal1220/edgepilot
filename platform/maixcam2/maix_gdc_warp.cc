#include "maixcam2/maix_gdc_warp.h"

#include "maixcam2/maix_frames.h"
#include "model/yuv6_pack.h"

#include <cmath>
#include <stdexcept>

namespace {

constexpr int kModelW = 512;
constexpr int kModelH = 256;
constexpr size_t kDstBytes = static_cast<size_t>(kModelW) * kModelH * 3 / 2;  // 196,608(페이지 배수)

// 가상 주소도 싣는다(CPU 주소를 아는 버퍼).
AX_VIDEO_FRAME_T mapped_nv12_frame(uint64_t phys, uint8_t *virt, int w, int h) {
    AX_VIDEO_FRAME_T f = nv12_frame(phys, w, h);
    f.u64VirAddr[0] = reinterpret_cast<uintptr_t>(virt);
    f.u64VirAddr[1] = reinterpret_cast<uintptr_t>(virt) + static_cast<uint64_t>(w) * h;
    return f;
}

}  // namespace

GdcWarp::GdcWarp(int src_width, int src_height) : src_w_(src_width), src_h_(src_height) {
    if (AX_IVPS_Init() != 0) throw std::runtime_error("AX_IVPS_Init failed");
    void *virt = nullptr;
    if (AX_SYS_MemAllocCached(&dst_phys_, &virt, static_cast<AX_U32>(2 * kDstBytes), 128,
                              reinterpret_cast<const AX_S8 *>("gdc_dst")) != 0)
        throw std::runtime_error("GDC output buffer alloc failed");
    dst_virt_ = static_cast<uint8_t *>(virt);
}

GdcWarp::~GdcWarp() {
    AX_SYS_MemFree(dst_phys_, dst_virt_);
    AX_IVPS_Deinit();
}

// view 0(medmodel)은 출력 블록 앞, 1(sbigmodel)은 뒤에 쓴다. projection[8]은 rpy 0에서 정확히 1이고, rpy 각 ±10°와
// 카메라 장착 설정 범위 전체에서 0.78~1.16이다(0이 되지 않는다).
bool GdcWarp::dewarp(unsigned long long src_phys, const float projection[9], int view) {
    AX_IVPS_DEWARP_ATTR_T attr = {};
    attr.eDewarpType = AX_IVPS_DEWARP_PERSPECTIVE;
    const double scale = 1.0 / projection[8];
    for (int i = 0; i < 9; ++i)  // 소수 6자리 고정소수점
        attr.tPerspectiveAttr.nMatrix[i] = static_cast<AX_S64>(std::llround(projection[i] * scale * 1e6));
    AX_VIDEO_FRAME_T src = mapped_nv12_frame(src_phys, nullptr, src_w_, src_h_);
    const size_t offset = static_cast<size_t>(view) * kDstBytes;
    AX_VIDEO_FRAME_T out = mapped_nv12_frame(dst_phys_ + offset, dst_virt_ + offset, kModelW, kModelH);
    return AX_IVPS_Dewarp(&src, &out, &attr) == 0;
}

bool GdcWarp::warp(unsigned long long src_phys, const float med_projection[9], const float sbig_projection[9],
                   uint8_t *med_yuv6, uint8_t *sbig_yuv6) {
    if (!dewarp(src_phys, med_projection, 0) || !dewarp(src_phys, sbig_projection, 1)) return false;
    // CPU는 출력 블록을 읽기만 하므로 두 시점을 다 받은 뒤 블록 전체를 한 번 무효화한다(프레임마다 같은 범위)
    AX_SYS_MinvalidateCache(dst_phys_, dst_virt_, static_cast<AX_U32>(2 * kDstBytes));
    nv12_to_yuv6(dst_virt_, med_yuv6, kModelW, kModelH);  // NV12 512x256 -> YUV6
    nv12_to_yuv6(dst_virt_ + kDstBytes, sbig_yuv6, kModelW, kModelH);
    return true;
}
