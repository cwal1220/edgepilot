#pragma once

// AX620E 하드웨어 비디오 디코더(VDEC). 한 번에 한 프레임(Annex B)을 넣고, 디코딩된
// NV12 프레임을 IVPS로 호출자의 CMM 블록(예: 프레임 링 슬롯)에 복사해 준다. H.264만 받는다. 리허설
// (replayd)이 녹화 영상을 카메라처럼 재생할 때 쓴다. MSP SDK를 직접 쓰며 AX 헤더는
// 이 파일 밖으로 새지 않는다. AX_SYS_Init은 생성자가 한다.

#include <cstddef>
#include <cstdint>
#include <memory>

class VideoDecoder {
public:
    VideoDecoder(int width, int height);
    ~VideoDecoder();
    VideoDecoder(const VideoDecoder &) = delete;
    VideoDecoder &operator=(const VideoDecoder &) = delete;

    // 한 프레임 분량의 비트스트림(세그먼트 첫 프레임이면 파라미터 세트 포함)을 넣는다.
    bool send(const uint8_t *data, size_t size, uint64_t pts, int timeout_ms = 100);
    // 디코딩된 프레임이 있으면 dst_phys(width x height NV12 CMM)에 복사하고 그 pts를
    // *pts에 넣는다. 없으면(timeout) false.
    bool receive_to(unsigned long long dst_phys, uint64_t *pts, int timeout_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
