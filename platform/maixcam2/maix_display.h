#pragma once

// MaixCAM2 LCD를 VO 하드웨어 두 레이어로 쓴다. 레이어 0은 카메라 YUV의 가운데 4:3을
// IVPS로 잘라 화면 크기로 줄여 올리고(비율 유지, 세로 패널로의 90° 회전과 보드 disp_flip/
// disp_mirror는 VO가 처리), 그 위 그래픽 레이어 fb0(세로 480x640 BGRA, 화소별 알파)에 HUD를
// CPU가 바로 쓴다. AX 헤더는 이 파일 밖으로 새지 않는다.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

class MaixDisplay {
public:
    static constexpr int kWidth = 640;
    static constexpr int kHeight = 480;

    MaixDisplay();
    ~MaixDisplay();
    MaixDisplay(const MaixDisplay &) = delete;
    MaixDisplay &operator=(const MaixDisplay &) = delete;

    // CMM 블록(프레임 링 슬롯)의 NV12에서 가운데 4:3을 IVPS가 직접 잘라 화면에
    // 맞추고, source_still_valid()가 참일 때만 레이어 0에 올린다(그동안 소스가
    // 덮어써졌으면 찢어진 프레임을 내보내지 않는다).
    bool show_video_phys(unsigned long long phys, int width, int height,
                         const std::function<bool()> &source_still_valid);
    // HUD 그림판: 세로 패널 방향(kHeight x kWidth BGRA, 행 stride 바이트)의 캐시 가능한 메모리.
    // 화면 좌표(가로 x, y)는 그림판의 (열 y, 행 x)다(transpose, 이 보드의 disp_flip=1·disp_mirror=0).
    // end_overlay가 이번과 지난번 프레임에 바뀐 칸만 fb0에 복사한다: dirty는 그림판 행마다
    // (1 << tile_shift) 폭 칸의 비트, nullptr이면 전부.
    // 예전에는 가로 버퍼를 MaixCDK VO가 TDP로 돌려 fb0에 넣었는데, TDP가 32비트 화소의 G·R·A를
    // 이웃 화소로 번지게 해(B만 그대로) 경로 가장자리가 계단지고 밝은 테가 생겼다(2026-10-03 fb0 덤프).
    struct OverlayBuffer {
        uint8_t *pixels = nullptr;
        int stride = 0;
    };
    OverlayBuffer begin_overlay();
    void end_overlay(const uint32_t *dirty = nullptr, int tile_shift = 0);
    // LCD 백라이트(PWM3). on이 거짓이면 끈다(duty 0). percent(1..100)에 보드 최대치(/boot/board
    // disp_max_backlight)를 곱한다. 시작할 때는 /boot/configs maix_backlight_value로 켜 둔다. sysfs에 쓰지
    // 못하면 false.
    bool set_backlight(bool on, float percent);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
