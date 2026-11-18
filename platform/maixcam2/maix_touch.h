#pragma once

// MaixCAM2 터치스크린(hyn_ts, 멀티터치 B형)에서 짧은 탭과 길게 누르기를 화면 좌표(MaixDisplay 640x480)로
// 알려 준다(가르는 규칙은 hud/touch_gesture.h). 패널은 480x640 세로라 MaixCDK(maix_touchscreen_maixcam2.hpp)처럼
// 시계 방향 90°로 돌린다. 장치는 막지 않고(O_NONBLOCK) 읽으며 첫 손가락(슬롯 0)만 본다.

#include "hud/touch_gesture.h"

#include <ctime>

class MaixTouch {
public:
    MaixTouch() = default;
    ~MaixTouch();
    MaixTouch(const MaixTouch &) = delete;
    MaixTouch &operator=(const MaixTouch &) = delete;

    // 장치가 없으면 false. 그 뒤 poll은 아무것도 하지 않는다.
    bool open();
    // 쌓인 이벤트를 읽고, 그사이 끝난 탭이나 지금 다 찬 길게 누르기가 있으면 그것(없으면 Kind::None). 루프마다
    // 부른다: 길게 누르기는 이벤트가 없어도 시각으로 찬다.
    TouchGesture::Event poll();

private:
    void to_screen(int *x, int *y) const;

    int fd_ = -1;
    clockid_t clock_ = CLOCK_REALTIME;  // 이벤트 시각의 시계(open이 CLOCK_MONOTONIC으로 바꾼다)
    int raw_x_max_ = 0;
    int slot_ = 0;
    int raw_x_ = 0;  // 슬롯 0의 최근 좌표(패널 기준)
    int raw_y_ = 0;
    bool touching_ = false;  // BTN_TOUCH
    TouchGesture gesture_;
};
