#pragma once

// 손가락 하나의 누름을 짧은 탭과 길게 누르기로 가른다. 터치 장치(MaixTouch)가 보고(SYN_REPORT)마다 누름과 그
// 자리(화면 좌표)를, 루프마다 지금 시각을 넘긴다. 장치 없이 돌아 호스트에서 시험한다. 시각은 마이크로초이고, 보고와
// poll이 같은 시계여야 한다.
//  - 탭: 누른 자리에서 kSlopPx 안에 머물다 kTapMaxUs 안에 뗐다. 뗄 때 누른 자리로 알린다.
//  - 길게 누르기: 누른 자리에서 kSlopPx 안에 머문 채 kLongPressUs가 지났다. 손을 떼기 전, 그 시각이 지난 첫 poll에
//    한 번 알린다. 그 누름은 떼어도 탭이 아니다.
//  - 누르는 사이 kSlopPx 넘게 움직였거나, kTapMaxUs와 kLongPressUs 사이에 뗀 누름은 아무것도 아니다.

#include <cstdint>
#include <cstdlib>

class TouchGesture {
public:
    static constexpr uint64_t kTapMaxUs = 600000;
    static constexpr uint64_t kLongPressUs = 1000000;
    static constexpr int kSlopPx = 40;

    enum class Kind { None, Tap, LongPress };
    struct Event {
        Kind kind = Kind::None;
        int x = 0;  // 누른 자리
        int y = 0;
    };

    // 보고 하나: 지금 누르고 있는지와 그 자리. 탭이 끝난 보고(뗌)에서 Tap이다.
    Event report(bool pressed, int x, int y, uint64_t us) {
        Event event;
        if (pressed && !pressed_) {
            pressed_ = true;
            moved_ = false;
            long_pressed_ = false;
            down_x_ = x;
            down_y_ = y;
            down_us_ = us;
        } else if (pressed) {
            moved_ = moved_ || away(x, y);
        } else if (pressed_) {
            pressed_ = false;
            if (!moved_ && !long_pressed_ && !away(x, y) && us - down_us_ <= kTapMaxUs)
                event = {Kind::Tap, down_x_, down_y_};
        }
        return event;
    }

    // 루프마다: 같은 자리를 kLongPressUs 넘게 누르고 있으면 그 누름에 한 번 LongPress.
    Event poll(uint64_t now_us) {
        if (!pressed_ || moved_ || long_pressed_ || now_us < down_us_ + kLongPressUs) return {};
        long_pressed_ = true;
        return {Kind::LongPress, down_x_, down_y_};
    }

    // 잃은 이벤트가 있으면(SYN_DROPPED) 지금 누름은 버린다.
    void drop() { pressed_ = false; }

private:
    bool away(int x, int y) const { return std::abs(x - down_x_) > kSlopPx || std::abs(y - down_y_) > kSlopPx; }

    bool pressed_ = false;
    bool moved_ = false;
    bool long_pressed_ = false;
    int down_x_ = 0;
    int down_y_ = 0;
    uint64_t down_us_ = 0;
};
