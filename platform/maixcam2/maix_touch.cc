#include "maixcam2/maix_touch.h"

#include "maixcam2/maix_display.h"

#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <ctime>
#include <fstream>
#include <string>

namespace {

// /sys/class/input/eventN/device/name이 hyn_ts인 장치(보드에서는 event1).
std::string find_device() {
    for (int i = 0;; ++i) {
        std::ifstream name("/sys/class/input/event" + std::to_string(i) + "/device/name");
        if (!name) return {};
        std::string line;
        std::getline(name, line);
        if (line == "hyn_ts") return "/dev/input/event" + std::to_string(i);
    }
}

uint64_t event_us(const input_event &event) {
    return static_cast<uint64_t>(event.time.tv_sec) * 1'000'000ULL + static_cast<uint64_t>(event.time.tv_usec);
}

}  // namespace

MaixTouch::~MaixTouch() {
    if (fd_ >= 0) ::close(fd_);
}

bool MaixTouch::open() {
    const std::string device = find_device();
    if (device.empty()) return false;
    fd_ = ::open(device.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) return false;
    int clock = CLOCK_MONOTONIC;  // 누른 시간을 시계 조정과 무관하게 잰다. 못 바꾸면 이벤트 시각은 CLOCK_REALTIME
    if (ioctl(fd_, EVIOCSCLOCKID, &clock) == 0) clock_ = CLOCK_MONOTONIC;
    input_absinfo info{};
    raw_x_max_ =
        ioctl(fd_, EVIOCGABS(ABS_MT_POSITION_X), &info) == 0 && info.maximum > 0 ? info.maximum : MaixDisplay::kHeight;
    return true;
}

TouchGesture::Event MaixTouch::poll() {
    if (fd_ < 0) return {};
    TouchGesture::Event result;
    input_event events[16];
    ssize_t bytes = 0;
    while ((bytes = ::read(fd_, events, sizeof(events))) > 0) {
        const int count = static_cast<int>(bytes / static_cast<ssize_t>(sizeof(input_event)));
        for (int i = 0; i < count; ++i) {
            const input_event &e = events[i];
            if (e.type == EV_ABS) {
                if (e.code == ABS_MT_SLOT)
                    slot_ = e.value;
                else if (slot_ == 0 && e.code == ABS_MT_POSITION_X)
                    raw_x_ = e.value;
                else if (slot_ == 0 && e.code == ABS_MT_POSITION_Y)
                    raw_y_ = e.value;
            } else if (e.type == EV_KEY && e.code == BTN_TOUCH) {
                touching_ = e.value != 0;
            } else if (e.type == EV_SYN && e.code == SYN_DROPPED) {
                touching_ = false;  // 잃은 이벤트가 있으면 이번 누름은 버린다
                gesture_.drop();
            } else if (e.type == EV_SYN && e.code == SYN_REPORT) {
                int sx = 0, sy = 0;
                to_screen(&sx, &sy);
                const TouchGesture::Event event = gesture_.report(touching_, sx, sy, event_us(e));
                if (event.kind != TouchGesture::Kind::None) result = event;
            }
        }
    }
    if (result.kind != TouchGesture::Kind::None) return result;
    timespec now{};
    clock_gettime(clock_, &now);
    return gesture_.poll(static_cast<uint64_t>(now.tv_sec) * 1'000'000ULL + static_cast<uint64_t>(now.tv_nsec) / 1000);
}

// MaixCDK와 같은 시계 방향 90°: 패널 y → 화면 x, 패널 x → 화면 아래에서 위로.
void MaixTouch::to_screen(int *x, int *y) const {
    *x = std::clamp(raw_y_, 0, MaixDisplay::kWidth - 1);
    *y = std::clamp(raw_x_max_ - raw_x_ - 1, 0, MaixDisplay::kHeight - 1);
}
