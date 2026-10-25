#pragma once

// HUD 알림음(alert_tones.h)을 MaixCAM2 보드 스피커로 낸다.
//
// 알림이 오면 소리 스레드가 ALSA 재생 장치를 직접 열어(ioctl, 48 kHz 스테레오 S16) 그 멜로디를 20 ms 단위로
// 흘려 넣고, 알림이 끝나면(버퍼의 마지막 60 ms까지 낸 뒤) 바로 닫는다. 장치가 열려 있는 동안 코덱 드라이버가
// 스피커 앰프(GPIO 49)를 켜 두는데, 켜진 앰프는 CPU·Wi-Fi 부하에 따른 전원 잡음을 그대로 낸다(2026-10-25: ssh로
// 영상을 받는 동안 지직거렸고, 장치를 닫자 조용했다. 아날로그 게인을 6 dB 낮춰도 그대로였다). 그래서 알림 사이에는
// 닫아 둔다.
// 앰프가 켜지는 동안 첫 음이 잘리지 않게 알림 앞에 무음 0.15초를 흘린다.
// 프로세스를 띄우지 않으므로 SD가 바빠도 알림이 늦지 않고 화면 루프도 멈추지 않는다(예전엔 알림마다 aplay를
// 띄웠는데, vfork 동안 부모가 멈춰 SD가 바쁠 때 HUD가 몇 초씩 얼었다). ALSA 버퍼는 60 ms다. 새 알림은 재생
// 중인 알림을 끊는다.
// 크기는 웹 기기 설정(params/display.json의 alert_volume_percent)으로 실행 중에 바꾼다.
// 환경: EDGEPILOT_ALERT_SOUND=0이면 끈다, EDGEPILOT_ALERT_VOLUME(0~100, 기본 70: 설정
// 파일에 값이 없을 때의 크기), EDGEPILOT_ALERT_PCM(재생 장치 노드, 기본 /dev/snd/pcmC0D1p).

#include "hud/alert_tones.h"

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

class AlertSound {
public:
    AlertSound();
    ~AlertSound();
    AlertSound(const AlertSound &) = delete;
    AlertSound &operator=(const AlertSound &) = delete;

    // 스레드에 알리기만 하고 곧바로 돌아온다.
    void play(AlertSoundId id);
    bool enabled() const { return enabled_; }
    // 0~100. 다음 20 ms 조각부터 적용된다.
    void set_volume_percent(float percent);
    float volume_percent() const { return volume_.load() * 100.0f; }

private:
    void loop();
    void close_pcm();

    bool enabled_ = false;
    std::string pcm_;
    std::vector<std::vector<int16_t>> clips_;  // 알림별 인터리브 스테레오 샘플
    std::atomic<int> pending_{-1};
    std::atomic<float> volume_{0.7f};  // 0~1, clips_(100% 크기)에 곱한다
    std::atomic<bool> stop_{false};
    int pcm_fd_ = -1;
    std::thread thread_;
};
