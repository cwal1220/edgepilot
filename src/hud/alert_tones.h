#pragma once

// HUD 알림음의 소리 자체. 짧은 종소리 같은 음(부드러운 시작, 지수 감쇠, 더 빨리 사라지는
// 2·3배음)을 겹쳐 만든다: 기쁜 알림은 위로, 해제는 아래로, 거부와 경고는 낮고 밝게.
// 재생(alert_sound.h)과 떨어져 있어 호스트에서 미리 듣기 파일을 만들고 시험한다.

#include <cstdint>
#include <vector>

enum class AlertSoundId { Unable, Engage, Disengage, SignalChanged, Unavailable, Count };

const char *alert_sound_name(AlertSoundId id);

// 모노 S16 샘플. 봉우리는 풀스케일의 kAlertTonePeak이고, 0에서 시작해 0으로 끝난다. 0.8에서는
// 보드 스피커가 찢어지는 소리를 내서(2026-10-04 실차) 예전 멜로디(0.6)보다 낮게 둔다. 크기는
// 웹 기기 설정의 알림음 크기로 맞춘다.
constexpr double kAlertTonePeak = 0.5;
std::vector<int16_t> render_alert_tone(AlertSoundId id, int rate);
