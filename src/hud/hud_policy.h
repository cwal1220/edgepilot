#pragma once

// overlayd가 그리기 밖에서 정하는 것: 프레임마다 울릴 알림음, 터치로 여닫는 카드, 깜빡이 애니메이션
// 단계, 길 위 차선 안 위치 눈금의 평활. 화면·스피커·터치 장치 없이 돌아 호스트에서 시험한다.

#include "common/ipc_messages.h"
#include "hud/alert_tones.h"
#include "hud/hud_state.h"

#include <cstdint>

// 한 프레임에 울린 알림. 앞의 넷은 controlsd 제어 이벤트(열거 순서가 같은 프레임 안의 우선순위다),
// take_control은 해제 예고나 조향 한계가 켜진 순간, unavailable은 가용 → 불가용 천이다.
enum class HudSound { None, Unable, Engage, Disengage, SignalChanged, TakeControl, Unavailable };

struct HudSoundDecision {
    HudSound sound = HudSound::None;
    uint32_t event_id = 0;  // 제어 이벤트 알림의 id
};

// controlsd 이벤트 카운터 → 이 프레임에 울릴 제어 이벤트 알림 하나. 신선한 제어 스냅샷에만
// 부른다. overlayd가 독립적으로 재시작될 수 있으므로 첫 스냅샷은 사용자 이벤트가
// 아니라 기준값이고, controlsd 재시작으로 카운터가 줄어들면 기준값을 다시 잡는다.
// 거부 > 결합 > 해제 > 출발 순으로 새 이벤트 하나만 고르고, 같은 프레임의
// 나머지는 다음 프레임에 잡힌다. 출발은 표시 중인 알림 유형이 있을 때만 소비한다.
class HudAlertEvents {
public:
    HudSoundDecision update(const ControlState &control, DepartureAlertType departure_type);

private:
    // 기준값을 (다시) 잡은 프레임이면 true.
    bool baseline(const ControlState &control);

    bool initialized_ = false;
    uint32_t last_engage_ = 0;
    uint32_t last_disengage_ = 0;
    uint32_t last_reject_ = 0;
    uint32_t last_departure_ = 0;
};

// 낼 소리. 해제 예고·조향 한계는 거부와 같은 소리다. none이면 AlertSoundId::Count.
AlertSoundId hud_sound_id(HudSound sound);

// 한 프레임에 알림음 하나를 고른다. 제어 이벤트가 먼저이고, 해제 예고·조향 한계가 켜지는 순간은
// 그 프레임에 이벤트가 울렸으면 다음 프레임으로 미룬다. 가용 → 불가용 천이에도 울리는데(active
// 천이는 정차 부근 path 깜빡임마다 울리므로 소리내지 않는다), 같은 프레임에 다른 알림이 울렸으면
// 넘긴다. 첫 프레임의 가용 상태는 기준값이다. 결합 거부는 HUD 토스트(engage_reject_label)도
// 3초 띄운다.
class HudAlertPolicy {
public:
    // hud는 이번 프레임의 매핑을 마친 HUD 상태(출발 알림 유형, panda 연결, 해제 예고·조향 한계를
    // 읽고 토스트 라벨을 쓴다).
    HudSoundDecision update(const ControlState &control, bool control_fresh, const PandaState &panda, bool panda_fresh,
                            uint64_t now_ns, HudState *hud);

private:
    HudAlertEvents events_;
    uint64_t reject_label_until_ns_ = 0;
    bool previous_soft_disabling_ = false;
    bool previous_steer_saturated_ = false;
    bool availability_initialized_ = false;
    bool previous_unavailable_ = false;
};

// 깜빡이 애니메이션 단계: 깜빡이 상태가 바뀐 시각부터 50 ms마다 한 단계, kTurnSignalSteps마다 처음으로.
// 재그리기 빈도와 무관하게 시각으로 센다.
class TurnSignalClock {
public:
    // hud의 깜빡이로 단계를 hud->turn_signal_step에 쓴다.
    void update(uint64_t now_ns, HudState *hud);

private:
    bool left_ = false;
    bool right_ = false;
    uint64_t start_ns_ = 0;
};

// 터치: 상태 알약은 네트워크 카드를, 왼쪽 열은 진단 카드를 켜고 끈다. 그 밖을 누르면 열린 네트워크
// 카드를 닫는다. 네트워크 카드는 10초 뒤 저절로 닫힌다. 진단 카드는 overlayd가 다시 시작하면 꺼진다.
// width·height는 화면 크기.
class HudTouch {
public:
    // 탭 하나를 반영하고 로그에 남길 동작 이름을 돌려준다.
    const char *tap(int x, int y, int width, int height, uint64_t now_ns, HudState *hud);
    // 시간이 다 된 네트워크 카드를 닫는다.
    void expire(uint64_t now_ns, HudState *hud) const;

private:
    uint64_t network_card_until_ns_ = 0;
};

// 길 위 차선 안 위치 눈금(draw_lane_position)의 오프셋을 모델 프레임마다 다듬는다(20 Hz에서 0.5초 시간 상수,
// 눈금과 cm 숫자의 떨림). raw가 NaN이면(차선을 놓침) 비우고, 다시 잡으면 그 값에서 시작한다.
float smooth_lane_center_offset(float smoothed, float raw);
