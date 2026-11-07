// hud_policy: overlayd가 그리기 밖에서 정하는 것. 제어 이벤트 카운터 → 알림(첫 스냅샷은 기준값,
// 우선순위, 한 프레임 하나, controlsd 재시작), 프레임마다 알림음 하나(해제 예고는 미루고 불가용 천이는
// 넘긴다, 결합 거부 토스트 3초), 깜빡이 단계, 터치로 여닫는 카드, 차선 위치 평활.
#include "hud/hud_policy.h"

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

constexpr uint64_t kMs = 1'000'000ULL;

ControlState control_with_events(uint32_t engage, uint32_t disengage, uint32_t reject, uint32_t departure) {
    ControlState c;
    c.engage_event_id = engage;
    c.disengage_event_id = disengage;
    c.engage_reject_event_id = reject;
    c.departure_alert_event_id = departure;
    return c;
}

TEST(HudAlertEvents, FirstSnapshotIsABaseline) {
    HudAlertEvents events;
    const auto first = events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::None);
    ASSERT_EQ(first.sound, HudSound::None) << "첫 스냅샷은 기준값만 잡는다";
    const auto same = events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::None);
    ASSERT_EQ(same.sound, HudSound::None) << "그대로인 카운터는 이벤트가 아니다";
    const auto engaged = events.update(control_with_events(6, 3, 2, 1), DepartureAlertType::None);
    // 새 결합 id는 그 id로 결합 알림음을 낸다
    ASSERT_EQ(engaged.sound, HudSound::Engage);
    ASSERT_EQ(engaged.event_id, 6);
}

TEST(HudAlertEvents, PriorityAndOneAlertPerFrame) {
    HudAlertEvents events;
    events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::None);
    const ControlState burst = control_with_events(6, 4, 3, 2);
    const auto first = events.update(burst, DepartureAlertType::LeadDeparted);
    // 같은 프레임에서는 결합 거부가 가장 먼저다
    ASSERT_EQ(first.sound, HudSound::Unable);
    ASSERT_EQ(first.event_id, 3);
    const auto second = events.update(burst, DepartureAlertType::LeadDeparted);
    // 다음 프레임에 결합 알림음
    ASSERT_EQ(second.sound, HudSound::Engage);
    ASSERT_EQ(second.event_id, 6);
    const auto third = events.update(burst, DepartureAlertType::LeadDeparted);
    // 그다음 해제 알림음
    ASSERT_EQ(third.sound, HudSound::Disengage);
    ASSERT_EQ(third.event_id, 4);
    const auto fourth = events.update(burst, DepartureAlertType::LeadDeparted);
    // 그다음 출발 알림음
    ASSERT_EQ(fourth.sound, HudSound::SignalChanged);
    ASSERT_EQ(fourth.event_id, 2);
    ASSERT_EQ(events.update(burst, DepartureAlertType::LeadDeparted).sound, HudSound::None)
        << "이벤트는 모두 한 번씩만 소비된다";
}

TEST(HudAlertEvents, DepartureWaitsForADisplayedType) {
    HudAlertEvents events;
    events.update(control_with_events(0, 0, 0, 0), DepartureAlertType::None);
    const ControlState departed = control_with_events(0, 0, 0, 7);
    ASSERT_EQ(events.update(departed, DepartureAlertType::None).sound, HudSound::None)
        << "표시할 알림 종류가 없으면 출발 id를 소비하지 않는다";
    const auto later = events.update(departed, DepartureAlertType::GreenLight);
    // 알림 종류가 표시되면 같은 id로 울린다
    ASSERT_EQ(later.sound, HudSound::SignalChanged);
    ASSERT_EQ(later.event_id, 7);
}

TEST(HudAlertEvents, ControlsdRestartRebaselines) {
    HudAlertEvents events;
    events.update(control_with_events(40, 39, 12, 9), DepartureAlertType::None);
    const auto restarted = events.update(control_with_events(1, 0, 0, 0), DepartureAlertType::None);
    ASSERT_EQ(restarted.sound, HudSound::None) << "카운터가 줄면 controlsd 재시작이다. 기준을 다시 잡고 울리지 않는다";
    const auto next = events.update(control_with_events(2, 0, 0, 0), DepartureAlertType::None);
    // 재시작 뒤 이벤트는 새 기준으로 잡는다
    ASSERT_EQ(next.sound, HudSound::Engage);
    ASSERT_EQ(next.event_id, 2);
}

// 신선한 제어·panda 스냅샷 한 쌍으로 정책을 돌리는 틀.
struct PolicyFrame {
    HudAlertPolicy policy;
    ControlState control = control_with_events(0, 0, 0, 0);
    PandaState panda;
    HudState hud;
    bool control_fresh = true;
    bool panda_fresh = true;

    PolicyFrame() {
        panda.timestamp_ns = 1;
        hud.panda_connected = hud.panda_healthy = true;
    }
    HudSoundDecision step(uint64_t now_ns) {
        return policy.update(control, control_fresh, panda, panda_fresh, now_ns, &hud);
    }
};

TEST(HudAlertPolicy, RejectShowsItsReasonForThreeSeconds) {
    PolicyFrame f;
    ASSERT_EQ(f.step(0).sound, HudSound::None) << "첫 프레임은 기준값";
    f.control.engage_reject_event_id = 4;
    std::snprintf(f.control.engage_reject_block, sizeof(f.control.engage_reject_block), "door_open");
    const HudSoundDecision rejected = f.step(1000 * kMs);
    ASSERT_EQ(rejected.sound, HudSound::Unable);
    ASSERT_EQ(rejected.event_id, 4u);
    ASSERT_EQ(hud_sound_id(rejected.sound), AlertSoundId::Unable);
    ASSERT_STREQ(f.hud.engage_reject_label, "DOOR OPEN") << "토스트는 사유 라벨";
    f.step(3999 * kMs);
    ASSERT_STREQ(f.hud.engage_reject_label, "DOOR OPEN");
    f.step(4000 * kMs);
    ASSERT_STREQ(f.hud.engage_reject_label, "") << "3초 뒤 내린다";
}

TEST(HudAlertPolicy, TakeControlWaitsForAFrameWithoutAnEvent) {
    PolicyFrame f;
    f.step(0);
    f.control.disengage_event_id = 1;
    f.hud.soft_disabling = true;
    ASSERT_EQ(f.step(10 * kMs).sound, HudSound::Disengage) << "제어 이벤트가 먼저";
    const HudSoundDecision next = f.step(20 * kMs);
    ASSERT_EQ(next.sound, HudSound::TakeControl) << "해제 예고는 다음 프레임에 울린다";
    ASSERT_EQ(hud_sound_id(next.sound), AlertSoundId::Unable);
    ASSERT_EQ(f.step(30 * kMs).sound, HudSound::None) << "켜지는 순간 한 번";
    f.hud.soft_disabling = false;
    f.hud.steer_saturated = true;
    ASSERT_EQ(f.step(40 * kMs).sound, HudSound::TakeControl) << "조향 한계도 같은 소리";
}

TEST(HudAlertPolicy, UnavailableSoundsOnTheTransitionOnly) {
    PolicyFrame f;
    f.control_fresh = false;
    ASSERT_EQ(f.step(0).sound, HudSound::None) << "첫 프레임의 불가용은 기준값";
    f.control_fresh = true;
    ASSERT_EQ(f.step(10 * kMs).sound, HudSound::None);
    f.control.steering_fault = 1;
    ASSERT_EQ(f.step(20 * kMs).sound, HudSound::Unavailable);
    ASSERT_EQ(f.step(30 * kMs).sound, HudSound::None) << "불가용이 이어지면 다시 울리지 않는다";
    f.control.steering_fault = 0;
    ASSERT_EQ(f.step(40 * kMs).sound, HudSound::None) << "가용으로 돌아올 때는 조용하다";
    f.panda.faults = 2;
    ASSERT_EQ(f.step(50 * kMs).sound, HudSound::Unavailable) << "panda 결함도 불가용";
    f.panda.faults = 0;
    f.step(60 * kMs);
    f.hud.panda_connected = false;
    ASSERT_EQ(f.step(70 * kMs).sound, HudSound::Unavailable) << "panda 끊김도 불가용";

    PolicyFrame never;
    never.panda.timestamp_ns = 0;
    never.hud.panda_connected = false;
    never.step(0);
    ASSERT_EQ(never.step(10 * kMs).sound, HudSound::None) << "panda 스냅샷을 본 적 없으면 panda는 따지지 않는다";
}

TEST(HudAlertPolicy, UnavailableIsSkippedInAFrameThatAlreadySounded) {
    PolicyFrame f;
    f.step(0);
    f.control.disengage_event_id = 1;
    f.control.steering_fault = 1;
    ASSERT_EQ(f.step(10 * kMs).sound, HudSound::Disengage);
    ASSERT_EQ(f.step(20 * kMs).sound, HudSound::None)
        << "같은 프레임에 다른 알림이 울렸으면 불가용 천이는 미루지 않고 넘긴다";
}

TEST(HudSound, Sounds) {
    ASSERT_EQ(hud_sound_id(HudSound::Engage), AlertSoundId::Engage);
    ASSERT_EQ(hud_sound_id(HudSound::Unavailable), AlertSoundId::Unavailable);
    ASSERT_EQ(hud_sound_id(HudSound::None), AlertSoundId::Count);
}

TEST(TurnSignalClock, StepsFromTheBlinkerChange) {
    TurnSignalClock clock;
    HudState hud;
    clock.update(1000 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, 0);
    hud.left_blinker = true;
    clock.update(1000 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, 0) << "켠 순간은 단계 0";
    clock.update(1049 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, 0);
    clock.update(1050 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, 1) << "50 ms마다 한 단계";
    clock.update(1000 * kMs + 24 * 50 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, kTurnSignalSteps - 1);
    clock.update(1000 * kMs + 25 * 50 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, 0) << "kTurnSignalSteps마다 처음으로";

    hud.right_blinker = true;  // 비상등: 상태가 바뀌어 다시 센다
    clock.update(2000 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, 0);
    hud.left_blinker = hud.right_blinker = false;
    clock.update(2170 * kMs, &hud);
    ASSERT_EQ(hud.turn_signal_step, 0) << "깜빡이가 꺼져 있으면 단계 0";
}

TEST(HudTouch, CardsToggleAndTheNetworkCardTimesOut) {
    constexpr int kW = 640, kH = 480;
    HudTouch touch;
    HudState hud;
    ASSERT_STREQ(touch.tap(600, 20, kW, kH, 0, &hud), "network card") << "오른쪽 위 상태 알약";
    ASSERT_TRUE(hud.network_card);
    touch.expire(9999 * kMs, &hud);
    ASSERT_TRUE(hud.network_card);
    touch.expire(10000 * kMs, &hud);
    ASSERT_FALSE(hud.network_card) << "10초 뒤 저절로 닫힌다";

    ASSERT_STREQ(touch.tap(600, 20, kW, kH, 20000 * kMs, &hud), "network card");
    ASSERT_STREQ(touch.tap(600, 20, kW, kH, 21000 * kMs, &hud), "network card");
    ASSERT_FALSE(hud.network_card) << "다시 누르면 닫는다";
    touch.tap(600, 20, kW, kH, 22000 * kMs, &hud);
    ASSERT_STREQ(touch.tap(400, 300, kW, kH, 23000 * kMs, &hud), "close") << "그 밖을 누르면 닫는다";
    ASSERT_FALSE(hud.network_card);

    ASSERT_STREQ(touch.tap(100, 200, kW, kH, 24000 * kMs, &hud), "debug card") << "왼쪽 열";
    ASSERT_TRUE(hud.debug_card);
    touch.expire(60000 * kMs, &hud);
    ASSERT_TRUE(hud.debug_card) << "진단 카드는 시간으로 닫지 않는다";
    touch.tap(100, 200, kW, kH, 61000 * kMs, &hud);
    ASSERT_FALSE(hud.debug_card);
}

TEST(LaneCenterSmoothing, FollowsAtATenthPerFrameAndRestartsAfterALoss) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    ASSERT_FLOAT_EQ(smooth_lane_center_offset(nan, 0.3f), 0.3f) << "처음은 그 값";
    ASSERT_FLOAT_EQ(smooth_lane_center_offset(0.3f, 0.5f), 0.32f);
    ASSERT_TRUE(std::isnan(smooth_lane_center_offset(0.3f, nan))) << "차선을 놓치면 비운다";
    float smoothed = 0.0f;
    for (int i = 0; i < 10; ++i) smoothed = smooth_lane_center_offset(smoothed, 1.0f);  // 0.5초
    ASSERT_NEAR(smoothed, 1.0f - std::pow(0.9f, 10.0f), 1e-5f);
}

}  // namespace
