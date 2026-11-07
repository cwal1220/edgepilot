#include "hud/hud_policy.h"

#include "hud/hud_renderer.h"

#include <cmath>
#include <cstdio>

namespace {

constexpr uint64_t kRejectLabelNs = 3'000'000'000ULL;
constexpr uint64_t kTurnSignalStepNs = 50'000'000ULL;
constexpr uint64_t kNetworkCardNs = 10'000'000'000ULL;

}  // namespace

bool HudAlertEvents::baseline(const ControlState &control) {
    const auto counter_reset = [](uint32_t current, uint32_t previous) { return previous != 0 && current < previous; };
    const bool counters_reset = initialized_ && (counter_reset(control.engage_event_id, last_engage_) ||
                                                 counter_reset(control.disengage_event_id, last_disengage_) ||
                                                 counter_reset(control.engage_reject_event_id, last_reject_) ||
                                                 counter_reset(control.departure_alert_event_id, last_departure_));
    if (initialized_ && !counters_reset) return false;
    last_engage_ = control.engage_event_id;
    last_disengage_ = control.disengage_event_id;
    last_reject_ = control.engage_reject_event_id;
    last_departure_ = control.departure_alert_event_id;
    initialized_ = true;
    return true;
}

HudSoundDecision HudAlertEvents::update(const ControlState &control, DepartureAlertType departure_type) {
    if (baseline(control)) return {};
    if (control.engage_reject_event_id != 0 && control.engage_reject_event_id != last_reject_) {
        last_reject_ = control.engage_reject_event_id;
        return {HudSound::Unable, last_reject_};
    }
    if (control.engage_event_id != 0 && control.engage_event_id != last_engage_) {
        last_engage_ = control.engage_event_id;
        return {HudSound::Engage, last_engage_};
    }
    if (control.disengage_event_id != 0 && control.disengage_event_id != last_disengage_) {
        last_disengage_ = control.disengage_event_id;
        return {HudSound::Disengage, last_disengage_};
    }
    if (departure_type != DepartureAlertType::None && control.departure_alert_event_id != 0 &&
        control.departure_alert_event_id != last_departure_) {
        last_departure_ = control.departure_alert_event_id;
        return {HudSound::SignalChanged, last_departure_};
    }
    return {};
}

AlertSoundId hud_sound_id(HudSound sound) {
    switch (sound) {
        case HudSound::Unable:
            return AlertSoundId::Unable;
        case HudSound::Engage:
            return AlertSoundId::Engage;
        case HudSound::Disengage:
            return AlertSoundId::Disengage;
        case HudSound::SignalChanged:
            return AlertSoundId::SignalChanged;
        case HudSound::TakeControl:
            return AlertSoundId::Unable;
        case HudSound::Unavailable:
            return AlertSoundId::Unavailable;
        case HudSound::None:
            break;
    }
    return AlertSoundId::Count;
}

HudSoundDecision HudAlertPolicy::update(const ControlState &control, bool control_fresh, const PandaState &panda,
                                        bool panda_fresh, uint64_t now_ns, HudState *hud) {
    HudSoundDecision decision;
    if (control_fresh) decision = events_.update(control, hud->departure_alert_type);
    if (decision.sound == HudSound::Unable) {
        std::snprintf(hud->engage_reject_label, sizeof(hud->engage_reject_label), "%s",
                      engage_block_label(control.engage_reject_block));
        reject_label_until_ns_ = now_ns + kRejectLabelNs;
    }
    if (now_ns >= reject_label_until_ns_) hud->engage_reject_label[0] = '\0';

    // 해제 예고·조향 한계: 다른 알림이 울린 프레임에는 이전 값을 두어 다음 프레임에 울린다.
    if (decision.sound == HudSound::None) {
        const bool soft = hud->soft_disabling, saturated = hud->steer_saturated;
        const bool rising = (soft && !previous_soft_disabling_) || (saturated && !previous_steer_saturated_);
        previous_soft_disabling_ = soft;
        previous_steer_saturated_ = saturated;
        if (rising) decision.sound = HudSound::TakeControl;
    }

    const bool panda_unavailable =
        panda.timestamp_ns != 0 && (!panda_fresh || !hud->panda_connected || !hud->panda_healthy || panda.faults != 0);
    const bool unavailable = !control_fresh || panda_unavailable || control.steering_fault != 0;
    if (!availability_initialized_) {
        availability_initialized_ = true;
    } else if (unavailable && !previous_unavailable_ && decision.sound == HudSound::None) {
        decision.sound = HudSound::Unavailable;
    }
    previous_unavailable_ = unavailable;
    return decision;
}

void TurnSignalClock::update(uint64_t now_ns, HudState *hud) {
    if (hud->left_blinker != left_ || hud->right_blinker != right_) {
        left_ = hud->left_blinker;
        right_ = hud->right_blinker;
        start_ns_ = now_ns;
    }
    const bool blinking = hud->left_blinker || hud->right_blinker;
    hud->turn_signal_step =
        blinking ? static_cast<int>(((now_ns - start_ns_) / kTurnSignalStepNs) % kTurnSignalSteps) : 0;
}

const char *HudTouch::tap(int x, int y, int width, int height, uint64_t now_ns, HudState *hud) {
    if (hud_status_touch(x, y, width)) {
        hud->network_card = !hud->network_card;
        network_card_until_ns_ = now_ns + kNetworkCardNs;
        return "network card";
    }
    if (hud_left_column_touch(x, y, height)) {
        hud->debug_card = !hud->debug_card;
        return "debug card";
    }
    hud->network_card = false;
    return "close";
}

void HudTouch::expire(uint64_t now_ns, HudState *hud) const {
    if (now_ns >= network_card_until_ns_) hud->network_card = false;
}

float smooth_lane_center_offset(float smoothed, float raw) {
    constexpr float kAlpha = 0.1f;
    if (!std::isfinite(raw)) return raw;
    return std::isfinite(smoothed) ? smoothed + kAlpha * (raw - smoothed) : raw;
}
