#pragma once

// 출발 알림. D 단에서 서 있을 때 앞차가 떠나거나 신호가 녹색으로 바뀌면(모델의 가스 예측) 정차 한 번에 한 번
// 알린다(docs/departure-alerts.md).

#include <cstdint>

enum class DepartureAlertType : uint32_t {
    None = 0,
    LeadDeparted = 1,
    GreenLight = 2,
};

const char *departure_alert_name(DepartureAlertType type);

struct DepartureAlertInput {
    double now_s = 0.0;
    bool vehicle_valid = false;
    int gear = 0;
    float speed_mps = 0.0f;
    bool gas_pressed = false;

    bool lead_updated = false;
    bool lead_valid = false;
    float lead_distance_m = 0.0f;
    float lead_relative_speed_mps = 0.0f;

    bool model_updated = false;
    bool model_valid = false;
    float plan_distance_m = 0.0f;
    float gas_press_prob = 0.0f;  // 모델이 본 "운전자가 2초 뒤 가속 페달을 밟고 있을" 확률
    bool turn_signal_on = false;  // 좌·우 방향지시등(비상등 포함)
};

struct DepartureAlertOutput {
    DepartureAlertType type = DepartureAlertType::None;
    uint32_t event_id = 0;
    bool green_light_armed = false;
};

class DepartureAlertDetector {
public:
    DepartureAlertOutput update(const DepartureAlertInput &input);

private:
    // 가까운 앞차(15 m 안)가 이번 정차에서 서 있는지: 본 최소 거리와, 그 뒤 거리·상대속도가 함께 넘어
    // 멀어졌는지. 서 있는 동안은 녹색 알림에 plan이 1.5초 넘게 열려 있기를 요구한다. 정차가 끝나야 지운다.
    struct CloseLeadGuard {
        double seen_s = -1.0;         // 가까운 앞차가 마지막으로 보인 시각
        float min_distance_m = 0.0f;  // 이번 정차에서 본 최소 거리
        bool moved = false;           // 그 앞차가 최소 거리에서 멀어졌다
        void observe(const DepartureAlertInput &input);
        bool holding(double now_s) const;
    };

    // vision 앞차 출발: 정차 1초 뒤부터 1초 보이면 무장하고, 거리 +0.5 m와 상대속도 0.5 m/s 초과가
    // 0.3초 이어지면 출발이다. 0.5초보다 짧은 끊김은 넘긴다.
    struct LeadDepartureTracker {
        double last_seen_s = -1.0;  // 마지막으로 보인 시각(짧은 끊김을 넘긴다)
        double seen_since_s = -1.0;
        double candidate_since_s = -1.0;
        float baseline_distance_m = 0.0f;
        bool armed = false;
        // 출발이 확인됐으면 true.
        bool update(const DepartureAlertInput &input, bool lead_present, bool stopped_long_enough);
    };

    // 신호 대기: 짧은 plan이 1.5초 이어지면 무장하고, plan이 0.3초 열리거나 2초 가속 확률이 두 프레임
    // 넘으면 녹색이다. 모델 프레임에서만 부른다.
    struct GreenLightTracker {
        double short_plan_since_s = -1.0;  // 이번 정차에서 plan이 짧게 이어지기 시작한 시각
        int gas_press_frames = 0;          // 무장 뒤 가속 확률이 기준을 넘은 연속 모델 프레임
        double candidate_since_s = -1.0;   // 무장 뒤 plan이 열려 있기 시작한 시각
        bool armed = false;
        // 녹색이 확인됐으면 true.
        bool update(const DepartureAlertInput &input);
        bool open_for(double now_s, double duration_s) const;
    };

    void reset_cycle();
    void trigger(DepartureAlertType type, double now_s);

    bool consumed_ = false;  // 이번 정차에서 이미 알렸다
    double stopped_since_s_ = -1.0;
    CloseLeadGuard close_lead_;
    LeadDepartureTracker lead_;
    GreenLightTracker green_light_;

    DepartureAlertType active_type_ = DepartureAlertType::None;
    double active_until_s_ = -1.0;
    uint32_t event_id_ = 0;
};
