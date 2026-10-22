#include "controls/departure_alert.h"

#include "car/can_frame.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kStoppedSpeedMps = 0.1f;
constexpr float kMovingResetSpeedMps = 0.5f;
constexpr double kMinimumStopTimeS = 1.0;

constexpr float kMinimumLeadDistanceM = 1.0f;
constexpr float kMaximumLeadDistanceM = 30.0f;
constexpr double kLeadArmTimeS = 1.0;
constexpr float kLeadDistanceChangeM = 0.5f;
constexpr float kLeadRelativeSpeedMps = 0.5f;
constexpr double kLeadConfirmTimeS = 0.3;

constexpr float kMinimumStoppedPlanDistanceM = -5.0f;
constexpr float kStoppedPlanDistanceM = 5.0f;
// 094 plan은 정차 중 중앙 1.3 m로 붕괴하고 출발 예고 시 10 m를 넘긴 뒤 25 m까지
// 0.8 s 더 걸린다(09-01 루트 13회 정차, 오경보 차이 없음).
constexpr float kOpenPlanDistanceM = 10.0f;
// 정차 중 plan이 이만큼 짧게 이어지면 신호 대기로 보고 무장한다(HUD 신호등도 이때 뜬다). 3 s였을 때
// 2026-10-04 실차에서 정차 2.7 s 만에 녹색이 켜진 신호를 놓쳤다.
constexpr double kGreenLightArmTimeS = 1.5;
constexpr double kGreenLightConfirmTimeS = 0.3;
// 모델 meta의 2초 가속 확률. 2026-10-04 녹화를 fp32 모델로 다시 돌리면 앞차 없는 녹색 8번 모두
// 녹색 0.25초 안에 0.3을 넘고(경로가 끝내 안 열린 두 번 포함) 빨간불 동안은 0.21 아래였다. 보드의
// U16 모델(SmoothQuant 빌드)은 같은 입력에서 6번 넘는다. 앞차가 서 있으면 녹색이어도 오르지 않고
// 앞차가 움직일 때 오른다. 두 프레임 연속을 요구해 한 프레임 튐을 거른다.
constexpr float kGasPressGoProbability = 0.3f;
constexpr int kGasPressGoFrames = 2;
constexpr double kAlertDisplayTimeS = 3.0;
// vision 앞차는 모델 프레임 하나씩 끊길 수 있다. 이보다 짧은 끊김은 앞차가 계속 있는 것으로 본다
// (예전에는 한 번 끊기면 무장을 처음부터 다시 재서 앞차 출발 알림이 거의 무장되지 못했다).
constexpr double kLeadDropoutHoldS = 0.5;
// 이 거리 안에 앞차가 서 있으면 녹색 알림(plan 열림·가속 확률)은 앞차가 실제로 멀어진 뒤에만 낸다.
// 2026-10-04 낮 주행에서 3 m 앞 정지 차 뒤 plan이 0.3초 열렸다 닫혀 23초 일찍 울렸다.
constexpr float kCloseLeadDistanceM = 15.0f;
constexpr double kCloseLeadHoldS = 1.0;
// 서 있는 앞차 뒤에서 plan이 이만큼 계속 열려 있으면 녹색으로 본다. 2026-10-04 오경보의 plan 열림은
// 0.3~0.5초였고, 앞차 운전자가 늦게 출발한 녹색은 2~4.6초 열려 있었다.
constexpr double kCloseLeadOpenConfirmS = 1.5;

bool elapsed(double now_s, double since_s, double duration_s) {
    return since_s >= 0.0 && now_s >= since_s && now_s - since_s >= duration_s;
}

}  // namespace

const char *departure_alert_name(DepartureAlertType type) {
    switch (type) {
        case DepartureAlertType::LeadDeparted:
            return "lead_departed";
        case DepartureAlertType::GreenLight:
            return "green_light";
        case DepartureAlertType::None:
        default:
            return "none";
    }
}

DepartureAlertOutput DepartureAlertDetector::update(const DepartureAlertInput &input) {
    if (active_type_ != DepartureAlertType::None && input.now_s >= active_until_s_) {
        active_type_ = DepartureAlertType::None;
    }

    // 정차 주기: 기어·가속 페달·0.5 m/s 넘는 출발은 정차를 끝내고, 서행(0.1~0.5 m/s)은 정차 시각과
    // 앞차·녹색 추적만 지운다(가까운 앞차와 이번 정차의 알림 여부는 남긴다).
    const bool reset = !input.vehicle_valid || input.gear != kGearDrive || input.gas_pressed ||
                       !std::isfinite(input.speed_mps) || input.speed_mps > kMovingResetSpeedMps;
    if (reset) {
        reset_cycle();
    } else if (input.speed_mps <= kStoppedSpeedMps) {
        if (stopped_since_s_ < 0.0) stopped_since_s_ = input.now_s;
    } else {
        stopped_since_s_ = -1.0;
        lead_ = LeadDepartureTracker{};
        green_light_ = GreenLightTracker{};
    }

    const bool stopped_long_enough = elapsed(input.now_s, stopped_since_s_, kMinimumStopTimeS);
    const bool stopped = stopped_since_s_ >= 0.0;
    if (stopped && !consumed_) {
        const bool lead_present = input.lead_valid && std::isfinite(input.lead_distance_m) &&
                                  input.lead_distance_m >= kMinimumLeadDistanceM &&
                                  input.lead_distance_m <= kMaximumLeadDistanceM;
        if (lead_present) close_lead_.observe(input);
        const bool lead_departed = lead_.update(input, lead_present, stopped_long_enough);
        bool green_light_changed = input.model_updated && input.model_valid && green_light_.update(input);

        // 가까운 앞차가 서 있는 동안은 plan이 kCloseLeadOpenConfirmS 넘게 계속 열려 있을 때만 녹색으로
        // 본다(앞차가 움직이면 평소대로). 가속 확률은 쓰지 않는다: 앞차가 움직일 때 함께 오른다. 무장과 후보
        // 시각은 그대로 두어 앞차가 멀어지는 순간 바로 낸다. 모델 프레임과 무관하게 매 틱 판단한다.
        if (close_lead_.holding(input.now_s))
            green_light_changed = green_light_.open_for(input.now_s, kCloseLeadOpenConfirmS);

        // 둘이 같은 프레임에 성립하면 더 구체적인 사유(앞차 출발)를 쓴다.
        if (lead_departed) {
            trigger(DepartureAlertType::LeadDeparted, input.now_s);
        } else if (green_light_changed) {
            trigger(DepartureAlertType::GreenLight, input.now_s);
        }
    }

    return {
        active_type_,
        event_id_,
        green_light_.armed,
    };
}

void DepartureAlertDetector::CloseLeadGuard::observe(const DepartureAlertInput &input) {
    if (input.lead_distance_m <= kCloseLeadDistanceM) {
        if (seen_s < 0.0 || input.lead_distance_m < min_distance_m) min_distance_m = input.lead_distance_m;
        seen_s = input.now_s;
    }
    // 서 있는 앞차의 vision 거리는 1 m까지 흔들리지만(2026-10-04 2.8~3.8 m) 상대속도는 0.12 m/s 아래다.
    // 그래서 거리와 상대속도가 함께 넘어야 움직인 것으로 본다(앞차 출발 판정과 같은 기준).
    if (seen_s >= 0.0 && input.lead_distance_m - min_distance_m > kLeadDistanceChangeM &&
        input.lead_relative_speed_mps > kLeadRelativeSpeedMps)
        moved = true;
}

bool DepartureAlertDetector::CloseLeadGuard::holding(double now_s) const {
    return seen_s >= 0.0 && !moved && !elapsed(now_s, seen_s, kCloseLeadHoldS);
}

bool DepartureAlertDetector::LeadDepartureTracker::update(const DepartureAlertInput &input, bool lead_present,
                                                          bool stopped_long_enough) {
    if (!lead_present) {
        if (!elapsed(input.now_s, last_seen_s, 0.0) || elapsed(input.now_s, last_seen_s, kLeadDropoutHoldS))
            *this = LeadDepartureTracker{};
        return false;
    }
    last_seen_s = input.now_s;
    if (!stopped_long_enough || !input.lead_updated) return false;
    if (seen_since_s < 0.0) {
        seen_since_s = input.now_s;
        baseline_distance_m = input.lead_distance_m;
    } else {
        baseline_distance_m = std::min(baseline_distance_m, input.lead_distance_m);
    }

    armed = elapsed(input.now_s, seen_since_s, kLeadArmTimeS);
    const bool departing = armed && input.lead_distance_m - baseline_distance_m > kLeadDistanceChangeM &&
                           input.lead_relative_speed_mps > kLeadRelativeSpeedMps;
    if (!departing) {
        candidate_since_s = -1.0;
        return false;
    }
    if (candidate_since_s < 0.0) candidate_since_s = input.now_s;
    return elapsed(input.now_s, candidate_since_s, kLeadConfirmTimeS);
}

bool DepartureAlertDetector::GreenLightTracker::update(const DepartureAlertInput &input) {
    const bool model_stopped = std::isfinite(input.plan_distance_m) &&
                               input.plan_distance_m > kMinimumStoppedPlanDistanceM &&
                               input.plan_distance_m < kStoppedPlanDistanceM;

    // 정차 중 모델이 여기서 멈추겠다고 계획한 지 1.5 s가 지나면 무장한다. 그 전에 길이 열리면(막
    // 서서 아직 출발 계획이거나 한 프레임 튐) 처음부터 다시 잰다. 5~10 m 사이 흔들림은 재지 않는다.
    // vision lead는 무장을 막지 않는다: 094 lead 확률은 상수 0.6에 가까워 앞차 유무를
    // 가르지 못하고(drive15: 9회 정차 중 4회가 가짜 lead로 무장 실패),
    // 앞차가 출발해도 plan이 같은 식으로 열려 출발 알림으로는 옳다.
    if (!armed) {
        if (std::isfinite(input.plan_distance_m) && input.plan_distance_m > kOpenPlanDistanceM)
            short_plan_since_s = -1.0;
        else if (model_stopped && short_plan_since_s < 0.0)
            short_plan_since_s = input.now_s;
        armed = model_stopped && elapsed(input.now_s, short_plan_since_s, kGreenLightArmTimeS);
    }

    bool changed = false;
    const bool road_open = armed && input.plan_distance_m > kOpenPlanDistanceM;
    if (road_open) {
        if (candidate_since_s < 0.0) candidate_since_s = input.now_s;
        changed = elapsed(input.now_s, candidate_since_s, kGreenLightConfirmTimeS);
    } else {
        candidate_since_s = -1.0;
    }
    // 길이 열리기 전에 모델이 출발을 내다보면(2초 가속 확률) 그것으로도 알린다. 방향지시등을 켜고
    // 기다리는 동안은 쓰지 않는다: 2026-10-05 좌회전 대기에서 직진 신호를 보고 0.33까지 올라 37초
    // 일찍 울렸다(대기 내내 0.10~0.30). 그때는 plan이 열릴 때만 알린다.
    gas_press_frames = armed && !input.turn_signal_on && std::isfinite(input.gas_press_prob) &&
                               input.gas_press_prob >= kGasPressGoProbability
                           ? gas_press_frames + 1
                           : 0;
    return changed || gas_press_frames >= kGasPressGoFrames;
}

bool DepartureAlertDetector::GreenLightTracker::open_for(double now_s, double duration_s) const {
    return elapsed(now_s, candidate_since_s, duration_s);
}

void DepartureAlertDetector::reset_cycle() {
    consumed_ = false;
    stopped_since_s_ = -1.0;
    close_lead_ = CloseLeadGuard{};
    lead_ = LeadDepartureTracker{};
    green_light_ = GreenLightTracker{};
}

void DepartureAlertDetector::trigger(DepartureAlertType type, double now_s) {
    active_type_ = type;
    active_until_s_ = now_s + kAlertDisplayTimeS;
    consumed_ = true;
    if (++event_id_ == 0) ++event_id_;
    lead_ = LeadDepartureTracker{};
    green_light_ = GreenLightTracker{};
}
