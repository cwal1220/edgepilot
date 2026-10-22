// DepartureAlertDetector: 정차 중 앞차 출발(lead_departed)과 신호 대기 뒤 길이 열릴 때
// (green_light) 알림. 입력은 0.05~0.1 s 틱으로 합성한다.
#include "controls/departure_alert.h"

#include <gtest/gtest.h>

namespace {

DepartureAlertInput stopped_input(double now_s) {
    DepartureAlertInput input;
    input.now_s = now_s;
    input.vehicle_valid = true;
    input.gear = 5;
    input.speed_mps = 0.0f;
    return input;
}

TEST(DepartureAlert, LeadDeparture) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;

    for (int i = 0; i <= 20; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.1);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 5.0f + (i % 2) * 0.2f;
        input.lead_relative_speed_mps = 0.2f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::None) << "멈춘 앞차의 거리 떨림으로는 알림이 뜨지 않는다";

    for (int i = 21; i <= 25; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.1);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 5.6f;
        input.lead_relative_speed_mps = 1.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::LeadDeparted) << "앞차가 출발하면 알림이 뜬다";
    ASSERT_EQ(output.event_id, 1u) << "첫 출발 알림의 이벤트 id";

    for (int i = 26; i <= 50; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.1);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 10.0f;
        input.lead_relative_speed_mps = 3.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.event_id, 1u) << "정차 한 번에 앞차 알림은 한 번만 뜬다";
}

// 정차하고 모델이 여기서 멈추겠다고 계획한 지 1.5 s 뒤 무장한다. 길이
// 열리면(plan > 10 m, 0.3 s) 알림이 뜬다.
TEST(DepartureAlert, GreenLight) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;

    for (int i = 0; i < 30; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 4.0f;
        output = detector.update(input);
    }
    ASSERT_FALSE(output.green_light_armed) << "신호 대기 검출은 짧은 plan 1.5초 전에는 무장하지 않는다";

    DepartureAlertInput armed_input = stopped_input(1.5);
    armed_input.model_updated = true;
    armed_input.model_valid = true;
    armed_input.plan_distance_m = 4.0f;
    output = detector.update(armed_input);
    ASSERT_TRUE(output.green_light_armed) << "짧은 plan이 1.5초 이어지면 무장한다";

    for (int i = 31; i <= 38; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 9.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::None) << "plan이 열림 기준보다 짧으면 알림이 뜨지 않는다";

    for (int i = 39; i <= 46; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 11.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::GreenLight) << "무장한 뒤 모델 경로가 열리면 알림이 뜬다";
    ASSERT_EQ(output.event_id, 1u) << "첫 신호 알림의 이벤트 id";
}

// 2026-10-04 실차: 정차 2.7초 만에 녹색이 켜져 길이 열렸다. 정차 3초를 기다리던 예전 규칙은
// 무장하지 못해 알림을 놓쳤다.
TEST(DepartureAlert, GreenSoonAfterStop) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;

    for (int i = 0; i < 54; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 3.0f;
        output = detector.update(input);
    }
    for (int i = 54; i <= 62; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 50.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::GreenLight);
}

// 무장은 정차 시간이 아니라 짧은 plan이 이어진 시간으로 잰다. 모델 입력 없이 서 있던 시간은 세지
// 않고, 무장 전에 길이 열리면(막 서서 아직 출발 계획, 한 프레임 튐) 처음부터 다시 잰다.
TEST(DepartureAlert, ArmingCountsShortPlanTime) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;

    for (int i = 0; i < 60; ++i) output = detector.update(stopped_input(i * 0.05));
    auto model = [&](double t, float plan) {
        DepartureAlertInput input = stopped_input(t);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = plan;
        return detector.update(input);
    };
    output = model(3.0, 4.0f);
    ASSERT_FALSE(output.green_light_armed) << "첫 짧은 plan만으로는 무장하지 않는다";
    for (int i = 1; i < 20; ++i) output = model(3.0 + i * 0.05, 4.0f);
    output = model(4.0, 12.0f);  // 무장 전 한 프레임 열림: 다시 잰다
    ASSERT_EQ(output.type, DepartureAlertType::None);
    for (int i = 1; i < 30; ++i) output = model(4.0 + i * 0.05, 4.0f);
    ASSERT_FALSE(output.green_light_armed) << "열린 뒤 짧은 plan이 1.5초 안 됐다";
    output = model(5.6, 4.0f);
    ASSERT_TRUE(output.green_light_armed);
    output = model(5.65, 7.0f);  // 5~10 m 흔들림은 무장을 풀지 않는다
    ASSERT_TRUE(output.green_light_armed);
}

// 무장(짧은 plan 1.5초) 뒤 모델의 2초 가속 확률이 두 프레임 연속 0.3을 넘으면 길이 열리기 전이라도
// 신호 알림이 뜬다. 한 프레임 튐이나 무장 전의 높은 확률로는 뜨지 않는다.
TEST(DepartureAlert, GasPressTriggersGreenLight) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;
    auto model = [&](double t, float gas) {
        DepartureAlertInput input = stopped_input(t);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 2.0f;
        input.gas_press_prob = gas;
        return detector.update(input);
    };
    output = model(0.0, 0.9f);
    output = model(0.05, 0.9f);
    ASSERT_EQ(output.type, DepartureAlertType::None) << "무장 전에는 가속 확률로 알리지 않는다";
    for (int i = 2; i <= 30; ++i) output = model(i * 0.05, 0.1f);
    ASSERT_TRUE(output.green_light_armed);
    output = model(1.55, 0.45f);
    output = model(1.6, 0.1f);
    output = model(1.65, 0.45f);
    ASSERT_EQ(output.type, DepartureAlertType::None) << "끊긴 한 프레임씩은 거른다";
    output = model(1.7, 0.45f);
    ASSERT_EQ(output.type, DepartureAlertType::GreenLight) << "두 프레임 연속이면 알린다";
    ASSERT_EQ(output.event_id, 1u);
}

// 정체(앞차 있음)에서도 무장은 된다(094 lead 확률은 앞차 유무를 가르지
// 못한다). plan이 닫힌 채 앞차만 출발하면 lead_departed가 뜬다.
TEST(DepartureAlert, QueueKeepsLeadAlert) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;

    for (int i = 0; i <= 60; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 6.0f;
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 3.0f;
        output = detector.update(input);
    }
    ASSERT_TRUE(output.green_light_armed) << "앞차가 있어도 무장한다";

    for (int i = 61; i <= 70; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 7.0f;
        input.lead_relative_speed_mps = 1.0f;
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 3.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::LeadDeparted) << "plan이 닫힌 채여도 앞차 출발 알림은 뜬다";
}

// 무장 뒤 앞차가 끼어들어도 무장은 유지된다. 그 앞차가 출발하며 plan도
// 같은 프레임에 열리면 더 구체적인 사유인 lead_departed가 뜬다.
TEST(DepartureAlert, LeadDepartureWinsWhenPlanOpens) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;

    for (int i = 0; i <= 61; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 4.0f;
        output = detector.update(input);
    }
    ASSERT_TRUE(output.green_light_armed) << "앞차 없는 정차에서 먼저 신호 알림을 무장한다";

    for (int i = 62; i <= 90; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 6.0f;
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 4.0f;
        output = detector.update(input);
    }
    ASSERT_TRUE(output.green_light_armed) << "앞차가 끼어들어도 신호 알림 무장은 유지된다";

    for (int i = 91; i <= 100; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 7.0f;
        input.lead_relative_speed_mps = 1.0f;
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = 30.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::LeadDeparted) << "같은 프레임에 plan도 열리면 앞차 출발이 이긴다";
}

// 무장된 정차에서 3 m 앞 vision 앞차가 서 있으면(거리는 흔들려도 상대속도 0) plan이 0.5초 열렸다
// 닫혀도 알리지 않는다(2026-10-04 낮 주행 23초 이른 오경보). 1.5초 넘게 계속 열리면 녹색으로 알린다.
TEST(DepartureAlert, CloseStationaryLeadNeedsSustainedOpenPlan) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;
    auto tick = [&](double t, float plan, float lead_d, float rel) {
        DepartureAlertInput input = stopped_input(t);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = lead_d;
        input.lead_relative_speed_mps = rel;
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = plan;
        input.gas_press_prob = 0.5f;  // 앞차가 서 있는 동안 가속 확률은 쓰지 않는다
        return detector.update(input);
    };
    int i = 0;
    for (; i <= 40; ++i) output = tick(i * 0.05, 2.0f, 3.0f + (i % 10) * 0.08f, 0.0f);
    ASSERT_TRUE(output.green_light_armed);
    ASSERT_EQ(output.type, DepartureAlertType::None) << "서 있는 앞차 뒤에서는 가속 확률로 알리지 않는다";
    for (int k = 0; k < 10; ++k, ++i) output = tick(i * 0.05, 30.0f, 3.5f, 0.05f);
    ASSERT_EQ(output.type, DepartureAlertType::None) << "0.5초 열림으로는 알리지 않는다";
    for (int k = 0; k < 20; ++k, ++i) output = tick(i * 0.05, 2.0f, 3.2f, 0.0f);
    ASSERT_EQ(output.type, DepartureAlertType::None);
    for (int k = 0; k < 29; ++k, ++i) output = tick(i * 0.05, 30.0f, 3.2f, 0.0f);
    ASSERT_EQ(output.type, DepartureAlertType::None) << "1.5초가 되기 전에는 기다린다";
    for (int k = 0; k < 2; ++k, ++i) output = tick(i * 0.05, 30.0f, 3.2f, 0.0f);
    ASSERT_EQ(output.type, DepartureAlertType::GreenLight) << "1.5초 넘게 열리면 알린다";
}

// 서 있는 앞차 뒤에서 plan이 열린 채 앞차가 실제로 멀어지면(거리 +0.5 m, 상대속도 0.5 m/s 초과)
// 바로 알린다.
TEST(DepartureAlert, CloseLeadMovingReleasesGreenLight) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;
    auto tick = [&](double t, float plan, float lead_d, float rel) {
        DepartureAlertInput input = stopped_input(t);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = lead_d;
        input.lead_relative_speed_mps = rel;
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = plan;
        return detector.update(input);
    };
    int i = 0;
    for (; i <= 40; ++i) output = tick(i * 0.05, 2.0f, 3.0f, 0.0f);
    for (int k = 0; k < 8; ++k, ++i) output = tick(i * 0.05, 40.0f, 3.1f, 0.1f);
    ASSERT_EQ(output.type, DepartureAlertType::None) << "앞차가 서 있으면 0.3초 열림으로는 알리지 않는다";
    output = tick(i++ * 0.05, 40.0f, 3.7f, 0.8f);
    ASSERT_EQ(output.type, DepartureAlertType::GreenLight) << "앞차가 움직이면 바로 알린다";
}

// 방향지시등을 켜고 기다리는 동안은 가속 확률로 알리지 않는다(2026-10-05 좌회전 대기 37초 이른
// 오경보). plan이 열리면 평소대로 알린다.
TEST(DepartureAlert, TurnSignalIgnoresGasPress) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;
    auto tick = [&](double t, float plan, float gas) {
        DepartureAlertInput input = stopped_input(t);
        input.model_updated = true;
        input.model_valid = true;
        input.plan_distance_m = plan;
        input.gas_press_prob = gas;
        input.turn_signal_on = true;
        return detector.update(input);
    };
    int i = 0;
    for (; i <= 40; ++i) output = tick(i * 0.05, 2.0f, 0.1f);
    ASSERT_TRUE(output.green_light_armed);
    for (int k = 0; k < 10; ++k, ++i) output = tick(i * 0.05, 2.0f, 0.4f);
    ASSERT_EQ(output.type, DepartureAlertType::None) << "깜빡이 대기 중 가속 확률은 쓰지 않는다";
    for (int k = 0; k < 8; ++k, ++i) output = tick(i * 0.05, 30.0f, 0.4f);
    ASSERT_EQ(output.type, DepartureAlertType::GreenLight) << "plan이 열리면 알린다";
}

// vision 앞차가 한 프레임 끊겨도 앞차 출발 무장은 이어진다(예전에는 처음부터 다시 쟀다).
TEST(DepartureAlert, LeadDropoutKeepsArming) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;
    for (int i = 0; i <= 40; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.lead_updated = true;
        input.lead_valid = i % 8 != 7;  // 0.4초마다 한 프레임 끊긴다
        input.lead_distance_m = input.lead_valid ? 5.0f : 0.0f;
        output = detector.update(input);
    }
    for (int i = 41; i <= 50; ++i) {
        DepartureAlertInput input = stopped_input(i * 0.05);
        input.lead_updated = true;
        input.lead_valid = true;
        input.lead_distance_m = 5.6f;
        input.lead_relative_speed_mps = 1.0f;
        output = detector.update(input);
    }
    ASSERT_EQ(output.type, DepartureAlertType::LeadDeparted) << "짧은 끊김을 넘겨 무장했으므로 출발을 알린다";
}

// controlsd 주기: 100 Hz 틱에 모델은 5틱마다. 무장(짧은 plan 1.5초)과 녹색 확인(열림 0.3초)은 모델
// 프레임에서만 판단해, 알림은 열린 plan을 처음 본 모델 프레임 0.3초 뒤의 모델 프레임에 뜬다.
TEST(DepartureAlert, GreenLightAtControlRateWithModelFrames) {
    DepartureAlertDetector detector;
    int alert_tick = -1;
    for (int tick = 0; tick < 600 && alert_tick < 0; ++tick) {
        DepartureAlertInput input = stopped_input(tick * 0.01);
        input.model_updated = tick % 5 == 0;
        input.model_valid = true;
        input.plan_distance_m = tick < 250 ? 2.0f : 30.0f;
        if (detector.update(input).type == DepartureAlertType::GreenLight) alert_tick = tick;
    }
    ASSERT_GE(alert_tick, 280) << "열린 plan을 0.3초 본 뒤에";
    ASSERT_LE(alert_tick, 285);
    ASSERT_EQ(alert_tick % 5, 0) << "모델 프레임에서 뜬다";
}

// 가까운 앞차 거부는 모델 프레임과 무관하게 매 틱 판단해, plan이 1.5초 넘게 열린 순간 모델 프레임
// 사이에서도 알린다(모델을 7틱마다 줘 1.5초 지점이 모델 프레임이 아니게 한다).
TEST(DepartureAlert, CloseLeadVetoDecidesBetweenModelFrames) {
    DepartureAlertDetector detector;
    int open_seen_tick = -1, alert_tick = -1;
    for (int tick = 0; tick < 1000 && alert_tick < 0; ++tick) {
        DepartureAlertInput input = stopped_input(tick * 0.01);
        const bool model_frame = tick % 7 == 0;
        input.lead_updated = model_frame;
        input.lead_valid = true;  // controlsd는 신선한 모델의 앞차를 매 틱 유효로 넘긴다
        input.lead_distance_m = 4.0f;
        input.model_updated = model_frame;
        input.model_valid = true;
        input.plan_distance_m = tick < 250 ? 2.0f : 30.0f;
        if (model_frame && tick >= 250 && open_seen_tick < 0) open_seen_tick = tick;
        if (detector.update(input).type == DepartureAlertType::GreenLight) alert_tick = tick;
    }
    ASSERT_GE(open_seen_tick, 250);
    ASSERT_GE(alert_tick, open_seen_tick + 149);
    ASSERT_LE(alert_tick, open_seen_tick + 151) << "열린 plan을 처음 본 지 1.5초에";
    ASSERT_NE(alert_tick % 7, 0) << "모델 프레임 사이의 틱에서 뜬다";
}

// 서행(0.1~0.5 m/s)은 이번 정차를 끝내지 않아 한 번 알린 정차에서는 다시 알리지 않는다. 가속 페달이나
// 0.5 m/s 넘는 출발이 정차를 끝내고, 다음 정차에서 다시 알린다(이벤트 id 2).
TEST(DepartureAlert, CreepingKeepsTheStopUntilTheDriverDrivesOff) {
    DepartureAlertDetector detector;
    DepartureAlertOutput output;
    double t = 0.0;
    auto stop_and_open = [&](float speed, bool gas = false) {
        for (int i = 0; i < 40; ++i, t += 0.05) {
            DepartureAlertInput input = stopped_input(t);
            input.speed_mps = speed;
            input.gas_pressed = gas;
            input.model_updated = true;
            input.model_valid = true;
            input.plan_distance_m = 2.0f;
            output = detector.update(input);
        }
        for (int i = 0; i < 10; ++i, t += 0.05) {
            DepartureAlertInput input = stopped_input(t);
            input.speed_mps = speed;
            input.gas_pressed = gas;
            input.model_updated = true;
            input.model_valid = true;
            input.plan_distance_m = 30.0f;
            output = detector.update(input);
        }
    };
    stop_and_open(0.0f);
    ASSERT_EQ(output.event_id, 1u);
    stop_and_open(0.3f);  // 서행
    stop_and_open(0.0f);
    ASSERT_EQ(output.event_id, 1u) << "서행 뒤 같은 정차에서는 다시 알리지 않는다";
    stop_and_open(0.0f, true);  // 가속 페달: 정차를 끝낸다
    stop_and_open(0.0f);
    ASSERT_EQ(output.event_id, 2u) << "다음 정차에서는 다시 알린다";
}

}  // namespace
