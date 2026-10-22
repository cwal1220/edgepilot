// AdaptiveCruiseController: 비전 앞차에 맞춰 SET/RES 버튼을 흉내 내는 크루즈(이 차에는 SCC가
// 없다). 차량 모형이 버튼 펄스를 실제 설정 속도 변화로 바꿔 돌려준다.
#include "controls/adaptive_cruise.h"

#include <gtest/gtest.h>
#include <cmath>

namespace {

// 차량 모형: 버튼 펄스가 실제 설정 속도를 움직이고 클러스터 속도가 그 뒤를 따른다.
// 설정 속도를 고정해 두면 설정 속도 동기화 결함을 잡지 못한다.
struct Vehicle {
    float set_speed_kph = 80.0f;
    float cluster_speed_kph = 80.0f;
    float step_kph = 2.0f;
    // 클러스터가 휠보다 높게 읽는 비(K7 실측 1.066).
    float over_read = 1.066f;
    int previous_button = 0;
    int previous_driver_button = 0;
    bool cruise_on = false;
    // 이 값마다 버튼 하나를 흘린다. 0이면 흘리지 않는다.
    int drop_every = 0;
    int presses = 0;

    // 버튼은 펄스의 상승 엣지당 한 번만 먹는다. 스톡 스토크와 같다.
    void apply(int button) {
        if (button == 0 || previous_button != 0) {
            previous_button = button;
            return;
        }
        previous_button = button;
        ++presses;
        if (drop_every != 0 && presses % drop_every == 0) return;
        if (button == 2) set_speed_kph = std::fmax(30.0f, set_speed_kph - step_kph);
        if (button == 1) set_speed_kph += step_kph;
    }
    // 운전자 버튼도 같은 차량에 작용한다.
    void driver_press(int button) {
        if (button != 0 && previous_driver_button == 0) {
            // 고정형 크루즈와 같다: 꺼져 있을 때의 SET은 현재 속도를 잡고,
            // 켜져 있을 때의 SET은 한 스텝 낮춘다 (vehicle_can.cc의 추정기와 동일).
            if (button == 2) {
                set_speed_kph =
                    cruise_on ? std::fmax(30.0f, set_speed_kph - step_kph) : std::fmax(30.0f, cluster_speed_kph);
                cruise_on = true;
            }
            if (button == 1) {
                if (cruise_on) set_speed_kph += step_kph;
                cruise_on = true;
            }
        }
        previous_driver_button = button;
    }
    // 클러스터 속도는 설정 속도를 따라간다. 즉시가 아니다.
    void settle(float dt_s) { cluster_speed_kph += (set_speed_kph - cluster_speed_kph) * dt_s * 0.8f; }
    // K7 HEV 실측에 가까운 응답: 설정 속도보다 빠르면 가속만 끊어 초당 coast_kph_s만큼 줄고(브레이크를
    // 쓰지 않는다), 느리면 초당 0.5 km/h만큼 는다. settle보다 훨씬 느리다.
    void coast(float dt_s, float coast_kph_s = 0.5f) {
        if (cluster_speed_kph > set_speed_kph)
            cluster_speed_kph = std::fmax(set_speed_kph, cluster_speed_kph - coast_kph_s * dt_s);
        else
            cluster_speed_kph = std::fmin(set_speed_kph, cluster_speed_kph + 0.5f * dt_s);
    }
    float wheel_kph() const { return cluster_speed_kph / over_read; }
};

AdaptiveCruiseInput base_input(double now_s) {
    AdaptiveCruiseInput input;
    input.now_s = now_s;
    input.controls_ready = true;
    input.cruise_active = true;
    input.ego_speed_kph = 80.0f;
    input.cluster_speed_kph = 80.0f;
    input.driver_set_speed_kph = 80.0f;
    return input;
}

// 한 틱: 차량이 보고하는 설정 속도를 넣고, 나온 버튼을 차량에 반영한다.
AdaptiveCruiseOutput tick(AdaptiveCruiseController *controller, AdaptiveCruiseInput input, Vehicle *vehicle) {
    // driver_set_speed_kph 는 차량 보고가 아니라 운전자 조작만 반영하는 추정치다
    // (이 차에는 SCC가 없다). 우리 명령은 여기에 절대 나타나지 않는다.
    vehicle->driver_press(input.driver_button);
    input.driver_set_speed_kph = vehicle->set_speed_kph;
    input.ego_speed_kph = vehicle->wheel_kph();
    input.cluster_speed_kph = vehicle->cluster_speed_kph;
    const AdaptiveCruiseOutput output = controller->update(input);
    vehicle->apply(output.command_button);
    return output;
}

AdaptiveCruiseOutput activate(AdaptiveCruiseController *controller, Vehicle *vehicle) {
    AdaptiveCruiseInput input = base_input(0.0);
    input.driver_button = 2;
    return tick(controller, input, vehicle);
}

TEST(AdaptiveCruise, CloseLeadSetsAndRepeatsEveryInterval) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    AdaptiveCruiseOutput output = activate(&controller, &vehicle);
    // 첫 SET은 최대·명령 속도를 현재 속도로 잡는다
    ASSERT_TRUE(output.session_valid);
    ASSERT_TRUE(output.active);
    ASSERT_NEAR(output.maximum_speed_kph, 80.0f, 0.001f);
    ASSERT_NEAR(output.commanded_speed_kph, 80.0f, 0.001f);

    AdaptiveCruiseInput input = base_input(0.01);
    tick(&controller, input, &vehicle);
    input.now_s = 0.5;
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    input.vision_lead_distance_m = 12.0f;
    input.vision_lead_relative_speed_mps = -4.0f;
    output = tick(&controller, input, &vehicle);
    // SET 뒤 1초 동안은 명령을 내지 않는다
    ASSERT_EQ(output.command_button, 0);
    ASSERT_NEAR(output.commanded_speed_kph, 80.0f, 0.001f);

    input.now_s = 1.0;
    output = tick(&controller, input, &vehicle);
    // 가까운 느린 앞차에는 SET 명령을 한 번 낸다
    ASSERT_EQ(output.command_button, 2);
    ASSERT_NEAR(output.commanded_speed_kph, 78.0f, 0.001f);
    ASSERT_LT(output.target_speed_kph, output.commanded_speed_kph);

    input.vision_lead_updated = false;
    for (int frame = 1; frame < 5; ++frame) {
        input.now_s = 1.0 + frame * 0.01;
        output = tick(&controller, input, &vehicle);
        ASSERT_EQ(output.command_button, 2) << "SET 명령은 5프레임 펄스로 나간다";
    }
    input.now_s = 1.05;
    output = tick(&controller, input, &vehicle);
    // 버튼 펄스는 5프레임 뒤 멈춘다
    ASSERT_EQ(output.command_button, 0);
    ASSERT_NEAR(output.commanded_speed_kph, 78.0f, 0.001f);

    input.now_s = 1.9;
    input.vision_lead_updated = true;
    output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 0) << "명령 시작은 1초에 한 번으로 제한된다";
    input.now_s = 2.0;
    output = tick(&controller, input, &vehicle);
    // 차가 아직 줄지 않으면(여기서는 속도 고정) 다음 간격에 다음 스텝을 요청한다
    ASSERT_EQ(output.command_button, 2);
    ASSERT_NEAR(output.commanded_speed_kph, 76.0f, 0.001f);
}

TEST(AdaptiveCruise, ClosingLeadPredictionAndResumeDelay) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);

    AdaptiveCruiseInput input = base_input(1.0);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    input.vision_lead_distance_m = 50.0f;
    input.vision_lead_relative_speed_mps = -5.0f;
    AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
    ASSERT_LT(output.target_speed_kph, 64.0f) << "다가오는 앞차 거리는 명령 응답 시점으로 예측한다";
    // 예측상 가까워지는 앞차에는 SET을 요청한다
    ASSERT_EQ(output.command_button, 2);
    ASSERT_NEAR(output.commanded_speed_kph, 78.0f, 0.001f);

    input.vision_lead_updated = false;
    for (int frame = 1; frame < 5; ++frame) {
        input.now_s = 1.0 + frame * 0.01;
        tick(&controller, input, &vehicle);
    }

    // 앞차가 멀어진다. 버튼 판단용 목표는 초당 1 km/h만 올라(앞차 잡음으로 SET-/RES+가 번갈아
    // 나가지 않게), 명령 속도 한 스텝 아래에서 출발해 약 4초 뒤 RES를 낸다.
    input.vision_lead_distance_m = 150.0f;
    input.vision_lead_relative_speed_mps = 40.0f;
    double resume_s = -1.0;
    for (int i = 0; i <= 600; ++i) {
        input.now_s = 2.0 + i * 0.01;
        input.vision_lead_updated = (i % 5 == 0);
        output = tick(&controller, input, &vehicle);
        if (output.command_button == 1 && resume_s < 0.0) resume_s = input.now_s;
        if (input.now_s < 3.0) ASSERT_EQ(output.command_button, 0) << "RES는 방금 낸 SET을 바로 되돌리지 않는다";
    }
    ASSERT_GT(resume_s, 5.5);
    ASSERT_LT(resume_s, 6.5) << "목표가 다 오르면 RES로 속도를 되돌린다";
    ASSERT_NEAR(output.commanded_speed_kph, 80.0f, 0.001f);
}

TEST(AdaptiveCruise, LeadLossHoldsThenRestoresMaximum) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);
    AdaptiveCruiseInput input = base_input(1.0);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    input.vision_lead_distance_m = 10.0f;
    input.vision_lead_relative_speed_mps = -5.0f;
    AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
    // 복원 검사 전에 앞차로 명령 속도를 낮춰 둔다
    ASSERT_EQ(output.command_button, 2);
    ASSERT_NEAR(output.commanded_speed_kph, 78.0f, 0.001f);
    input.vision_lead_updated = false;
    for (int frame = 1; frame < 5; ++frame) {
        input.now_s = 1.0 + frame * 0.01;
        tick(&controller, input, &vehicle);
    }

    input.vision_lead_updated = true;
    input.vision_lead_valid = false;
    input.now_s = 2.0;
    output = tick(&controller, input, &vehicle);
    // 앞차를 잠깐 놓치면 낮춘 설정을 유지한다
    ASSERT_EQ(output.command_button, 0);
    ASSERT_NEAR(output.commanded_speed_kph, 78.0f, 0.001f);

    // 앞차가 계속 없으면 복원 지연(2초) 뒤 목표가 최대 속도 쪽으로 천천히 올라 RES를 낸다
    double resume_s = -1.0;
    for (int i = 1; i <= 800; ++i) {
        input.now_s = 2.0 + i * 0.01;
        output = tick(&controller, input, &vehicle);
        if (output.command_button == 1 && resume_s < 0.0) resume_s = input.now_s;
    }
    ASSERT_GT(resume_s, 3.0) << "복원 지연 전에는 되돌리지 않는다";
    ASSERT_LT(resume_s, 7.5);
    ASSERT_NEAR(output.commanded_speed_kph, 80.0f, 0.001f);
}

TEST(AdaptiveCruise, DriverAndPedalGates) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);
    AdaptiveCruiseInput input = base_input(1.0);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    input.vision_lead_distance_m = 8.0f;
    input.vision_lead_relative_speed_mps = -6.0f;
    input.brake_pressed = true;
    AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 0) << "브레이크를 밟으면 버튼 출력을 막는다";

    input.brake_pressed = false;
    input.gas_pressed = true;
    input.now_s = 2.0;
    output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 0) << "가속 페달을 밟으면 버튼 출력을 막는다";

    input.gas_pressed = false;
    input.now_s = 2.49;
    output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 0) << "가속 페달을 뗀 직후에는 출력을 기다린다";

    input.now_s = 2.5;
    output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 2) << "가속 페달 해제 지연이 지나면 출력을 재개한다";

    input.now_s = 2.6;
    input.driver_accelerator_override = true;
    output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 0) << "TCS 운전자 개입은 진행 중인 버튼 펄스를 취소한다";

    // 운전자가 RES로 설정 속도를 올린다. 천장은 버튼을 뗀 뒤 차량이 정착하고
    // 나서 실측 속도로 잡는다 — 누르는 도중의 값은 아직 이동 중이다.
    input.driver_accelerator_override = false;
    input.vision_lead_valid = false;  // 앞차를 치워 천장 동작만 본다
    input.driver_button = 1;
    input.now_s = 3.1;
    output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 0) << "운전자가 버튼을 누르는 동안 자동 출력을 멈춘다";

    input.driver_button = 0;
    for (int frame = 1; frame <= 600; ++frame) {
        input.now_s = 3.1 + frame * 0.01;
        output = tick(&controller, input, &vehicle);
        vehicle.settle(0.01f);
    }
    // 지수 수렴이라 정착 판정 시점에 한 스텝의 1/6 정도가 남는다. 명령 단위가
    // 2 km/h이므로 실질 영향은 없다.
    // 앞 단계에서 자동 SET-이 한 번 나가 80 -> 78이 됐으므로 운전자 RES 후는 80이다.
    {
        // 차 속도가 자리 잡으면 운전자 RES가 상한을 올린다
        ASSERT_NEAR(output.maximum_speed_kph, 80.0f, 0.5f);
        ASSERT_NEAR(output.commanded_speed_kph, 80.0f, 0.5f);
    }
}

TEST(AdaptiveCruise, SessionResetAndMinimumSpeed) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    vehicle.set_speed_kph = 20.0f;
    vehicle.cluster_speed_kph = 20.0f;
    AdaptiveCruiseInput input = base_input(0.0);
    input.driver_button = 2;
    AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
    // km/h 설정 속도는 순정 크루즈 최저 속도 밑으로 내려가지 않는다
    ASSERT_TRUE(output.session_valid);
    ASSERT_NEAR(output.maximum_speed_kph, 30.0f, 0.001f);

    input.driver_button = 0;
    input.driver_main_button = 1;
    input.now_s = 0.1;
    output = tick(&controller, input, &vehicle);
    // 메인 버튼은 세션을 지운다
    ASSERT_FALSE(output.session_valid);
    ASSERT_FALSE(output.active);
    ASSERT_EQ(output.command_button, 0);

    AdaptiveCruiseController imperial_controller;
    Vehicle imperial_vehicle;
    imperial_vehicle.set_speed_kph = 25.0f;
    imperial_vehicle.cluster_speed_kph = 25.0f;
    input = base_input(0.0);
    input.speed_unit_mph = true;
    input.driver_button = 2;
    output = tick(&imperial_controller, input, &imperial_vehicle);
    // mph 설정 속도는 순정 크루즈 최저 20 mph를 쓴다
    ASSERT_TRUE(output.session_valid);
    ASSERT_NEAR(output.maximum_speed_kph, 20.0f * 1.609344f, 0.001f);

    input.enabled = false;
    input.driver_button = 0;
    input.now_s = 0.1;
    output = tick(&imperial_controller, input, &imperial_vehicle);
    // 기능을 끄면 세션과 출력을 지운다
    ASSERT_FALSE(output.session_valid);
    ASSERT_EQ(output.command_button, 0);
}

TEST(AdaptiveCruise, RuntimeConfigUpdate) {
    AdaptiveCruiseParams config;
    config.command_interval_s = 5.0f;
    AdaptiveCruiseController controller(config);
    Vehicle vehicle;
    activate(&controller, &vehicle);

    AdaptiveCruiseInput input = base_input(1.0);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    input.vision_lead_distance_m = 8.0f;
    input.vision_lead_relative_speed_mps = -6.0f;
    AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
    // 가까운 앞차지만 세션 시작 뒤 명령 간격(5초)이 아직 지나지 않았다
    ASSERT_TRUE(output.lead_valid);
    ASSERT_EQ(output.command_button, 0);

    config.command_interval_s = 0.5f;
    config.button_pulse_frames = 1;
    controller.update_params(config);
    input.now_s = 1.01;
    output = tick(&controller, input, &vehicle);
    // 런타임 설정 변경은 컨트롤러를 다시 만들지 않아도 적용된다
    ASSERT_EQ(output.command_button, 2);

    input.vision_lead_updated = false;
    input.now_s = 1.02;
    output = tick(&controller, input, &vehicle);
    ASSERT_EQ(output.command_button, 0) << "1프레임 펄스 설정은 다음 제어 틱에 끝난다";
}

// 이 차에는 SCC가 없어 설정 속도를 CAN으로 읽을 수 없다(2026-09-13 주행 실측:
// SCC11/SCC12 수신 0건). driver_set_speed_kph 는 운전자 조작만 반영하는
// 추정치이고 우리 명령은 절대 나타나지 않는다. 아래 회귀는 그 전제 위에서
// 실제로 보고된 증상과 리뷰에서 나온 결함을 각각 고정한다.

// 정속 추종: 차간이 맞으면 설정 속도를 건드리지 않아야 한다. 예전에는 차간·
// 상대속도(휠 기준)를 설정 속도(클러스터 기준)와 그대로 비교해, 실측 6.6%
// 차이만으로 SET-이 끝없이 나가 30 km/h 바닥까지 내려갔다.
TEST(AdaptiveCruise, SteadyFollowingIssuesNoCommands) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);
    int commands = 0;
    AdaptiveCruiseOutput output;
    for (int i = 1; i <= 12000; ++i) {
        AdaptiveCruiseInput input = base_input(i * 0.01);
        input.vision_lead_updated = true;
        input.vision_lead_valid = true;
        const float wheel_mps = vehicle.wheel_kph() / 3.6f;
        input.vision_lead_distance_m = 5.0f + 1.8f * wheel_mps;
        input.vision_lead_relative_speed_mps = 0.0f;
        output = tick(&controller, input, &vehicle);
        vehicle.settle(0.01f);
        if (output.command_button != 0) ++commands;
    }
    ASSERT_NEAR(vehicle.set_speed_kph, 80.0f, 0.001f) << "안정적으로 따라갈 때는 차의 설정 속도를 건드리지 않는다";
    ASSERT_EQ(commands, 0) << "안정적으로 따라갈 때는 버튼 명령을 내지 않는다";
}

// 해제 뒤 재설정: 이전 주행의 천장이 남으면 앞차가 없는데도 RES+로 속도를
// 끌어올린다. 고속도로 100 -> 브레이크 -> 시내 60 재설정이 그 경로였다.
TEST(AdaptiveCruise, ReengageDoesNotInheritTheOldCeiling) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    vehicle.set_speed_kph = 100.0f;
    vehicle.cluster_speed_kph = 100.0f;
    activate(&controller, &vehicle);
    double t = 0.0;
    for (int i = 0; i < 300; ++i) {
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.vision_lead_updated = true;
        tick(&controller, input, &vehicle);
    }
    for (int i = 0; i < 100; ++i) {  // 브레이크: 크루즈 해제
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.cruise_active = false;
        input.brake_pressed = true;
        tick(&controller, input, &vehicle);
    }
    vehicle.cruise_on = false;
    vehicle.set_speed_kph = 60.0f;
    vehicle.cluster_speed_kph = 60.0f;
    AdaptiveCruiseInput set_input = base_input(t += 0.01);
    set_input.driver_button = 2;
    tick(&controller, set_input, &vehicle);
    AdaptiveCruiseOutput output;
    for (int i = 0; i < 9000; ++i) {  // 90초, 앞차 없음
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.vision_lead_updated = true;
        output = tick(&controller, input, &vehicle);
        vehicle.settle(0.01f);
    }
    ASSERT_NEAR(output.maximum_speed_kph, 60.0f, 1.0f) << "다시 켜면 예전 상한을 물려받지 않고 새로 잡는다";
    ASSERT_NEAR(vehicle.set_speed_kph, 60.0f, 1.0f) << "더 낮은 속도에서 다시 켜면 도로 올리지 않는다";
}

// 운전자가 SET-을 길게 눌러 속도를 내리는 것은 일상적인 조작이다. 이전 구현은
// 버튼 누른 순간에 천장 재설정을 무장해 0.5초 뒤 중간값에 굳히고, 그 뒤 운전자가
// 더 내린 분을 RES+로 되돌렸다.
TEST(AdaptiveCruise, HeldDriverSetIsNotUndone) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    vehicle.set_speed_kph = 100.0f;
    vehicle.cluster_speed_kph = 100.0f;
    activate(&controller, &vehicle);
    double t = 0.0;
    for (int i = 0; i < 300; ++i) {
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.vision_lead_updated = true;
        tick(&controller, input, &vehicle);
    }
    for (int i = 0; i < 400; ++i) {  // 4초 길게 누름: 차량이 연속 감속
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.driver_button = 2;
        input.vision_lead_updated = true;
        tick(&controller, input, &vehicle);
        vehicle.set_speed_kph = std::fmax(60.0f, vehicle.set_speed_kph - 0.1f);
        vehicle.cluster_speed_kph = vehicle.set_speed_kph;
    }
    AdaptiveCruiseOutput output;
    for (int i = 0; i < 9000; ++i) {
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.vision_lead_updated = true;
        output = tick(&controller, input, &vehicle);
        vehicle.settle(0.01f);
    }
    ASSERT_NEAR(vehicle.set_speed_kph, 60.0f, 1.0f) << "운전자가 눌러 둔 SET-를 나중에 도로 올리지 않는다";
    ASSERT_NEAR(output.maximum_speed_kph, 60.0f, 1.0f) << "상한은 운전자가 실제로 맞춰 둔 속도를 따른다";
}

// 차량이 버튼을 무시하면(펄스 유실, 배선 문제) 예전에는 분당 45펄스를 영원히
// 쐈다. 듣지 않는다고 판단하면 물러나야 한다.
TEST(AdaptiveCruise, BacksOffWhenTheCarIgnoresCommands) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);
    vehicle.drop_every = 1;  // 모든 버튼을 흘린다
    int presses = 0, previous = 0;
    for (int i = 1; i <= 6000; ++i) {
        AdaptiveCruiseInput input = base_input(i * 0.01);
        input.vision_lead_updated = true;
        input.vision_lead_valid = true;
        input.vision_lead_distance_m = 12.0f;
        input.vision_lead_relative_speed_mps = -5.0f;
        const AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
        if (output.command_button != 0 && previous == 0) ++presses;
        previous = output.command_button;
    }
    ASSERT_GT(presses, 0) << "반응 없는 차에도 처음에는 명령을 시도한다";
    // 처음 한 번 목표까지 누른 뒤(17번) 듣지 않는다고 보고 30초 물러난다
    ASSERT_LE(presses, 25) << "반응 없는 차를 끝없이 누르지 않는다";
}

// 클러스터/휠 비가 학습 범위를 벗어나면 두 척도를 비교할 근거가 없다.
// 그 상태에서 판단하면 원래 버그가 그대로 재현된다.
TEST(AdaptiveCruise, NoCommandsWithoutALearnedScale) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    vehicle.over_read = 1.60f;  // 범위 밖
    activate(&controller, &vehicle);
    int commands = 0;
    for (int i = 1; i <= 6000; ++i) {
        AdaptiveCruiseInput input = base_input(i * 0.01);
        input.vision_lead_updated = true;
        input.vision_lead_valid = true;
        const float wheel_mps = vehicle.wheel_kph() / 3.6f;
        input.vision_lead_distance_m = 5.0f + 1.8f * wheel_mps;
        input.vision_lead_relative_speed_mps = 0.0f;
        if (tick(&controller, input, &vehicle).command_button != 0) ++commands;
        vehicle.settle(0.01f);
    }
    ASSERT_EQ(commands, 0) << "클러스터/휠 비가 비정상이면 명령을 모두 막는다";
    ASSERT_NEAR(vehicle.set_speed_kph, 80.0f, 0.001f) << "클러스터/휠 비가 비정상이면 설정 속도를 건드리지 않는다";
}

// 차량 스텝이 2 km/h가 아니면(롱프레스형 10 km/h) 예전에는 SET-/RES+가 번갈아
// 나오며 설정 속도가 무한 왕복했다.
TEST(AdaptiveCruise, NoLimitCycleWithACoarseVehicleStep) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    vehicle.step_kph = 10.0f;
    activate(&controller, &vehicle);
    int presses = 0, previous = 0;
    for (int i = 1; i <= 6000; ++i) {
        AdaptiveCruiseInput input = base_input(i * 0.01);
        input.vision_lead_updated = true;
        input.vision_lead_valid = true;
        const float wheel_mps = vehicle.wheel_kph() / 3.6f;
        input.vision_lead_distance_m = 5.0f + 1.8f * wheel_mps;
        input.vision_lead_relative_speed_mps = 0.0f;
        const AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
        if (output.command_button != 0 && previous == 0) ++presses;
        previous = output.command_button;
        vehicle.settle(0.01f);
    }
    ASSERT_LE(presses, 2) << "차의 설정 스텝이 커도 오르내림을 반복하지 않는다";
}

// 느린 앞차는 실제로 따라가야 한다 — 위 안전장치들이 기능 자체를 죽이지
// 않았는지 확인한다.
TEST(AdaptiveCruise, SlowerLeadIsActuallyFollowed) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);
    float gap = 60.0f;
    const float lead_mps = 50.0f / 3.6f;
    for (int i = 1; i <= 18000; ++i) {  // 180초
        AdaptiveCruiseInput input = base_input(i * 0.01);
        const float ego_mps = vehicle.wheel_kph() / 3.6f;
        gap = std::fmax(1.0f, gap + (lead_mps - ego_mps) * 0.01f);
        input.vision_lead_updated = true;
        input.vision_lead_valid = true;
        input.vision_lead_distance_m = gap;
        input.vision_lead_relative_speed_mps = lead_mps - ego_mps;
        tick(&controller, input, &vehicle);
        vehicle.settle(0.01f);
    }
    const float ego_kph = vehicle.wheel_kph();
    ASSERT_NEAR(ego_kph, 50.0f, 3.0f) << "느린 앞차를 실제로 그 속도로 따라간다";
    const float desired_gap = 5.0f + 1.8f * ego_kph / 3.6f;
    ASSERT_NEAR(gap, desired_gap, 5.0f) << "느린 앞차를 원하는 간격 근처에서 따라간다";
}

// 운전자가 가속 페달로 설정 속도보다 빨리 달리는 것은 버튼 유실이 아니다. 예전에는 클러스터가 추정보다
// 5초 넘게 빠르면 "SET-이 안 먹었다"로 보고 30초 물러났고, 페달을 계속 밟는 동안 5초마다 다시 걸었다.
// 그래서 페달을 뗀 뒤 앞차에 붙는 동안에도 SET-이 나가지 않았다(2026-10-10 주행).
TEST(AdaptiveCruise, PedalOverrideIsNotAMissedCommand) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);
    double t = 0.0;
    for (int i = 0; i < 3000; ++i) {  // 30초 동안 페달로 95까지
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.gas_pressed = true;
        vehicle.cluster_speed_kph = std::fmin(95.0f, vehicle.cluster_speed_kph + 0.02f);
        ASSERT_EQ(tick(&controller, input, &vehicle).command_button, 0);
    }
    const double release_s = t;
    double first_set_s = -1.0;
    AdaptiveCruiseOutput output;
    for (int i = 0; i < 2000; ++i) {  // 페달을 떼자 느린 앞차가 가깝다
        AdaptiveCruiseInput input = base_input(t += 0.01);
        input.vision_lead_updated = true;
        input.vision_lead_valid = true;
        input.vision_lead_distance_m = 25.0f;
        input.vision_lead_relative_speed_mps = -3.0f;
        output = tick(&controller, input, &vehicle);
        vehicle.coast(0.01f, 1.5f);
        if (output.command_button == 2 && first_set_s < 0.0) first_set_s = t;
    }
    ASSERT_GT(first_set_s, 0.0) << "페달을 뗀 뒤 앞차에 SET-을 낸다";
    ASSERT_LT(first_set_s - release_s, 10.0) << "페달 사용을 이유로 30초 물러나 있지 않는다";
    ASSERT_NEAR(output.maximum_speed_kph, 80.0f, 0.01f) << "페달로 달린 속도를 상한으로 굳히지 않는다";
}

// 고정형 크루즈는 SET-에 가속만 끊어 천천히 준다(K7 실측 약 0.15 m/s²). 그래서 앞차에 다가가는 속도를
// 그 감속으로 원하는 차간 전에 없앨 수 있는 만큼(√(2·a·남은 거리))만 허용한다.
TEST(AdaptiveCruise, ApproachSpeedRespectsTheCoastingDeceleration) {
    auto target_for = [](float deceleration_kph_per_s) {
        AdaptiveCruiseParams config;
        config.deceleration_rate_kph_per_s = deceleration_kph_per_s;
        AdaptiveCruiseController controller(config);
        Vehicle vehicle;
        vehicle.set_speed_kph = vehicle.cluster_speed_kph = 100.0f;
        activate(&controller, &vehicle);
        vehicle.cluster_speed_kph = 70.0f;  // 상한(100)보다 느리게 따라가는 중
        AdaptiveCruiseInput input = base_input(1.0);
        input.vision_lead_updated = true;
        input.vision_lead_valid = true;
        input.vision_lead_distance_m = 5.0f + 1.8f * vehicle.wheel_kph() / 3.6f + 8.0f;  // 원하는 차간 + 8 m
        input.vision_lead_relative_speed_mps = 0.0f;
        return tick(&controller, input, &vehicle).target_speed_kph;
    };
    const float coasting = target_for(0.5f);  // 실측
    const float braking = target_for(5.0f);   // 브레이크가 있는 차라면
    // 비례 보정은 0.25·8 = 2.0 m/s, 감속 한계는 √(2·0.139·8) = 1.49 m/s다
    const float expected = (2.0f - std::sqrt(2.0f * 0.5f / 3.6f * 8.0f)) * 3.6f * 1.066f;
    ASSERT_NEAR(braking - coasting, expected, 0.3f) << "약한 감속이면 앞차에 더 천천히 다가간다";
}

// 차가 이미 줄고 있으면 설정 속도를 실제 속도보다 한참 아래로 보내지 않는다. 더 낮춰도 고정형 크루즈는
// 빨리 줄이지 않고, 앞차가 바뀌면 되돌릴 RES+만 늘어난다(2026-10-10 18:34 주행: 차 88에 설정 추정 67).
// 차가 줄지 않으면(내리막, 버튼 유실) 계속 누른다.
TEST(AdaptiveCruise, SetStaysNearTheSpeedWhileTheCarSlows) {
    auto run = [](bool responds, float *worst_gap_kph, float *final_set_kph) {
        AdaptiveCruiseController controller;
        Vehicle vehicle;
        vehicle.set_speed_kph = vehicle.cluster_speed_kph = 100.0f;
        activate(&controller, &vehicle);
        float gap = 45.0f;
        const float lead_mps = 60.0f / 3.6f;
        *worst_gap_kph = 0.0f;
        for (int i = 1; i <= 6000; ++i) {  // 60초
            AdaptiveCruiseInput input = base_input(i * 0.01);
            const float ego_mps = vehicle.wheel_kph() / 3.6f;
            gap = std::fmax(1.0f, gap + (lead_mps - ego_mps) * 0.01f);
            input.vision_lead_updated = (i % 5 == 0);
            input.vision_lead_valid = true;
            input.vision_lead_distance_m = gap;
            input.vision_lead_relative_speed_mps = lead_mps - ego_mps;
            tick(&controller, input, &vehicle);
            if (responds) vehicle.coast(0.01f);
            *worst_gap_kph = std::fmax(*worst_gap_kph, vehicle.cluster_speed_kph - vehicle.set_speed_kph);
        }
        *final_set_kph = vehicle.set_speed_kph;
        return vehicle.wheel_kph();
    };
    float worst = 0.0f, final_set = 0.0f;
    const float final_kph = run(true, &worst, &final_set);
    ASSERT_LE(worst, 6.0f + 2.0f + 0.5f) << "차가 줄이는 동안 설정 속도는 실제 속도에서 한두 스텝 안쪽이다";
    ASSERT_LT(final_kph, 72.0f) << "그래도 느린 앞차 쪽으로 계속 줄인다";
    run(false, &worst, &final_set);
    ASSERT_LT(final_set, 100.0f - 10.0f) << "차가 줄지 않으면 계속 누른다";
}

// 앞차 측정 잡음(거리 ±7 m, 상대속도 ±1.5 m/s)으로 목표가 매초 흔들려도 SET-/RES+를 번갈아 내지 않는다.
// 예전에는 2026-10-10 18:34 주행 펄스의 36%가 3초 안에 방향을 바꿨고, 그사이 차가 앞차 쪽으로 빨라졌다.
TEST(AdaptiveCruise, NoisyLeadDoesNotAlternateButtons) {
    AdaptiveCruiseController controller;
    Vehicle vehicle;
    activate(&controller, &vehicle);
    int flips = 0, previous = 0, last_button = 0;
    double last_press_s = -10.0;
    for (int i = 1; i <= 12000; ++i) {  // 120초
        const double now = i * 0.01;
        AdaptiveCruiseInput input = base_input(now);
        const float wheel_mps = vehicle.wheel_kph() / 3.6f;
        const float sign = static_cast<int>(now / 0.7) % 2 == 0 ? 1.0f : -1.0f;
        input.vision_lead_updated = (i % 5 == 0);
        input.vision_lead_valid = true;
        input.vision_lead_distance_m = 5.0f + 1.8f * wheel_mps + 7.0f * sign;
        input.vision_lead_relative_speed_mps = 1.5f * sign;
        const AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
        vehicle.settle(0.01f);
        if (output.command_button != 0 && previous == 0) {
            if (last_button != 0 && output.command_button != last_button && now - last_press_s < 3.0) ++flips;
            last_button = output.command_button;
            last_press_s = now;
        }
        previous = output.command_button;
    }
    ASSERT_LE(flips, 1) << "앞차 측정 잡음으로 SET-/RES+를 번갈아 내지 않는다";
    ASSERT_LE(vehicle.set_speed_kph, 80.0f) << "잡음으로 설정 속도가 앞차 쪽으로 오르지 않는다";
}

TEST(AdaptiveCruise, RepositoryConfigLoads) {
    AdaptiveCruiseParams config;
    std::string error;
    ASSERT_TRUE(load_adaptive_cruise_params("params/adaptive_cruise.json", &config, &error)) << error.c_str();
    // 저장소 params/adaptive_cruise.json 기본값이 런타임 스키마와 같다
    ASSERT_TRUE(config.enabled);
    ASSERT_NEAR(config.following_time_s, 1.8f, 0.001f);
    ASSERT_NEAR(config.standstill_gap_m, 5.0f, 0.001f);
    ASSERT_NEAR(config.deceleration_rate_kph_per_s, 0.5f, 0.001f);
    ASSERT_NEAR(config.command_interval_s, 1.0f, 0.001f);
}

}  // namespace
