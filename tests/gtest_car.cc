// K7 YG HEV CAN: 받은 신호 해석(vehicle_can: WHL_SPD11, TPMS11, TCS13/15, AHB1 페달, SCC11,
// CLU11 크루즈 버튼과 고정형 크루즈 설정 속도, MDPS12 고장 필터)과 보낼 프레임 구성(hyundai_can: MDPS용
// CLU11 속도 바꿔치기). 신호 배치대로 손으로 채운 바이트로 검사한다. 보내는 LKAS11·CLU11·MDPS12 바이트와 토크
// 제한기는 opendbc(상류)로 만든 골든 값(hyundai_can_golden.h)과 Panda 펌웨어의 검사식과도 대조한다.
#include "car/can_frame.h"
#include "car/hyundai_can.h"
#include "car/speed_filter.h"
#include "car/vehicle_can.h"
#include "hyundai_can_golden.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <random>

namespace {

// 보내는 LKAS11(두 버스)은 opendbc 패커와 상류 create_lkas11(기본 분기 체크섬)로 만든 바이트와 같다
TEST(K7Can, LkasFramesMatchOpendbc) {
    for (const LkasGolden &g : kLkasGolden) {
        HyundaiLkasCommand command;
        command.apply_steer = g.apply_steer;
        command.steer_req = g.steer_req;
        command.cut_steer_temp = g.cut_steer_temp;
        command.sys_warning = g.sys_warning;
        command.sys_state = g.sys_state;
        command.left_lane_depart = g.left_lane_depart;
        command.right_lane_depart = g.right_lane_depart;
        command.lkas_msg_count = g.lkas_msg_count;
        const std::vector<CanFrame> frames =
            build_lateral_can_frames(decode_lkas11(g.seed), HyundaiClu11Values{}, {}, command, false, false, 0);
        ASSERT_GE(frames.size(), 2u);
        for (int bus = 0; bus < 2; ++bus) {
            EXPECT_EQ(frames[bus].address, kHyundaiLkas11Address);
            EXPECT_EQ(frames[bus].data, g.expected) << "토크 " << g.apply_steer << " 횟수 " << g.lkas_msg_count;
        }
    }
}

// 크루즈 버튼 CLU11은 상류 create_clu11과, MDPS12 오류 회피 프레임은 커뮤니티 create_mdps12와 같다
TEST(K7Can, Clu11AndMdps12MatchOpendbc) {
    for (const Clu11Golden &g : kClu11Golden) {
        const CanFrame frame = create_cruise_button_frame(decode_clu11(g.seed), g.button, g.frame);
        EXPECT_EQ(frame.length, 4);
        EXPECT_TRUE(std::equal(g.expected.begin(), g.expected.end(), frame.data.begin()))
            << "버튼 " << g.button << " 프레임 " << g.frame;
    }
    for (const Mdps12Golden &g : kMdps12Golden) {
        const std::vector<CanFrame> frames = build_lateral_can_frames(
            HyundaiLkas11Values{}, HyundaiClu11Values{}, g.seed, HyundaiLkasCommand{}, false, false, g.frame);
        EXPECT_EQ(frames.back().address, kHyundaiMdps12Address);
        EXPECT_EQ(frames.back().data, g.expected) << "프레임 " << g.frame;
    }
}

// 토크 제한기는 opendbc apply_driver_steer_torque_limits와 같은 값을 낸다
TEST(K7Can, TorqueLimiterMatchesOpendbc) {
    for (const TorqueGolden &g : kTorqueGolden)
        EXPECT_EQ(apply_hyundai_steer_torque_limits(g.desired, g.last, g.driver), g.expected)
            << "원하는 " << g.desired << " 지난 " << g.last << " 운전자 " << g.driver;
}

// Panda 펌웨어(firmware/panda/board/safety.h)의 driver_limit_check와 max_limit_check를 그대로 옮긴 것. 위반이면
// 참이다. 운전자 토크는 최근 표본의 최소·최대인데, 아래 시험은 한 구간 동안 같은 값을 쓴다.
bool firmware_violation(int val, int val_last, int driver_min, int driver_max) {
    constexpr int kMaxSteer = 384, kRateUp = 3, kRateDown = 7, kAllowance = 50, kDriverFactor = 2;
    if (val > kMaxSteer || val < -kMaxSteer) return true;
    const int highest_allowed_rl = std::max(val_last, 0) + kRateUp;
    const int lowest_allowed_rl = std::min(val_last, 0) - kRateUp;
    const int driver_max_limit = kMaxSteer + (kAllowance + driver_max) * kDriverFactor;
    const int driver_min_limit = -kMaxSteer + (-kAllowance + driver_min) * kDriverFactor;
    const int highest_allowed =
        std::min(highest_allowed_rl, std::max(val_last - kRateDown, std::max(driver_max_limit, 0)));
    const int lowest_allowed =
        std::max(lowest_allowed_rl, std::min(val_last + kRateDown, std::min(driver_min_limit, 0)));
    return val < lowest_allowed || val > highest_allowed;
}

// 어떤 원하는 토크와 운전자 토크가 와도 제한기를 거친 토크는 Panda가 막지 않는다(막으면 controls_allowed가 풀린다)
TEST(K7Can, TorqueLimiterPassesThePandaChecks) {
    std::mt19937 rng(11);
    std::uniform_int_distribution<int> desired_step(-60, 60), driver_value(-350, 350), hold(1, 80);
    int last = 0, desired = 0, driver = 0, driver_ticks = 0;
    for (int tick = 0; tick < 200000; ++tick) {
        if (driver_ticks-- <= 0) {
            driver = tick % 3 == 0 ? 0 : driver_value(rng);
            driver_ticks = hold(rng);
        }
        desired = std::clamp(desired + desired_step(rng), -600, 600);
        const int torque = apply_hyundai_steer_torque_limits(desired, last, driver);
        ASSERT_FALSE(firmware_violation(torque, last, driver, driver))
            << "틱 " << tick << ": 원하는 " << desired << " 지난 " << last << " 운전자 " << driver << " -> " << torque;
        last = torque;
    }
}

TEST(K7Can, MdpsSpeedSpoof) {
    HyundaiLkas11Values lkas;
    HyundaiClu11Values clu;
    clu.speed = 20.0f;
    clu.speed_decimal = 0.375f;
    const std::array<uint8_t, 8> mdps12_seed{};
    HyundaiLkasCommand command;
    command.steer_req = true;
    const auto frames = build_lateral_can_frames(lkas, clu, mdps12_seed, command, true, false, 1);
    ASSERT_EQ(frames.size(), 4) << "홀수 프레임: LKAS11 두 개, MDPS용 CLU11, MDPS12";
    ASSERT_EQ(frames[0].address, kHyundaiLkas11Address);
    ASSERT_EQ(frames[0].bus, kPowertrainBus);
    ASSERT_EQ(frames[1].address, kHyundaiLkas11Address);
    ASSERT_EQ(frames[1].bus, kMdpsBus) << "MDPS 버스에도 LKAS11을 보낸다";
    ASSERT_EQ(frames[3].address, kHyundaiMdps12Address);
    ASSERT_EQ(frames[3].bus, kHyundaiMdps12TxBus);
    std::array<uint8_t, 4> bytes = {};
    std::copy_n(frames[2].data.begin(), bytes.size(), bytes.begin());
    const HyundaiClu11Values decoded = decode_clu11(bytes);
    // MDPS용 CLU11은 버스 1로 나간다
    ASSERT_EQ(frames[2].address, kHyundaiClu11Address);
    ASSERT_EQ(frames[2].bus, kMdpsBus);
    ASSERT_NEAR(decoded.speed, 60.0f, 0.001f) << "MDPS용 CLU11 속도는 60 km/h로 바꿔 보낸다";
    ASSERT_NEAR(decoded.speed_decimal, 0.375f, 0.001f) << "MDPS용 CLU11의 소수부는 그대로 둔다";
    ASSERT_EQ(build_lateral_can_frames(lkas, clu, mdps12_seed, command, true, false, 2).size(), 3)
        << "짝수 프레임에는 MDPS용 CLU11이 없다";
}

TEST(K7Can, WhlSpd11) {
    std::array<uint8_t, 8> bytes{};
    const auto set_speed_raw = [&bytes](int start_bit, float speed_kph) {
        set_signal_le(&bytes, start_bit, 14, static_cast<uint32_t>(std::lround(speed_kph / 0.03125f)));
    };
    set_speed_raw(0, 40.0f);
    set_speed_raw(16, 41.0f);
    set_speed_raw(32, 39.0f);
    set_speed_raw(48, 40.5f);

    const WhlSpd11Values decoded = decode_whl_spd11(bytes);
    // WHL_SPD11 휠 속도 해석
    ASSERT_NEAR(decoded.speed_fl_kph, 40.0f, 0.001f);
    ASSERT_NEAR(decoded.speed_fr_kph, 41.0f, 0.001f);
    ASSERT_NEAR(decoded.speed_rl_kph, 39.0f, 0.001f);
    ASSERT_NEAR(decoded.speed_rr_kph, 40.5f, 0.001f);

    VehicleCanState vehicle;
    vehicle.cluster_speed_raw = 72.0f;
    update_vehicle_can_state(&vehicle, kHyundaiWhlSpd11Address, bytes, bytes.size(), kPowertrainBus, 1.0);
    // WHL_SPD11 네 바퀴 평균이 차속이다
    ASSERT_EQ(vehicle.whl_spd11_time_s, 1.0);
    ASSERT_NEAR(vehicle_speed_kph(vehicle, 1.2), 40.125f, 0.001f);
    // 클러스터로 대체하지 않는다: 도메인이 달라 최소 조향 속도 게이트가
    // 뒤집힌다. 대신 낡은 휠 속도는 vehicle_state_fresh에서 막힌다.
    ASSERT_FALSE(std::isfinite(vehicle_speed_kph(vehicle, 1.6))) << "낡은 WHL_SPD11을 CLU 속도로 대체하지 않는다";
    ASSERT_FALSE(vehicle_state_fresh(vehicle, 1.6, 0.5)) << "낡은 WHL_SPD11은 제어를 막는다";
}

TEST(K7Can, Tpms11) {
    std::array<uint8_t, 8> bytes{};
    set_signal_le(&bytes, 11, 2, 2);
    bytes[2] = 23;
    bytes[3] = 24;
    bytes[4] = 25;
    bytes[5] = 26;
    const Tpms11Values decoded = decode_tpms11(bytes);
    // TPMS11 공기압 해석
    ASSERT_EQ(decoded.unit, 2);
    ASSERT_NEAR(decoded.pressure_fl, 2.3f, 0.001f);
    ASSERT_NEAR(decoded.pressure_fr, 2.4f, 0.001f);
    ASSERT_NEAR(decoded.pressure_rl, 2.5f, 0.001f);
    ASSERT_NEAR(decoded.pressure_rr, 2.6f, 0.001f);
    ASSERT_FALSE(decoded.warning);

    bytes[0] |= 1U << 4;
    VehicleCanState vehicle;
    update_vehicle_can_state(&vehicle, kHyundaiTpms11Address, bytes, 6, kPowertrainBus, 1.0);
    // TPMS11로 차량 상태 갱신
    ASSERT_TRUE(tpms_state_fresh(vehicle, 2.0));
    ASSERT_TRUE(vehicle.tpms_warning);
    ASSERT_FALSE(tpms_state_fresh(vehicle, 7.0)) << "TPMS11 신선도 시한";

    bytes[2] = 0xff;
    update_vehicle_can_state(&vehicle, kHyundaiTpms11Address, bytes, 6, kPowertrainBus, 8.0);
    // TPMS11에서 값이 없는 바퀴는 0
    ASSERT_TRUE(tpms_state_fresh(vehicle, 8.0));
    ASSERT_EQ(vehicle.tpms_pressure_fl, 0.0f);
    ASSERT_GT(vehicle.tpms_pressure_fr, 0.0f);
}

TEST(K7Can, Tcs15) {
    std::array<uint8_t, 8> bytes{};
    set_signal_le(&bytes, 29, 3, 2);
    const Tcs15Values decoded = decode_tcs15(bytes);
    // TCS15 Auto Hold 작동 해석
    ASSERT_TRUE(decoded.brake_hold);
    ASSERT_FALSE(decoded.esp_disabled);

    VehicleCanState vehicle;
    update_vehicle_can_state(&vehicle, kHyundaiTcs15Address, bytes, bytes.size(), kPowertrainBus, 1.0);
    // TCS15 Auto Hold로 차량 상태 갱신
    ASSERT_TRUE(vehicle.brake_hold);
    ASSERT_EQ(vehicle.tcs15_time_s, 1.0);

    set_signal_le(&bytes, 29, 3, 3);
    update_vehicle_can_state(&vehicle, kHyundaiTcs15Address, bytes, bytes.size(), kPowertrainBus, 1.1);
    ASSERT_FALSE(vehicle.brake_hold) << "TCS15 대기 상태는 Auto Hold 작동이 아니다";
}

TEST(K7Can, Tcs13DriverOverride) {
    std::array<uint8_t, 8> bytes{};
    set_signal_le(&bytes, 45, 2, 2);
    const Tcs13Values decoded = decode_tcs13(bytes);
    ASSERT_EQ(decoded.driver_override, 2) << "TCS13 운전자 가속 개입 해석";

    VehicleCanState vehicle;
    update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, bytes, bytes.size(), kPowertrainBus, 1.0);
    ASSERT_EQ(vehicle.driver_override, 2) << "TCS13 운전자 가속 개입으로 차량 상태 갱신";
}

TEST(K7Can, Cgw1BcanTimeout) {
    std::array<uint8_t, 8> bytes{};
    set_signal_le(&bytes, 10, 2, 1);  // 안전벨트 착용
    set_signal_le(&bytes, 19, 2, 1);  // 왼쪽 깜빡이 켜짐
    Cgw1Values decoded = decode_cgw1(bytes);
    ASSERT_TRUE(decoded.left_blinker);
    ASSERT_FALSE(decoded.right_blinker);
    ASSERT_FALSE(decoded.seatbelt_unlatched);
    ASSERT_FALSE(decoded.driver_door_open);

    // 3 = B-CAN 신호 타임아웃: 모르는 값은 안전한 쪽으로
    for (const int start : {8, 10, 19, 62}) set_signal_le(&bytes, start, 2, 3);
    decoded = decode_cgw1(bytes);
    ASSERT_FALSE(decoded.left_blinker) << "타임아웃은 깜빡이 켜짐이 아니다(차선 변경 desire가 생긴다)";
    ASSERT_FALSE(decoded.right_blinker);
    ASSERT_TRUE(decoded.driver_door_open) << "타임아웃 문은 열린 것으로(결합을 막는다)";
    ASSERT_TRUE(decoded.seatbelt_unlatched) << "타임아웃 안전벨트는 미착용으로(결합을 막는다)";
}

// vEgo는 상류 opendbc CarStateBase.update_speed_kf(simple_kalman KF1D)와 같다. 기준값은 상류 파이썬을 같은
// 입력으로 돌려 얻었다: 10 m/s 50틱, 2 m/s² 가속에 ±0.05 m/s 잡음 100틱, 그다음 15 m/s로 뛰기(다시 시작).
TEST(SpeedFilter, MatchesUpstreamKalman) {
    SpeedFilter filter;
    auto raw_at = [](int t) {
        if (t < 50) return 10.0;
        if (t < 150) return 10.0 + 0.02 * (t - 49) + (t % 2 ? 0.05 : -0.05);
        return 15.0;
    };
    struct Expected {
        int t;
        double v;
    };
    const Expected expected[] = {{0, 10.0},
                                 {49, 10.0},
                                 {50, 9.994778188325945},
                                 {60, 10.163186164791778},
                                 {100, 11.03736443927332},
                                 {149, 12.024550996758542},
                                 {150, 15.0}};
    size_t next = 0;
    for (int t = 0; t <= 150; ++t) {
        const double v = filter.update(raw_at(t));
        if (next < std::size(expected) && expected[next].t == t) {
            EXPECT_NEAR(v, expected[next].v, 1e-9) << t;
            ++next;
        }
    }
    ASSERT_EQ(next, std::size(expected));
}

// 휠 속도가 낡으면(NaN) NaN을 내고, 다음 값에서 그 값으로 다시 시작한다.
TEST(SpeedFilter, StaleSpeedRestarts) {
    SpeedFilter filter;
    for (int i = 0; i < 20; ++i) filter.update(10.0 + 0.02 * i);
    ASSERT_TRUE(std::isnan(filter.update(std::nan(""))));
    ASSERT_DOUBLE_EQ(filter.update(10.2), 10.2) << "가까운 값이라도 끊긴 뒤에는 새로 시작한다";
    ASSERT_DOUBLE_EQ(filter.update(10.2), 10.2) << "가속도도 0에서 다시 시작해 그대로다";
}

TEST(K7Can, Ahb1BrakeLights) {
    std::array<uint8_t, 8> bytes{};
    set_signal_le(&bytes, 8, 16, 165);
    ASSERT_NEAR(decode_ahb1(bytes).pedal_stroke_mm, 16.5f, 1e-4f) << "AHB1 페달 스트로크 0.1 mm 단위";
    set_signal_le(&bytes, 8, 16, 0xffff);
    ASSERT_NEAR(decode_ahb1(bytes).pedal_stroke_mm, -0.1f, 1e-4f) << "부호 있는 값";

    VehicleCanState vehicle;
    ASSERT_FALSE(brake_lights_on(vehicle, 1.0)) << "신호가 없으면 꺼진 것";
    set_signal_le(&bytes, 8, 16, 165);
    update_vehicle_can_state(&vehicle, kHyundaiAhb1Address, bytes, bytes.size(), kCameraBus, 1.0);
    ASSERT_FALSE(brake_lights_on(vehicle, 1.0)) << "AHB1은 파워트레인 버스 것만 쓴다";
    update_vehicle_can_state(&vehicle, kHyundaiAhb1Address, bytes, bytes.size(), kPowertrainBus, 1.0);
    ASSERT_TRUE(brake_lights_on(vehicle, 1.1)) << "페달 16.5 mm면 켜진다";
    ASSERT_FALSE(brake_lights_on(vehicle, 2.0)) << "AHB1이 0.5 s 넘게 끊기면 꺼진 것";

    set_signal_le(&bytes, 8, 16, 20);
    update_vehicle_can_state(&vehicle, kHyundaiAhb1Address, bytes, bytes.size(), kPowertrainBus, 2.0);
    ASSERT_FALSE(brake_lights_on(vehicle, 2.0)) << "2 mm는 밟지 않은 페달의 흔들림";

    // AUTO HOLD: 페달은 놓았고 ESC가 잡고 있다(TCS13 BrakeLight)
    std::array<uint8_t, 8> tcs13{};
    set_signal_le(&tcs13, 11, 1, 1);
    update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13, tcs13.size(), kPowertrainBus, 2.0);
    ASSERT_TRUE(brake_lights_on(vehicle, 2.1)) << "AUTO HOLD 중에도 켜진다";
}

TEST(K7Can, Scc11) {
    std::array<uint8_t, 8> bytes{};
    bytes[0] = 1;
    bytes[1] = 88;
    set_signal_le(&bytes, 22, 2, 1);
    set_signal_le(&bytes, 33, 11, 54);
    set_signal_le(&bytes, 44, 12, 1715);
    const Scc11Values decoded = decode_scc11(bytes);
    // SCC11 크루즈 상태 해석
    ASSERT_TRUE(decoded.main_mode);
    ASSERT_NEAR(decoded.set_speed_raw, 88.0f, 0.001f);
    ASSERT_TRUE(decoded.object_valid);
    ASSERT_NEAR(decoded.object_distance_m, 5.4f, 0.001f);
    ASSERT_NEAR(decoded.object_relative_speed_mps, 1.5f, 0.001f);

    VehicleCanState vehicle;
    update_vehicle_can_state(&vehicle, kHyundaiScc11Address, bytes, bytes.size(), kPowertrainBus, 1.0);
    // SCC11로 차량 상태 갱신
    ASSERT_TRUE(vehicle.cruise_main);
    ASSERT_NEAR(vehicle.cruise_set_speed_raw, 88.0f, 0.001f);
    ASSERT_TRUE(vehicle.radar_lead_valid);
    ASSERT_NEAR(vehicle.radar_lead_distance_m, 5.4f, 0.001f);
    ASSERT_NEAR(vehicle.radar_lead_relative_speed_mps, 1.5f, 0.001f);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 88.0f, 0.001f) << "SCC11 km/h 설정 속도";
    vehicle.speed_unit_mph = true;
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 141.622272f, 0.001f) << "SCC11 mph 설정 속도 변환";
}

void update_clu11(VehicleCanState *vehicle, float speed, int button, bool unit_mph, double now_s) {
    std::array<uint8_t, 8> bytes{};
    set_signal_le(&bytes, 0, 3, static_cast<uint32_t>(button));
    set_signal_le(&bytes, 8, 9, static_cast<uint32_t>(std::lround(speed * 2.0f)));
    set_signal_le(&bytes, 17, 1, unit_mph ? 1U : 0U);
    update_vehicle_can_state(vehicle, kHyundaiClu11Address, bytes, 4, kPowertrainBus, now_s);
}

void release_cruise_button(VehicleCanState *vehicle, float speed, bool unit_mph, double now_s) {
    update_clu11(vehicle, speed, 0, unit_mph, now_s);
}

TEST(K7Can, FixedCruiseSpeedEstimate) {
    VehicleCanState vehicle;
    update_clu11(&vehicle, 64.0f, 2, false, 1.0);
    // 고정형 크루즈 SET은 클러스터 속도를 잡는다
    ASSERT_TRUE(vehicle.cruise_active);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 64.0f, 0.001f);

    release_cruise_button(&vehicle, 64.0f, false, 1.1);
    update_clu11(&vehicle, 64.0f, 1, false, 1.2);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 66.0f, 0.001f) << "작동 중 RES는 목표 속도를 올린다";
    update_clu11(&vehicle, 64.0f, 1, false, 1.3);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 66.0f, 0.001f) << "누르고 있는 버튼은 떼기 전까지 반복하지 않는다";

    release_cruise_button(&vehicle, 64.0f, false, 1.4);
    update_clu11(&vehicle, 64.0f, 2, false, 1.5);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 64.0f, 0.001f) << "작동 중 SET은 목표 속도를 내린다";
    release_cruise_button(&vehicle, 64.0f, false, 1.6);
    update_clu11(&vehicle, 64.0f, 4, false, 1.7);
    // CANCEL은 목표 속도를 남긴다
    ASSERT_FALSE(vehicle.cruise_active);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 64.0f, 0.001f);
    release_cruise_button(&vehicle, 70.0f, false, 1.8);
    update_clu11(&vehicle, 70.0f, 2, false, 1.9);
    // CANCEL 뒤 SET은 현재 속도를 잡는다
    ASSERT_TRUE(vehicle.cruise_active);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 70.0f, 0.001f);

    std::array<uint8_t, 8> tcs13{};
    set_signal_le(&tcs13, 55, 1, 1);
    update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13, tcs13.size(), kPowertrainBus, 2.0);
    ASSERT_FALSE(vehicle.cruise_active) << "브레이크는 추정한 크루즈 작동을 끈다";
    set_signal_le(&tcs13, 55, 1, 0);
    update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13, tcs13.size(), kPowertrainBus, 2.1);
    release_cruise_button(&vehicle, 60.0f, false, 2.2);
    update_clu11(&vehicle, 60.0f, 1, false, 2.3);
    // 브레이크 뒤 RES는 추정 목표 속도를 되살린다
    ASSERT_TRUE(vehicle.cruise_active);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 70.0f, 0.001f);

    release_cruise_button(&vehicle, 70.0f, false, 2.4);
    std::array<uint8_t, 8> clu_main{};
    set_signal_le(&clu_main, 3, 1, 1);
    set_signal_le(&clu_main, 8, 9, 140);
    update_vehicle_can_state(&vehicle, kHyundaiClu11Address, clu_main, 4, kPowertrainBus, 2.5);
    // MAIN을 누르면 작동을 끄고 목표 속도는 남긴다
    ASSERT_FALSE(vehicle.cruise_active);
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 70.0f, 0.001f);

    VehicleCanState imperial;
    update_clu11(&imperial, 40.0f, 2, true, 1.0);
    ASSERT_NEAR(cruise_set_speed_kph(imperial), 64.37376f, 0.001f) << "mph SET 변환";
    release_cruise_button(&imperial, 40.0f, true, 1.1);
    update_clu11(&imperial, 40.0f, 1, true, 1.2);
    ASSERT_NEAR(cruise_set_speed_kph(imperial), 67.592448f, 0.001f) << "mph 증가 단위";

    std::array<uint8_t, 8> scc11{};
    scc11[1] = 88;
    update_vehicle_can_state(&imperial, kHyundaiScc11Address, scc11, scc11.size(), kPowertrainBus, 1.3);
    ASSERT_NEAR(cruise_set_speed_kph(imperial), 141.622272f, 0.001f)
        << "유효한 SCC 설정 속도가 고정형 추정보다 우선한다";
}

// K7 HEV는 TCS13 DriverBraking이 늘 0이라 페달은 AHB1 스트로크로만 보인다. 예전에는 추정이 꺼지지 않아
// 비전 크루즈가 브레이크 뒤에도 SET-/RES+를 눌러 순정 크루즈를 다시 켰다.
TEST(K7Can, Ahb1PedalCancelsFixedCruiseEstimate) {
    VehicleCanState vehicle;
    update_clu11(&vehicle, 64.0f, 2, false, 1.0);
    release_cruise_button(&vehicle, 64.0f, false, 1.1);
    ASSERT_TRUE(vehicle.cruise_active);

    std::array<uint8_t, 8> ahb1{};
    set_signal_le(&ahb1, 8, 16, 165);  // 16.5 mm
    update_vehicle_can_state(&vehicle, kHyundaiAhb1Address, ahb1, ahb1.size(), kPowertrainBus, 2.0);
    ASSERT_TRUE(vehicle.brake_pressed) << "AHB1 페달 16.5 mm는 제동이다";
    ASSERT_FALSE(vehicle.cruise_active) << "페달은 추정한 크루즈 작동을 끈다";

    const std::array<uint8_t, 8> tcs13{};  // DriverBraking 0
    update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13, tcs13.size(), kPowertrainBus, 2.01);
    ASSERT_TRUE(vehicle.brake_pressed) << "TCS13의 0이 AHB1 제동을 덮어쓰지 않는다";

    update_clu11(&vehicle, 60.0f, 1, false, 2.1);
    ASSERT_FALSE(vehicle.cruise_active) << "밟은 채 누른 RES는 크루즈를 켜지 않는다";
    release_cruise_button(&vehicle, 60.0f, false, 2.2);

    set_signal_le(&ahb1, 8, 16, 20);  // 2 mm: 놓은 페달
    update_vehicle_can_state(&vehicle, kHyundaiAhb1Address, ahb1, ahb1.size(), kPowertrainBus, 2.3);
    ASSERT_FALSE(vehicle.brake_pressed);
    ASSERT_FALSE(vehicle.cruise_active) << "페달을 떼도 운전자가 다시 켜기 전까지 꺼진 채다";
    update_clu11(&vehicle, 60.0f, 1, false, 2.4);
    ASSERT_TRUE(vehicle.cruise_active) << "놓은 뒤 RES는 다시 켠다";
    ASSERT_NEAR(cruise_set_speed_kph(vehicle), 64.0f, 0.001f) << "RES는 이전 목표 속도로 돌아간다";
}

// opendbc hyundai carstate: steerFaultTemporary = CF_Mdps_ToiUnavail != 0 or CF_Mdps_ToiFlt != 0(거르지 않는다)
TEST(K7Can, MdpsTemporaryFault) {
    VehicleCanState vehicle;
    std::array<uint8_t, 8> bytes{};
    const auto feed = [&](uint8_t byte1, double t) {
        bytes[1] = byte1;
        update_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes, bytes.size(), kMdpsBus, t);
    };
    feed(1U << 7, 1.0);
    ASSERT_FALSE(vehicle.steer_fault_temporary) << "FailStat만으로는 상류도 고장으로 보지 않는다";
    feed(1U << 6, 1.01);
    ASSERT_TRUE(vehicle.steer_fault_temporary) << "ToiFlt는 그 프레임부터 고장이다";
    feed(0, 1.02);
    ASSERT_FALSE(vehicle.steer_fault_temporary) << "비트가 내려가면 바로 풀린다";
    feed(1U << 4, 1.03);
    ASSERT_TRUE(vehicle.steer_fault_temporary) << "ToiUnavail도 그 프레임부터 고장이다";
}

// opendbc hyundai carstate standstill: 앞 왼쪽·뒤 오른쪽 휠속도가 12 × 0.03125 km/h 이하
TEST(K7Can, WheelStandstill) {
    VehicleCanState vehicle;
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_rr_kph = 0.375f;
    vehicle.wheel_speed_fr_kph = vehicle.wheel_speed_rl_kph = 5.0f;
    ASSERT_TRUE(vehicle_standstill(vehicle));
    vehicle.wheel_speed_rr_kph = 0.40625f;
    ASSERT_FALSE(vehicle_standstill(vehicle));
}

}  // namespace
