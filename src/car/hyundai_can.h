#pragma once

// 현대 LKAS 명령 프레임을 만든다: LKAS11(조향 토크), CLU11(MDPS 속도 스푸프·크루즈 버튼), MDPS12. 받은 프레임을
// 바탕(seed)으로 바꿀 칸만 고친다.

#include <array>
#include <cstdint>
#include <vector>

#include "car/can_frame.h"

// 조향 토크 한계(상류 opendbc hyundai CarControllerParams의 대부분 HKG 값, 런타임 파라미터가 아니다). panda의 hyundai
// safety가 같은 숫자를 강제한다(safety_hyundai.h: MAX_STEER 384, MAX_RATE_UP 3, MAX_RATE_DOWN 7,
// DRIVER_TORQUE_ALLOWANCE 50, DRIVER_TORQUE_FACTOR 2, gtest_panda_firmware가 대조한다). 올리면 panda가 프레임을 거부하고
// 내리면 순정보다 약해지기만 한다. steer_driver_factor는 panda에 대응 항이 없어 1이 아니면 우리 계산만 어긋난다.
// 바꾸려면 panda 펌웨어와 함께 바꾼다.
constexpr int kHyundaiSteerMax = 384;
constexpr int kHyundaiSteerDeltaUp = 3;
constexpr int kHyundaiSteerDeltaDown = 7;
constexpr int kHyundaiSteerDriverAllowance = 50;
constexpr int kHyundaiSteerDriverMultiplier = 2;
constexpr int kHyundaiSteerDriverFactor = 1;

// opendbc apply_driver_steer_torque_limits: 운전자 토크 허용과 레이트 한계를 건 토크.
int apply_hyundai_steer_torque_limits(int desired_torque, int last_torque, int driver_torque);

struct HyundaiLkas11Values {
    int ldws_active_mode = 0;
    int ldws_sys_state = 0;
    int sys_warning = 0;
    int left_lane_depart = 0;
    int right_lane_depart = 0;
    int hba_lamp = 0;
    int fcw_bas_req = 0;
    int steer_torque = 0;
    bool steer_req = false;
    bool toi_fault = false;
    int hba_sys_state = 0;
    int fcw_opt = 0;
    int hba_opt = 0;
    int msg_count = 0;
    int fcw_sys_state = 0;
    int fcw_collision_warning = 0;
    int fusion_state = 0;
    int unknown1 = 0;
    int fcw_opt_usm = 0;
    int ldws_opt_usm = 0;
    int unknown2 = 0;
};

struct HyundaiClu11Values {
    int cruise_sw_state = 0;
    int cruise_sw_main = 0;
    int sld_main_sw = 0;
    int parity_bit = 0;
    float speed_decimal = 0.0f;
    float speed = 0.0f;
    bool speed_unit_mph = false;
    int detent_out = 0;
    int rheostat_level = 0;
    int clu_info = 0;
    int amp_info = 0;
    int alive_count = 0;
};

struct HyundaiLkasCommand {
    int apply_steer = 0;
    bool steer_req = false;
    bool cut_steer_temp = false;
    bool sys_warning = false;
    int sys_state = 0;
    int left_lane_depart = 0;
    int right_lane_depart = 0;
    int lkas_msg_count = 0;
};

HyundaiLkas11Values decode_lkas11(const std::array<uint8_t, 8> &data);
HyundaiClu11Values decode_clu11(const std::array<uint8_t, 4> &data);

// K7 커뮤니티 하네스(MDPS가 버스 1)의 횡제어 프레임을 송신 순서대로: LKAS11(MDPS 버스), LKAS11(파워트레인
// 버스), 홀수 프레임이면 MDPS가 보는 CLU11 속도 바꿔치기(MDPS 버스), MDPS12 오류 회피 프레임(카메라 버스).
// Panda safety는 버스와 상관없이 LKAS11마다 직전 LKAS11과 토크 변화를 대조한다. 운전자가 반대로 잡아 토크를
// 프레임마다 줄여야 할 때는 같은 값의 두 번째 사본이 거절되므로, MDPS가 받는 사본을 먼저 보낸다.
std::vector<CanFrame> build_lateral_can_frames(const HyundaiLkas11Values &lkas_seed, const HyundaiClu11Values &clu_seed,
                                               const std::array<uint8_t, 8> &mdps12_seed,
                                               const HyundaiLkasCommand &lkas_command, bool lkas_active, bool is_mph,
                                               int frame);

// 비전 크루즈의 버튼 펄스: 운전자 CLU11 seed에 버튼만 바꿔 파워트레인 버스로 보낸다.
CanFrame create_cruise_button_frame(const HyundaiClu11Values &clu_seed, int button, int frame);
