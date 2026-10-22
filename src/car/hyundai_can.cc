#include "car/hyundai_can.h"

#include "common/utils_math.h"

#include <algorithm>
#include <cmath>

namespace {

uint32_t clamp_u32(int value, int lo, int hi) { return static_cast<uint32_t>(std::min(std::max(value, lo), hi)); }

void set_common_lkas_fields(std::array<uint8_t, 8> *data, const HyundaiLkas11Values &values) {
    set_signal_le(data, 0, 2, clamp_u32(values.ldws_active_mode, 0, 3));
    set_signal_le(data, 2, 4, clamp_u32(values.ldws_sys_state, 0, 15));
    set_signal_le(data, 6, 4, clamp_u32(values.sys_warning, 0, 15));
    set_signal_le(data, 10, 2, clamp_u32(values.left_lane_depart, 0, 3));
    set_signal_le(data, 12, 2, clamp_u32(values.right_lane_depart, 0, 3));
    set_signal_le(data, 14, 1, clamp_u32(values.hba_lamp, 0, 1));
    set_signal_le(data, 15, 1, clamp_u32(values.fcw_bas_req, 0, 1));
    set_signal_le(data, 16, 11, clamp_u32(values.steer_torque + 1024, 0, 2047));
    set_signal_le(data, 27, 1, values.steer_req ? 1U : 0U);
    set_signal_le(data, 28, 1, values.toi_fault ? 1U : 0U);
    set_signal_le(data, 29, 3, clamp_u32(values.hba_sys_state, 0, 7));
    set_signal_le(data, 32, 2, clamp_u32(values.fcw_opt, 0, 3));
    set_signal_le(data, 34, 2, clamp_u32(values.hba_opt, 0, 3));
    set_signal_le(data, 36, 4, clamp_u32(values.msg_count, 0, 15));
    set_signal_le(data, 40, 3, clamp_u32(values.fcw_sys_state, 0, 7));
    set_signal_le(data, 43, 2, clamp_u32(values.fcw_collision_warning, 0, 3));
    set_signal_le(data, 45, 2, clamp_u32(values.fusion_state, 0, 3));
    set_signal_le(data, 47, 1, clamp_u32(values.unknown1, 0, 1));
    set_signal_le(data, 48, 8, 0);
    set_signal_le(data, 56, 3, clamp_u32(values.fcw_opt_usm, 0, 7));
    set_signal_le(data, 59, 3, clamp_u32(values.ldws_opt_usm, 0, 7));
    set_signal_le(data, 62, 2, clamp_u32(values.unknown2, 0, 3));
}

std::array<uint8_t, 4> pack_clu11_data(const HyundaiClu11Values &values) {
    std::array<uint8_t, 8> data = {};
    set_signal_le(&data, 0, 3, clamp_u32(values.cruise_sw_state, 0, 7));
    set_signal_le(&data, 3, 1, clamp_u32(values.cruise_sw_main, 0, 1));
    set_signal_le(&data, 4, 1, clamp_u32(values.sld_main_sw, 0, 1));
    set_signal_le(&data, 5, 1, clamp_u32(values.parity_bit, 0, 1));
    set_signal_le(
        &data, 6, 2,
        clamp_u32(static_cast<int>(std::lround(clamp_float(values.speed_decimal, 0.0f, 0.375f) / 0.125f)), 0, 3));
    set_signal_le(&data, 8, 9,
                  clamp_u32(static_cast<int>(std::lround(clamp_float(values.speed, 0.0f, 255.5f) / 0.5f)), 0, 511));
    set_signal_le(&data, 17, 1, values.speed_unit_mph ? 1U : 0U);
    set_signal_le(&data, 18, 1, clamp_u32(values.detent_out, 0, 1));
    set_signal_le(&data, 19, 5, clamp_u32(values.rheostat_level, 0, 31));
    set_signal_le(&data, 24, 1, clamp_u32(values.clu_info, 0, 1));
    set_signal_le(&data, 25, 1, clamp_u32(values.amp_info, 0, 1));
    set_signal_le(&data, 28, 4, clamp_u32(values.alive_count, 0, 15));
    return {{data[0], data[1], data[2], data[3]}};
}

HyundaiLkas11Values apply_lkas_command(HyundaiLkas11Values values, const HyundaiLkasCommand &command) {
    values.ldws_sys_state = command.sys_state;
    values.sys_warning = command.sys_warning ? 3 : 0;
    values.left_lane_depart = command.left_lane_depart;
    values.right_lane_depart = command.right_lane_depart;
    values.steer_torque = command.apply_steer;
    values.steer_req = command.steer_req && !command.cut_steer_temp;
    values.toi_fault = command.cut_steer_temp;
    values.msg_count = command.lkas_msg_count & 0xf;
    return values;
}

}  // namespace

HyundaiLkas11Values decode_lkas11(const std::array<uint8_t, 8> &data) {
    HyundaiLkas11Values values;
    values.ldws_active_mode = static_cast<int>(get_signal_le(data.data(), 0, 2));
    values.ldws_sys_state = static_cast<int>(get_signal_le(data.data(), 2, 4));
    values.sys_warning = static_cast<int>(get_signal_le(data.data(), 6, 4));
    values.left_lane_depart = static_cast<int>(get_signal_le(data.data(), 10, 2));
    values.right_lane_depart = static_cast<int>(get_signal_le(data.data(), 12, 2));
    values.hba_lamp = static_cast<int>(get_signal_le(data.data(), 14, 1));
    values.fcw_bas_req = static_cast<int>(get_signal_le(data.data(), 15, 1));
    values.steer_torque = static_cast<int>(get_signal_le(data.data(), 16, 11)) - 1024;
    values.steer_req = get_signal_le(data.data(), 27, 1) != 0;
    values.toi_fault = get_signal_le(data.data(), 28, 1) != 0;
    values.hba_sys_state = static_cast<int>(get_signal_le(data.data(), 29, 3));
    values.fcw_opt = static_cast<int>(get_signal_le(data.data(), 32, 2));
    values.hba_opt = static_cast<int>(get_signal_le(data.data(), 34, 2));
    values.msg_count = static_cast<int>(get_signal_le(data.data(), 36, 4));
    values.fcw_sys_state = static_cast<int>(get_signal_le(data.data(), 40, 3));
    values.fcw_collision_warning = static_cast<int>(get_signal_le(data.data(), 43, 2));
    values.fusion_state = static_cast<int>(get_signal_le(data.data(), 45, 2));
    values.unknown1 = static_cast<int>(get_signal_le(data.data(), 47, 1));
    values.fcw_opt_usm = static_cast<int>(get_signal_le(data.data(), 56, 3));
    values.ldws_opt_usm = static_cast<int>(get_signal_le(data.data(), 59, 3));
    values.unknown2 = static_cast<int>(get_signal_le(data.data(), 62, 2));
    return values;
}

HyundaiClu11Values decode_clu11(const std::array<uint8_t, 4> &short_data) {
    const std::array<uint8_t, 8> data = {{short_data[0], short_data[1], short_data[2], short_data[3], 0, 0, 0, 0}};
    HyundaiClu11Values values;
    values.cruise_sw_state = static_cast<int>(get_signal_le(data.data(), 0, 3));
    values.cruise_sw_main = static_cast<int>(get_signal_le(data.data(), 3, 1));
    values.sld_main_sw = static_cast<int>(get_signal_le(data.data(), 4, 1));
    values.parity_bit = static_cast<int>(get_signal_le(data.data(), 5, 1));
    values.speed_decimal = static_cast<float>(get_signal_le(data.data(), 6, 2)) * 0.125f;
    values.speed = static_cast<float>(get_signal_le(data.data(), 8, 9)) * 0.5f;
    values.speed_unit_mph = get_signal_le(data.data(), 17, 1) != 0;
    values.detent_out = static_cast<int>(get_signal_le(data.data(), 18, 1));
    values.rheostat_level = static_cast<int>(get_signal_le(data.data(), 19, 5));
    values.clu_info = static_cast<int>(get_signal_le(data.data(), 24, 1));
    values.amp_info = static_cast<int>(get_signal_le(data.data(), 25, 1));
    values.alive_count = static_cast<int>(get_signal_le(data.data(), 28, 4));
    return values;
}

namespace {

struct HyundaiCluCommand {
    int button = 0;
    float speed = 0.0f;
    int frame = 0;
};

// 저속에서도 MDPS가 LKAS 토크를 받도록 MDPS 버스 CLU11에 쓰는 속도. 상류에는 없는 기능이고, 커뮤니티 포크
// (openpilot_c2)도 60 km/h(38 mph)를 코드에 박아 둔다.
constexpr float kMdpsSpeedSpoofKph = 60.0f;

// CLU11 속도는 부호 없는 정수 해석이다
float mdps_speed_for_lkas(float cluster_speed_raw, bool lkas_active, bool is_mph) {
    const float threshold = is_mph ? kMdpsSpeedSpoofKph / kMphToKph : kMdpsSpeedSpoofKph;
    return lkas_active && cluster_speed_raw <= threshold ? threshold : cluster_speed_raw;
}

uint8_t hyundai_lkas11_checksum(const std::array<uint8_t, 8> &data) {
    return static_cast<uint8_t>((data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[7]) % 256);
}

CanFrame create_lkas11_frame(const HyundaiLkas11Values &seed, const HyundaiLkasCommand &command, uint8_t bus) {
    std::array<uint8_t, 8> data = {};
    const HyundaiLkas11Values values = apply_lkas_command(seed, command);
    set_common_lkas_fields(&data, values);
    data[6] = hyundai_lkas11_checksum(data);
    return {kHyundaiLkas11Address, bus, 8, data};
}

CanFrame create_clu11_frame(const HyundaiClu11Values &seed, const HyundaiCluCommand &command, uint8_t bus) {
    HyundaiClu11Values values = seed;
    values.cruise_sw_state = command.button;
    values.speed = command.speed;
    values.alive_count = command.frame & 0xf;

    CanFrame frame;
    frame.address = kHyundaiClu11Address;
    frame.bus = bus;
    frame.length = 4;
    const std::array<uint8_t, 4> packed = pack_clu11_data(values);
    frame.data = {{packed[0], packed[1], packed[2], packed[3], 0, 0, 0, 0}};
    return frame;
}

// 최신 MDPS12 seed에서 openpilot create_mdps12와 같은 오류 회피 frame을 만든다.
CanFrame create_mdps12_frame(const std::array<uint8_t, 8> &seed, int frame_count) {
    std::array<uint8_t, 8> data = seed;
    set_signal_le(&data, 13, 1, 0);
    set_signal_le(&data, 12, 1, 1);
    set_signal_le(&data, 16, 8, static_cast<uint32_t>(frame_count) & 0xffU);
    set_signal_le(&data, 24, 8, 0);

    uint32_t checksum = 0;
    for (uint8_t byte : data) {
        checksum += byte;
    }
    set_signal_le(&data, 24, 8, checksum & 0xffU);
    return {kHyundaiMdps12Address, kHyundaiMdps12TxBus, 8, data};
}

}  // namespace

std::vector<CanFrame> build_lateral_can_frames(const HyundaiLkas11Values &lkas_seed, const HyundaiClu11Values &clu_seed,
                                               const std::array<uint8_t, 8> &mdps12_seed,
                                               const HyundaiLkasCommand &lkas_command, bool lkas_active, bool is_mph,
                                               int frame) {
    std::vector<CanFrame> frames;
    frames.reserve(4);  // 10 ms마다 만든다: 늘리며 다시 잡지 않게
    // 두 버스의 LKAS11은 바이트가 같다(버스만 다르다)
    frames.push_back(create_lkas11_frame(lkas_seed, lkas_command, kPowertrainBus));
    frames.push_back(frames.back());
    frames.back().bus = kMdpsBus;
    if ((frame % 2) != 0) {
        HyundaiCluCommand clu_command;
        clu_command.button = 0;
        clu_command.speed = mdps_speed_for_lkas(clu_seed.speed, lkas_active, is_mph);
        clu_command.frame = frame;
        frames.push_back(create_clu11_frame(clu_seed, clu_command, kMdpsBus));
    }
    frames.push_back(create_mdps12_frame(mdps12_seed, frame));
    return frames;
}

CanFrame create_cruise_button_frame(const HyundaiClu11Values &clu_seed, int button, int frame) {
    HyundaiCluCommand command;
    command.button = button;
    command.speed = clu_seed.speed;
    command.frame = frame;
    return create_clu11_frame(clu_seed, command, kPowertrainBus);
}

int apply_hyundai_steer_torque_limits(int desired_torque, int last_torque, int driver_torque) {
    const int driver_max_torque =
        kHyundaiSteerMax +
        (kHyundaiSteerDriverAllowance + driver_torque * kHyundaiSteerDriverFactor) * kHyundaiSteerDriverMultiplier;
    const int driver_min_torque =
        -kHyundaiSteerMax +
        (-kHyundaiSteerDriverAllowance + driver_torque * kHyundaiSteerDriverFactor) * kHyundaiSteerDriverMultiplier;

    const int max_steer_allowed = std::max(std::min(kHyundaiSteerMax, driver_max_torque), 0);
    const int min_steer_allowed = std::min(std::max(-kHyundaiSteerMax, driver_min_torque), 0);
    int apply_torque = clamp_int(desired_torque, min_steer_allowed, max_steer_allowed);

    if (last_torque > 0) {
        apply_torque = clamp_int(apply_torque, std::max(last_torque - kHyundaiSteerDeltaDown, -kHyundaiSteerDeltaUp),
                                 last_torque + kHyundaiSteerDeltaUp);
    } else {
        apply_torque = clamp_int(apply_torque, last_torque - kHyundaiSteerDeltaUp,
                                 std::min(last_torque + kHyundaiSteerDeltaDown, kHyundaiSteerDeltaUp));
    }
    return apply_torque;
}
