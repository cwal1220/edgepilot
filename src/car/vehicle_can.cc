#include "car/vehicle_can.h"

#include "common/utils_time.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "car/hyundai_can.h"

namespace {

constexpr double kBlinkerHoldSeconds = 0.5;
// opendbc hyundai carstate STANDSTILL_THRESHOLD(12 × 0.03125 km/h)
constexpr float kStandstillWheelSpeedKph = 12.0f * 0.03125f;
/* 이보다 깊으면 페달을 밟은 것이다. 2026-10-04 녹화 두 건: 제동 중 스트로크 중앙값 16 mm(하위 5%
 * 4 mm), 비제동 주행의 99%가 3 mm 이하. 이 기준이 AHB1 작동 상태(CF_Ahb_Act)와 98% 같다. */
constexpr float kBrakePedalStrokeMm = 3.0f;
constexpr float kWheelSpeedScaleKph = 0.03125f;

// signed raw 값을 지정 bit 수로 sign extension한다.
int32_t sign_extend(uint32_t raw, int bits) {
  const uint32_t sign_bit = 1U << (bits - 1);
  if ((raw & sign_bit) == 0U) return static_cast<int32_t>(raw);
  return static_cast<int32_t>(raw - (1U << bits));
}

float cluster_speed_to_kph(float speed, bool unit_mph) {
  return speed * (unit_mph ? kMphToKph : 1.0f);
}

float current_cluster_speed_kph(const HyundaiClu11Values &clu) {
  return cluster_speed_to_kph(std::round(clu.speed + clu.speed_decimal),
                              clu.speed_unit_mph);
}

// K7 HEV에는 유효한 SCC/LVR 목표 속도가 없고 E_EMS11 후보는 엔진 회전수와
// 비트가 겹치므로 CLU11 버튼과 클러스터 속도로 고정형 크루즈 상태를 추정한다.
void update_fixed_cruise_estimate(VehicleCanState *state,
                                  const HyundaiClu11Values &clu) {
  const bool main_pressed =
      clu.cruise_sw_main != 0 && state->clu_main_button == 0;
  const bool button_pressed =
      clu.cruise_sw_state != 0 && state->clu_button == 0;
  const float display_unit_kph = clu.speed_unit_mph ? kMphToKph : 1.0f;
  const float step_kph = kFixedCruiseStep * display_unit_kph;
  const float minimum_kph =
      (clu.speed_unit_mph ? kMinimumCruiseSpeedMph
                          : kMinimumCruiseSpeedKph) * display_unit_kph;

  if (main_pressed) state->estimated_cruise_active = false;
  if (button_pressed) {
    if (clu.cruise_sw_state == kCruiseButtonCancel) {
      state->estimated_cruise_active = false;
    } else if (clu.cruise_sw_state == kCruiseButtonSet &&
               !state->brake_pressed) {
      if (state->estimated_cruise_active &&
          state->estimated_cruise_set_speed_valid) {
        state->estimated_cruise_set_speed_kph = std::max(
            minimum_kph, state->estimated_cruise_set_speed_kph - step_kph);
      } else {
        state->estimated_cruise_set_speed_kph = std::max(
            minimum_kph, current_cluster_speed_kph(clu));
      }
      state->estimated_cruise_set_speed_valid = true;
      state->estimated_cruise_active = true;
    } else if (clu.cruise_sw_state == kCruiseButtonResume &&
               !state->brake_pressed) {
      if (!state->estimated_cruise_set_speed_valid) {
        state->estimated_cruise_set_speed_kph = std::max(
            minimum_kph, current_cluster_speed_kph(clu));
        state->estimated_cruise_set_speed_valid = true;
      } else if (state->estimated_cruise_active) {
        state->estimated_cruise_set_speed_kph += step_kph;
      }
      state->estimated_cruise_active = true;
    }
  }

  state->clu_button = clu.cruise_sw_state;
  state->clu_main_button = clu.cruise_sw_main;
  if (!state->has_scc_cruise_state)
    state->cruise_active = state->estimated_cruise_active;
}

/* 제동은 TCS13 DriverBraking이나 AHB1 페달 스트로크로 본다. K7 HEV는 TCS13 비트가 늘 0이라
 * AHB1만 실제로 보인다. 둘을 따로 두고 합쳐야 0만 보내는 TCS13이 AHB1의 제동을 덮어쓰지 않는다.
 * 제동은 순정 크루즈를 끄므로 고정형 크루즈 추정도 끈다. 예전에는 TCS13만 봐서 추정이 꺼지지 않았고,
 * 비전 크루즈가 브레이크 뒤에도 SET-/RES+를 눌러 순정 크루즈를 다시 켰다(2026-09-27 주행). */
void update_brake_pressed(VehicleCanState *state) {
  state->brake_pressed = state->tcs13_driver_braking ||
                         state->brake_pedal_stroke_mm > kBrakePedalStrokeMm;
  if (state->brake_pressed) {
    state->estimated_cruise_active = false;
    if (!state->has_scc_cruise_state) state->cruise_active = false;
  }
}

}  // namespace

Sas11Values decode_sas11(const std::array<uint8_t, 8> &data) {
  Sas11Values values;
  values.steering_angle_deg =
      static_cast<float>(sign_extend(get_signal_le(data.data(), 0, 16), 16)) * 0.1f;
  values.steering_rate_deg =
      static_cast<float>(get_signal_le(data.data(), 16, 8)) * 4.0f;
  return values;
}

Mdps12Values decode_mdps12(const std::array<uint8_t, 8> &data) {
  Mdps12Values values;
  values.driver_torque_raw_signal = static_cast<int>(get_signal_le(data.data(), 0, 11));
  // openpilot CR_Mdps_StrColTq (factor 1, offset -1024)와 같은 스케일이다.
  values.driver_torque = values.driver_torque_raw_signal - 1024;
  values.toi_unavailable = get_signal_le(data.data(), 12, 1) != 0;
  values.toi_active = get_signal_le(data.data(), 13, 1) != 0;
  values.toi_fault = get_signal_le(data.data(), 14, 1) != 0;
  values.fail_state = get_signal_le(data.data(), 15, 1) != 0;
  values.msg_count = static_cast<int>(get_signal_le(data.data(), 16, 8));
  values.checksum = static_cast<int>(get_signal_le(data.data(), 24, 8));
  values.sensor_error = get_signal_le(data.data(), 37, 1) != 0;
  return values;
}

Esp12Values decode_esp12(const std::array<uint8_t, 8> &data) {
  Esp12Values values;
  const float yaw_rate_deg_s =
      static_cast<float>(get_signal_le(data.data(), 40, 13)) * 0.01f - 40.95f;
  values.yaw_rate_deg_s = yaw_rate_deg_s;
  values.yaw_rate_rad_s = yaw_rate_deg_s * 0.017453292519943295f;
  values.yaw_rate_valid = get_signal_le(data.data(), 54, 1) == 0;
  /* ESP12 LAT_ACCEL도 YAW_RATE처럼 제어 관례와 부호가 반대다. 2026-08-25
   * 실차 344샘플 회귀: 실측 = -1.022 x (curveYaw*v^2) + 0.158. 반전해 저장. */
  values.lat_accel_mps2 =
      -(static_cast<float>(get_signal_le(data.data(), 0, 11)) * 0.01f - 10.23f);
  values.lat_accel_valid = get_signal_le(data.data(), 12, 1) == 0;
  // 부호 관례 미검증(로그 전용). LAT처럼 반대일 수 있으니 제어에 쓰기 전 실측할 것.
  values.long_accel_mps2 =
      static_cast<float>(get_signal_le(data.data(), 13, 11)) * 0.01f - 10.23f;
  values.long_accel_valid = get_signal_le(data.data(), 25, 1) == 0;
  values.brake_pressure_bar =
      static_cast<float>(get_signal_le(data.data(), 26, 12)) * 0.1f;
  return values;
}

WhlSpd11Values decode_whl_spd11(const std::array<uint8_t, 8> &data) {
  WhlSpd11Values values;
  values.speed_fl_kph = static_cast<float>(get_signal_le(data.data(), 0, 14)) *
                        kWheelSpeedScaleKph;
  values.speed_fr_kph = static_cast<float>(get_signal_le(data.data(), 16, 14)) *
                        kWheelSpeedScaleKph;
  values.speed_rl_kph = static_cast<float>(get_signal_le(data.data(), 32, 14)) *
                        kWheelSpeedScaleKph;
  values.speed_rr_kph = static_cast<float>(get_signal_le(data.data(), 48, 14)) *
                        kWheelSpeedScaleKph;
  return values;
}

Scc11Values decode_scc11(const std::array<uint8_t, 8> &data) {
  Scc11Values values;
  values.main_mode = get_signal_le(data.data(), 0, 1) != 0;
  values.set_speed_raw = static_cast<float>(get_signal_le(data.data(), 8, 8));
  values.object_valid = get_signal_le(data.data(), 22, 2) != 0;
  values.object_distance_m =
      static_cast<float>(get_signal_le(data.data(), 33, 11)) * 0.1f;
  values.object_relative_speed_mps =
      static_cast<float>(get_signal_le(data.data(), 44, 12)) * 0.1f - 170.0f;
  return values;
}

Tcs13Values decode_tcs13(const std::array<uint8_t, 8> &data) {
  Tcs13Values values;
  const int acc_enable = static_cast<int>(get_signal_le(data.data(), 43, 2));
  values.brake_light = get_signal_le(data.data(), 11, 1) != 0;
  values.brake_error = acc_enable == 3;
  values.driver_override = static_cast<int>(get_signal_le(data.data(), 45, 2));
  values.park_brake = get_signal_le(data.data(), 53, 1) != 0;
  values.brake_pressed = get_signal_le(data.data(), 55, 1) != 0;
  return values;
}

Tcs15Values decode_tcs15(const std::array<uint8_t, 8> &data) {
  Tcs15Values values;
  values.esp_disabled = get_signal_le(data.data(), 8, 2) != 0;
  values.brake_hold = get_signal_le(data.data(), 29, 3) == 2;
  return values;
}

Ahb1Values decode_ahb1(const std::array<uint8_t, 8> &data) {
  Ahb1Values values;
  // CR_Ahb_StDep_mm: 8|16 signed, 0.1 mm
  values.pedal_stroke_mm = static_cast<float>(sign_extend(get_signal_le(data.data(), 8, 16), 16)) * 0.1f;
  return values;
}

EEms11Values decode_e_ems11(const std::array<uint8_t, 8> &data) {
  EEms11Values values;
  values.gas = static_cast<int>(get_signal_le(data.data(), 56, 8));
  values.gas_pressed = values.gas > 0;
  return values;
}

ElectGearValues decode_elect_gear(const std::array<uint8_t, 8> &data) {
  ElectGearValues values;
  values.gear = static_cast<int>(get_signal_le(data.data(), 16, 4));
  return values;
}

/* CGW1의 2비트 B-CAN 신호는 0·1이 꺼짐·켜짐(닫힘·열림, 미착용·착용)이고 3이 "B-CAN 신호
 * 타임아웃"이다(svrs_dl3_can_v6.dbc VAL_). 타임아웃은 모르는 값이라 안전한 쪽으로 읽는다:
 * 깜빡이·비상등은 꺼짐(켜짐으로 읽으면 차선 변경 desire가 생긴다), 문은 열림, 안전벨트는
 * 미착용(둘 다 결합을 막는다). 2026-10-02~04 녹화 6건에서 3은 한 번도 나오지 않았다. */
Cgw1Values decode_cgw1(const std::array<uint8_t, 8> &data) {
  Cgw1Values values;
  values.driver_door_open = get_signal_le(data.data(), 8, 2) != 0;
  values.passenger_door_open = get_signal_le(data.data(), 35, 1) != 0;
  values.front_door_open = values.driver_door_open || values.passenger_door_open;
  values.seatbelt_unlatched = get_signal_le(data.data(), 10, 2) != 1;
  values.left_blinker = get_signal_le(data.data(), 19, 2) == 1;
  values.right_blinker = get_signal_le(data.data(), 62, 2) == 1;
  values.hazard = get_signal_le(data.data(), 33, 2) == 1;
  return values;
}

Cgw2Values decode_cgw2(const std::array<uint8_t, 8> &data) {
  Cgw2Values values;
  values.rear_right_door_open = get_signal_le(data.data(), 23, 1) != 0;
  values.rear_left_door_open = get_signal_le(data.data(), 24, 1) != 0;
  values.rear_door_open = values.rear_left_door_open || values.rear_right_door_open;
  return values;
}

Lca11Values decode_lca11(const std::array<uint8_t, 8> &data) {
  Lca11Values values;
  values.left_blindspot = get_signal_le(data.data(), 8, 2) != 0;
  values.right_blindspot = get_signal_le(data.data(), 16, 2) != 0;
  return values;
}

Tpms11Values decode_tpms11(const std::array<uint8_t, 8> &data) {
  Tpms11Values values;
  values.unit = static_cast<int>(get_signal_le(data.data(), 11, 2));
  const float factor =
      values.unit == 1 ? 0.72519f : (values.unit == 2 ? 0.1f : 1.0f);
  auto pressure = [factor](uint8_t raw) {
    return raw == 0xff ? 0.0f : static_cast<float>(raw) * factor;
  };
  values.pressure_fl = pressure(data[2]);
  values.pressure_fr = pressure(data[3]);
  values.pressure_rl = pressure(data[4]);
  values.pressure_rr = pressure(data[5]);
  values.warning =
      get_signal_le(data.data(), 0, 2) != 0 ||
      get_signal_le(data.data(), 4, 1) != 0 ||
      get_signal_le(data.data(), 5, 1) != 0 ||
      get_signal_le(data.data(), 6, 1) != 0 ||
      get_signal_le(data.data(), 7, 1) != 0;
  return values;
}

namespace {

// 문 하나라도 열려 있으면 열림(CGW1 앞문, CGW2 뒷문을 합친다).
void update_door_open(VehicleCanState *state) {
  state->door_open = state->driver_door_open || state->passenger_door_open ||
                     state->rear_left_door_open || state->rear_right_door_open;
}

void apply_lkas11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  state->lkas11_seed = data;
  state->has_lkas11_seed = true;
  state->lkas11_time_s = now_s;
}

void apply_clu11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  state->clu11_seed = {{data[0], data[1], data[2], data[3]}};
  state->has_clu11_seed = true;
  const HyundaiClu11Values clu = decode_clu11(state->clu11_seed);
  update_fixed_cruise_estimate(state, clu);
  state->cluster_speed_raw = clu.speed + clu.speed_decimal;
  state->speed_unit_mph = clu.speed_unit_mph;
  state->clu11_time_s = now_s;
}

void apply_sas11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  state->steering_angle_deg = decode_sas11(data).steering_angle_deg;
  state->sas11_time_s = now_s;
}

void apply_esp12(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const Esp12Values esp = decode_esp12(data);
  state->yaw_rate_rad_s = esp.yaw_rate_rad_s;
  state->yaw_rate_valid = esp.yaw_rate_valid;
  state->lat_accel_mps2 = esp.lat_accel_mps2;
  state->lat_accel_valid = esp.lat_accel_valid;
  state->long_accel_mps2 = esp.long_accel_mps2;
  state->esp12_time_s = now_s;
}

void apply_whl_spd11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const WhlSpd11Values wheel = decode_whl_spd11(data);
  state->wheel_speed_fl_kph = wheel.speed_fl_kph;
  state->wheel_speed_fr_kph = wheel.speed_fr_kph;
  state->wheel_speed_rl_kph = wheel.speed_rl_kph;
  state->wheel_speed_rr_kph = wheel.speed_rr_kph;
  state->whl_spd11_time_s = now_s;
}

// MDPS12: 운전자 토크와 일시 고장(opendbc hyundai carstate의 steeringTorque, steerFaultTemporary).
void apply_mdps12(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  state->mdps12_seed = data;
  state->has_mdps12_seed = true;
  const Mdps12Values mdps = decode_mdps12(data);
  state->driver_torque = mdps.driver_torque;
  state->steer_fault_temporary = mdps.toi_unavailable || mdps.toi_fault;
  state->mdps12_time_s = now_s;
}

void apply_scc11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const Scc11Values scc = decode_scc11(data);
  state->cruise_main = scc.main_mode;
  state->cruise_set_speed_raw = scc.set_speed_raw;
  state->radar_lead_valid = scc.object_valid;
  state->radar_lead_distance_m = scc.object_distance_m;
  state->radar_lead_relative_speed_mps = scc.object_relative_speed_mps;
  state->scc11_time_s = now_s;
}

void apply_scc12(VehicleCanState *state, const std::array<uint8_t, 8> &data, double) {
  state->acc_mode = static_cast<int>(get_signal_le(data.data(), 13, 2));
  state->has_scc_cruise_state = true;
  state->cruise_active = state->acc_mode != 0;
}

void apply_tcs13(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const Tcs13Values tcs = decode_tcs13(data);
  state->brake_light = tcs.brake_light;
  state->brake_error = tcs.brake_error;
  state->park_brake = tcs.park_brake;
  state->driver_override = tcs.driver_override;
  state->tcs13_driver_braking = tcs.brake_pressed;
  update_brake_pressed(state);
  state->tcs13_time_s = now_s;
}

void apply_tcs15(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const Tcs15Values tcs = decode_tcs15(data);
  state->esp_disabled = tcs.esp_disabled;
  state->brake_hold = tcs.brake_hold;
  state->tcs15_time_s = now_s;
}

void apply_ahb1(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  state->brake_pedal_stroke_mm = decode_ahb1(data).pedal_stroke_mm;
  state->ahb1_time_s = now_s;
  update_brake_pressed(state);
}

void apply_e_ems11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const EEms11Values ems = decode_e_ems11(data);
  state->gas = ems.gas;
  state->gas_pressed = ems.gas_pressed;
  state->e_ems11_time_s = now_s;
}

void apply_elect_gear(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  state->gear = decode_elect_gear(data).gear;
  state->elect_gear_time_s = now_s;
}

/* CGW1: 앞문, 안전벨트, 깜빡이, 비상등. 깜빡이는 점멸 사이에도 켜진 것으로 보도록
 * kBlinkerHoldSeconds 동안 유지한다. */
void apply_cgw1(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const Cgw1Values cgw = decode_cgw1(data);
  state->driver_door_open = cgw.driver_door_open;
  state->passenger_door_open = cgw.passenger_door_open;
  update_door_open(state);
  state->seatbelt_unlatched = cgw.seatbelt_unlatched;
  if (cgw.left_blinker) state->left_blinker_until_s = now_s + kBlinkerHoldSeconds;
  if (cgw.right_blinker) state->right_blinker_until_s = now_s + kBlinkerHoldSeconds;
  state->left_blinker = now_s < state->left_blinker_until_s;
  state->right_blinker = now_s < state->right_blinker_until_s;
  state->hazard = cgw.hazard;
  state->cgw1_time_s = now_s;
}

void apply_cgw2(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const Cgw2Values cgw = decode_cgw2(data);
  state->rear_left_door_open = cgw.rear_left_door_open;
  state->rear_right_door_open = cgw.rear_right_door_open;
  update_door_open(state);
  state->cgw2_time_s = now_s;
}

void apply_lca11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double) {
  const Lca11Values lca = decode_lca11(data);
  state->left_blindspot = lca.left_blindspot;
  state->right_blindspot = lca.right_blindspot;
}

void apply_tpms11(VehicleCanState *state, const std::array<uint8_t, 8> &data, double now_s) {
  const Tpms11Values tpms = decode_tpms11(data);
  state->tpms_unit = tpms.unit;
  state->tpms_pressure_fl = tpms.pressure_fl;
  state->tpms_pressure_fr = tpms.pressure_fr;
  state->tpms_pressure_rl = tpms.pressure_rl;
  state->tpms_pressure_rr = tpms.pressure_rr;
  state->tpms_warning = tpms.warning;
  state->tpms11_time_s = now_s;
}

constexpr uint8_t bus_bit(uint8_t bus) { return static_cast<uint8_t>(1U << bus); }

/* 수신 메시지 표: 주소, 받는 버스(openpilot K7 파서와 같다), 최소 길이, 반영 함수. 표에 없는 주소는
 * 무시한다. */
struct CanMessage {
  uint32_t address;
  uint8_t buses;
  uint8_t min_length;
  void (*apply)(VehicleCanState *, const std::array<uint8_t, 8> &, double);
};

constexpr CanMessage kCanMessages[] = {
    {kHyundaiLkas11Address, bus_bit(kCameraBus), 8, apply_lkas11},
    {kHyundaiClu11Address, bus_bit(kPowertrainBus), 4, apply_clu11},
    {kHyundaiSas11Address, static_cast<uint8_t>(bus_bit(kPowertrainBus) | bus_bit(kMdpsBus)), 5, apply_sas11},
    {kHyundaiEsp12Address, bus_bit(kPowertrainBus), 8, apply_esp12},
    {kHyundaiWhlSpd11Address, bus_bit(kPowertrainBus), 8, apply_whl_spd11},
    {kHyundaiMdps12Address, bus_bit(kMdpsBus), 8, apply_mdps12},
    {kHyundaiScc11Address, bus_bit(kPowertrainBus), 8, apply_scc11},
    {kHyundaiScc12Address, bus_bit(kPowertrainBus), 8, apply_scc12},
    {kHyundaiTcs13Address, bus_bit(kPowertrainBus), 8, apply_tcs13},
    {kHyundaiTcs15Address, bus_bit(kPowertrainBus), 4, apply_tcs15},
    {kHyundaiAhb1Address, bus_bit(kPowertrainBus), 8, apply_ahb1},
    {kHyundaiEEms11Address, bus_bit(kPowertrainBus), 8, apply_e_ems11},
    {kHyundaiElectGearAddress, bus_bit(kPowertrainBus), 8, apply_elect_gear},
    {kHyundaiCgw1Address, bus_bit(kPowertrainBus), 8, apply_cgw1},
    {kHyundaiCgw2Address, bus_bit(kPowertrainBus), 8, apply_cgw2},
    {kHyundaiLca11Address, bus_bit(kPowertrainBus), 8, apply_lca11},
    {kHyundaiTpms11Address, bus_bit(kPowertrainBus), 6, apply_tpms11},
};

}  // namespace

void update_vehicle_can_state(VehicleCanState *state, uint32_t address,
                              const std::array<uint8_t, 8> &data,
                              uint8_t length, uint8_t bus,
                              double now_s) {
  if (!state) return;
  for (const CanMessage &message : kCanMessages) {
    if (message.address != address) continue;
    if (bus < 8 && (message.buses & bus_bit(bus)) != 0 && length >= message.min_length)
      message.apply(state, data, now_s);
    return;
  }
}

bool vehicle_state_fresh(const VehicleCanState &state, double now_s,
                         double timeout_s) {
  return signal_time_fresh(state.lkas11_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.clu11_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.sas11_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.mdps12_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.tcs13_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.tcs15_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.e_ems11_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.elect_gear_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.cgw1_time_s, now_s, timeout_s) &&
         signal_time_fresh(state.cgw2_time_s, now_s, timeout_s) &&
         /* 휠 속도는 제어의 모든 속도 임계값 기준이다. 낡으면 대체하지 않고
          * 제어를 막는다(vehicle_speed_kph는 NaN을 돌려준다). */
         signal_time_fresh(state.whl_spd11_time_s, now_s, timeout_s);
}

bool seed_frames_ready(const VehicleCanState &state) {
  return state.has_lkas11_seed && state.has_clu11_seed && state.has_mdps12_seed;
}

bool tpms_state_fresh(const VehicleCanState &state, double now_s,
                      double timeout_s) {
  return signal_time_fresh(state.tpms11_time_s, now_s, timeout_s);
}

bool brake_lights_on(const VehicleCanState &state, double now_s, double timeout_s) {
  const bool pedal = signal_time_fresh(state.ahb1_time_s, now_s, timeout_s) &&
                     state.brake_pedal_stroke_mm > kBrakePedalStrokeMm;
  const bool held = signal_time_fresh(state.tcs13_time_s, now_s, timeout_s) && state.brake_light;
  return pedal || held;
}

float cruise_set_speed_kph(const VehicleCanState &state) {
  if (state.cruise_set_speed_raw > 0.0f && state.cruise_set_speed_raw < 255.0f) {
    return cluster_speed_to_kph(state.cruise_set_speed_raw, state.speed_unit_mph);
  }
  return state.estimated_cruise_set_speed_valid
      ? state.estimated_cruise_set_speed_kph
      : 0.0f;
}

float cluster_speed_kph(const VehicleCanState &state) {
  if (!std::isfinite(state.cluster_speed_raw) || state.cluster_speed_raw < 0.0f) {
    return 0.0f;
  }
  return state.cluster_speed_raw * (state.speed_unit_mph ? kMphToKph : 1.0f);
}

float vehicle_speed_kph(const VehicleCanState &state, double now_s,
                        double timeout_s) {
  const bool wheel_speed_fresh =
      signal_time_fresh(state.whl_spd11_time_s, now_s, timeout_s);
  const float wheel_speeds[] = {
      state.wheel_speed_fl_kph,
      state.wheel_speed_fr_kph,
      state.wheel_speed_rl_kph,
      state.wheel_speed_rr_kph,
  };
  if (wheel_speed_fresh) {
    bool wheel_speed_valid = true;
    float wheel_speed_sum = 0.0f;
    for (const float speed_kph : wheel_speeds) {
      wheel_speed_valid = wheel_speed_valid && std::isfinite(speed_kph) &&
                          speed_kph >= 0.0f;
      wheel_speed_sum += speed_kph;
    }
    if (wheel_speed_valid) return wheel_speed_sum * 0.25f;
  }

  /* 클러스터 속도로 대체하지 않는다. 도메인이 달라(저속에서 1.2배) 조용히
   * 단위가 바뀌면 최소 조향 속도 게이트가 뒤집힌다. */
  return std::numeric_limits<float>::quiet_NaN();
}

bool vehicle_standstill(const VehicleCanState &state) {
  return state.wheel_speed_fl_kph <= kStandstillWheelSpeedKph &&
         state.wheel_speed_rr_kph <= kStandstillWheelSpeedKph;
}
