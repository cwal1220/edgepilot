#pragma once

#include "car/can_frame.h"

#include <array>
#include <cstdint>

struct Sas11Values {
  float steering_angle_deg = 0.0f;
  float steering_rate_deg = 0.0f;
};

struct Mdps12Values {
  int driver_torque = 0;
  int driver_torque_raw_signal = 0;
  bool toi_unavailable = false;
  bool toi_active = false;
  bool toi_fault = false;
  bool fail_state = false;
  bool sensor_error = false;
  int msg_count = 0;
  int checksum = 0;
};

struct Esp12Values {
  float yaw_rate_deg_s = 0.0f;
  float yaw_rate_rad_s = 0.0f;
  bool yaw_rate_valid = true;
  float lat_accel_mps2 = 0.0f;
  bool lat_accel_valid = false;
  float long_accel_mps2 = 0.0f;
  bool long_accel_valid = false;
  float brake_pressure_bar = 0.0f;
};

struct WhlSpd11Values {
  float speed_fl_kph = 0.0f;
  float speed_fr_kph = 0.0f;
  float speed_rl_kph = 0.0f;
  float speed_rr_kph = 0.0f;
};

struct Scc11Values {
  bool main_mode = false;
  float set_speed_raw = 0.0f;
  bool object_valid = false;
  float object_distance_m = 0.0f;
  float object_relative_speed_mps = 0.0f;
};

struct Tcs13Values {
  bool brake_light = false;
  bool brake_error = false;
  int driver_override = 0;
  bool park_brake = false;
  bool brake_pressed = false;
};

struct Tcs15Values {
  bool esp_disabled = false;
  bool brake_hold = false;
};

struct Ahb1Values {
  float pedal_stroke_mm = 0.0f;
};

struct EEms11Values {
  int gas = 0;
  bool gas_pressed = false;
};

struct ElectGearValues {
  int gear = 0;
};

struct Cgw1Values {
  bool driver_door_open = false;
  bool passenger_door_open = false;
  bool front_door_open = false;
  bool seatbelt_unlatched = false;
  bool left_blinker = false;
  bool right_blinker = false;
  bool hazard = false;
};

struct Cgw2Values {
  bool rear_left_door_open = false;
  bool rear_right_door_open = false;
  bool rear_door_open = false;
};

struct Lca11Values {
  bool left_blindspot = false;
  bool right_blindspot = false;
};

struct Tpms11Values {
  int unit = 0;
  float pressure_fl = 0.0f;
  float pressure_fr = 0.0f;
  float pressure_rl = 0.0f;
  float pressure_rr = 0.0f;
  bool warning = false;
};

struct VehicleCanState {
  std::array<uint8_t, 8> lkas11_seed{};
  std::array<uint8_t, 4> clu11_seed{};
  std::array<uint8_t, 8> mdps12_seed{};

  bool has_lkas11_seed = false;
  bool has_clu11_seed = false;
  bool has_mdps12_seed = false;

  double lkas11_time_s = -1.0;
  double clu11_time_s = -1.0;
  double sas11_time_s = -1.0;
  double esp12_time_s = -1.0;
  double whl_spd11_time_s = -1.0;
  double scc11_time_s = -1.0;
  double mdps12_time_s = -1.0;
  double tcs13_time_s = -1.0;
  double tcs15_time_s = -1.0;
  double e_ems11_time_s = -1.0;
  double elect_gear_time_s = -1.0;
  double cgw1_time_s = -1.0;
  double cgw2_time_s = -1.0;
  double tpms11_time_s = -1.0;
  double ahb1_time_s = -1.0;

  int clu_button = 0;
  int clu_main_button = 0;
  float cluster_speed_raw = 0.0f;
  bool speed_unit_mph = false;

  float steering_angle_deg = 0.0f;
  float yaw_rate_rad_s = 0.0f;
  bool yaw_rate_valid = true;
  float lat_accel_mps2 = 0.0f;
  bool lat_accel_valid = false;
  float long_accel_mps2 = 0.0f;
  float wheel_speed_fl_kph = 0.0f;
  float wheel_speed_fr_kph = 0.0f;
  float wheel_speed_rl_kph = 0.0f;
  float wheel_speed_rr_kph = 0.0f;
  int driver_torque = 0;
  /* MDPS가 LKAS 토크를 받지 못한다고 알린다: MDPS12 CF_Mdps_ToiUnavail 또는 CF_Mdps_ToiFlt(opendbc hyundai
   * carstate의 steerFaultTemporary와 같다). K7은 고장 때 ToiFlt(FailStat과 함께)만 켜고 ToiUnavail은 2026-09·10
   * 녹화 18건에서 한 번도 켜지 않았다. */
  bool steer_fault_temporary = false;

  int gear = 0;
  /* 운전자가 브레이크 페달을 밟고 있다: TCS13 DriverBraking 또는 AHB1 페달 스트로크 3 mm 초과.
   * K7 HEV는 TCS13 비트가 늘 0이라 AHB1이 실제 신호다. 고정형 크루즈 추정과 비전 크루즈가 쓴다. */
  bool brake_pressed = false;
  bool tcs13_driver_braking = false;
  bool brake_light = false;
  bool brake_error = false;
  bool park_brake = false;
  int driver_override = 0;
  bool esp_disabled = false;
  bool brake_hold = false;
  float brake_pedal_stroke_mm = 0.0f;
  int gas = 0;
  bool gas_pressed = false;
  bool driver_door_open = false;
  bool passenger_door_open = false;
  bool rear_left_door_open = false;
  bool rear_right_door_open = false;
  bool door_open = false;
  bool seatbelt_unlatched = false;
  bool left_blinker = false;
  bool right_blinker = false;
  double left_blinker_until_s = -1.0;
  double right_blinker_until_s = -1.0;
  bool hazard = false;
  bool left_blindspot = false;
  bool right_blindspot = false;
  int tpms_unit = 0;
  float tpms_pressure_fl = 0.0f;
  float tpms_pressure_fr = 0.0f;
  float tpms_pressure_rl = 0.0f;
  float tpms_pressure_rr = 0.0f;
  bool tpms_warning = false;
  int acc_mode = 0;
  bool has_scc_cruise_state = false;
  bool cruise_main = false;
  bool cruise_active = false;
  float cruise_set_speed_raw = 0.0f;
  float estimated_cruise_set_speed_kph = 0.0f;
  bool estimated_cruise_set_speed_valid = false;
  bool estimated_cruise_active = false;
  bool radar_lead_valid = false;
  float radar_lead_distance_m = 0.0f;
  float radar_lead_relative_speed_mps = 0.0f;
};

// K7 SAS11 steering angle/rate를 해석한다.
Sas11Values decode_sas11(const std::array<uint8_t, 8> &data);

// K7 MDPS12 steering feedback/fault 값을 해석한다.
Mdps12Values decode_mdps12(const std::array<uint8_t, 8> &data);

// K7 ESP12 yaw/lateral acceleration 값을 해석한다.
Esp12Values decode_esp12(const std::array<uint8_t, 8> &data);

// K7 WHL_SPD11 네 바퀴 속도를 해석한다.
WhlSpd11Values decode_whl_spd11(const std::array<uint8_t, 8> &data);

// K7 SCC11 cruise와 전방 객체 상태를 해석한다.
Scc11Values decode_scc11(const std::array<uint8_t, 8> &data);

// K7 TCS13 brake 관련 상태를 해석한다.
Tcs13Values decode_tcs13(const std::array<uint8_t, 8> &data);

// K7 TCS15 ESP/brake-hold 상태를 해석한다.
Tcs15Values decode_tcs15(const std::array<uint8_t, 8> &data);

// K7 HEV AHB1(브레이크 부스터)의 페달 스트로크를 해석한다. 이 차는 TCS13 DriverBraking과
// DriverOverride가 늘 0이고 BrakeLight가 정차 중(거의 AUTO HOLD)에만 켜져서, 페달은 이것으로 본다.
Ahb1Values decode_ahb1(const std::array<uint8_t, 8> &data);

// K7 E_EMS11 hybrid gas 값을 해석한다.
EEms11Values decode_e_ems11(const std::array<uint8_t, 8> &data);

// K7 ELECT_GEAR gear 값을 해석한다.
ElectGearValues decode_elect_gear(const std::array<uint8_t, 8> &data);

// K7 CGW1 door/seatbelt/blinker 값을 해석한다.
Cgw1Values decode_cgw1(const std::array<uint8_t, 8> &data);

// K7 CGW2 rear door 값을 해석한다.
Cgw2Values decode_cgw2(const std::array<uint8_t, 8> &data);

// K7 LCA11 blind-spot indicators를 해석한다.
Lca11Values decode_lca11(const std::array<uint8_t, 8> &data);

Tpms11Values decode_tpms11(const std::array<uint8_t, 8> &data);

// raw CAN frame을 K7 vehicle state에 반영한다.
void update_vehicle_can_state(VehicleCanState *state, uint32_t address,
                              const std::array<uint8_t, 8> &data,
                              uint8_t length, uint8_t bus,
                              double now_s);

// K7 lateral 제어에 필요한 필수 CAN이 모두 최신인지 확인한다.
bool vehicle_state_fresh(const VehicleCanState &state, double now_s,
                         double timeout_s = 0.5);

// LKAS/CLU/MDPS seed frame이 모두 준비됐는지 확인한다.
bool seed_frames_ready(const VehicleCanState &state);

bool tpms_state_fresh(const VehicleCanState &state, double now_s,
                      double timeout_s = 5.0);

// 브레이크등이 켜져 있는가: 페달을 밟았거나(AHB1 스트로크) ESC가 차를 잡고 있다(TCS13 BrakeLight,
// AUTO HOLD). 오래된 신호는 꺼진 것으로 본다.
bool brake_lights_on(const VehicleCanState &state, double now_s,
                     double timeout_s = 0.5);

// SCC11 설정 속도를 사용하고, 없으면 고정형 크루즈 추정값을 반환한다.
float cruise_set_speed_kph(const VehicleCanState &state);

// 클러스터 표시 속도(km/h). 휠 속도보다 높게 나온다(K7 실측 약 6.6%). 값이 없으면 0.
float cluster_speed_kph(const VehicleCanState &state);

/* 최신 휠속도 평균. 프레임이 없거나 오래되면 NaN이다(클러스터 속도로 대체하지 않는다: 도메인이 달라
 * 최소 조향 속도 게이트가 뒤집힌다). */
float vehicle_speed_kph(const VehicleCanState &state, double now_s,
                        double timeout_s = 0.5);

// 바퀴가 멈췄다: 앞 왼쪽과 뒤 오른쪽 휠속도가 0.375 km/h 이하(opendbc hyundai carstate의 standstill).
bool vehicle_standstill(const VehicleCanState &state);
