#pragma once

// 차 제원과 차량 모델 기준값. 상류 opendbc car/__init__.py의 CarSpecs(이 차의 값), VehicleDynamicsParams(시빅
// 기준값), scale_tire_stiffness_per_mass와 같다.

#include <algorithm>
#include <utility>

// KIA K7 YG HEV: 축거, 무게중심에서 앞차축까지(축거 비율). 상류도 차종 상수다.
constexpr float kWheelbaseM = 2.855f;
constexpr float kCenterToFrontRatio = 0.4f;
constexpr float kCenterToFrontM = kWheelbaseM * kCenterToFrontRatio;

// 카메라 레버(m) = kCameraLeverM − kCameraLeverPerV2·v²(0 밑이면 0): 곡선에서 카메라 점의 진행 방향이 차체 축보다
// 레버·곡률만큼 바깥이다(타이어 미끄럼). 정상 곡선에서 카메라 위치의 차선 방향 대 곡률로 쟀다(2026-10-14~22 주행
// 다섯: 51 km/h 1.74 m, 64 km/h 1.16 m, 78 km/h 0.78 m, 88 km/h 0.59 m). 차선 MPC의 회전 반경(lateral_planner)과
// 폐루프 재생의 카메라 점 이동(replay_closed_loop)이 같이 쓴다.
constexpr double kCameraLeverM = 2.33;
constexpr double kCameraLeverPerV2 = 0.0029;

inline double camera_lever_m(double speed_mps) {
    return std::max(0.0, kCameraLeverM - kCameraLeverPerV2 * speed_mps * speed_mps);
}

// VehicleDynamicsParams: 시빅(1326 kg + 짐 136 kg). 강성과 관성은 이 질량으로 나눈 값으로 쓴다. 강성이 질량에
// 비례해 차량 모델에서 질량이 약분되므로 상류처럼 단위 질량으로 계산한다.
constexpr double kCivicMass = 1462.0;
constexpr double kCivicWheelbase = 2.70;
constexpr double kCivicCenterToFrontRatio = 0.4;
constexpr double kCivicInertia = 2500.0;
constexpr double kCivicTireStiffnessFront = 192150.0;
constexpr double kCivicTireStiffnessRear = 202500.0;

// opendbc scale_tire_stiffness_per_mass: (앞, 뒤) 강성. 연산 순서도 상류와 같다.
template <typename T>
std::pair<T, T> scale_tire_stiffness_per_mass(T wheelbase, T center_to_front, T tire_stiffness_factor) {
    const T civic_wheelbase = static_cast<T>(kCivicWheelbase);
    const T civic_center_to_front = civic_wheelbase * static_cast<T>(kCivicCenterToFrontRatio);
    const T civic_center_to_rear = civic_wheelbase - civic_center_to_front;
    const T center_to_rear = wheelbase - center_to_front;
    const T front = static_cast<T>(kCivicTireStiffnessFront) * tire_stiffness_factor / static_cast<T>(kCivicMass) *
                    (center_to_rear / wheelbase) / (civic_center_to_rear / civic_wheelbase);
    const T rear = static_cast<T>(kCivicTireStiffnessRear) * tire_stiffness_factor / static_cast<T>(kCivicMass) *
                   (center_to_front / wheelbase) / (civic_center_to_front / civic_wheelbase);
    return {front, rear};
}
