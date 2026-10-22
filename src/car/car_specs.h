#pragma once

// 차 제원과 차량 모델 기준값. 상류 opendbc car/__init__.py의 CarSpecs(이 차의 값), VehicleDynamicsParams(시빅
// 기준값), scale_tire_stiffness_per_mass와 같다.

#include <utility>

// KIA K7 YG HEV: 축거, 무게중심에서 앞차축까지(축거 비율). 상류도 차종 상수다.
constexpr float kWheelbaseM = 2.855f;
constexpr float kCenterToFrontRatio = 0.4f;
constexpr float kCenterToFrontM = kWheelbaseM * kCenterToFrontRatio;

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
