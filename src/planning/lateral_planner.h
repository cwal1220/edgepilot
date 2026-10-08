#pragma once

#include "controls/lateral_target.h"

#include <memory>

struct ModelState;
struct SteeringParams;
struct VehicleCanState;

class LateralPlanner {
public:
  explicit LateralPlanner(const SteeringParams &params);
  ~LateralPlanner();

  LateralPlanner(const LateralPlanner &) = delete;
  LateralPlanner &operator=(const LateralPlanner &) = delete;

  void update_params(const SteeringParams &params);

  LateralTarget update(const ModelState &model,
                       const VehicleCanState &vehicle, float v_ego,
                       float measured_curvature, bool active);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
