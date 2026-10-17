/**
 * Copyright (c) 2026, Heriot-Watt University.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#pragma once

#include <array>
#include <cmath>
#include <string>

namespace mujoco_ros2_control
{

/** Parameters of the differentiable six-coefficient joint-friction law. */
struct JointFrictionParameters
{
  std::array<double, 6> coefficients{};
  double scale{ 0.0 };

  /** Return the signed friction effort; the plant applies its negative. */
  double effort(const double velocity) const noexcept
  {
    const auto& g = coefficients;
    return scale * (g[0] * std::tanh(g[1] * velocity) - g[0] * std::tanh(g[2] * velocity) +
                    g[3] * std::tanh(g[4] * velocity) + g[5] * velocity);
  }
};

/** Runtime mapping between a configured joint and one MuJoCo generalized velocity. */
struct MuJoCoJointFrictionData
{
  std::string joint_name;
  int mj_dof_adr{ -1 };
  JointFrictionParameters parameters;
  double applied_effort{ 0.0 };
};

}  // namespace mujoco_ros2_control
