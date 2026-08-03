// Copyright 2026 Robot Motor Intelligence (RoMI) Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware_interface/command_limit_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hardware_interface
{
namespace detail
{
namespace
{

bool outside(const double value, const double lower, const double upper)
{
  return value < lower || value > upper;
}

CommandLimitDiagnostic diagnostic(
  const char * constraint, const double value, const double lower, const double upper)
{
  return {constraint, value, lower, upper};
}

}  // namespace

PositionBoundsResult enforce_position_bounds_only(
  const double command, const joint_limits::JointLimits & limits)
{
  if (!limits.has_position_limits || !std::isfinite(command))
  {
    return {command, false};
  }

  return {
    std::clamp(command, limits.min_position, limits.max_position),
    outside(command, limits.min_position, limits.max_position)};
}

CommandLimitDiagnostic diagnose_command_limit(
  const std::string & interface_name, const double command, const double previous_command,
  const joint_limits::JointControlInterfacesData & actual,
  const joint_limits::JointLimits & limits, const double period_seconds)
{
  const double infinity = std::numeric_limits<double>::infinity();
  if (!(period_seconds > 0.0) || !std::isfinite(command))
  {
    return diagnostic("non-finite or invalid-period", command, -infinity, infinity);
  }

  if (interface_name == hardware_interface::HW_IF_POSITION)
  {
    if (
      limits.has_position_limits &&
      outside(command, limits.min_position, limits.max_position))
    {
      return diagnostic("position", command, limits.min_position, limits.max_position);
    }

    return diagnostic(
      "soft or coupled position constraint", command,
      limits.has_position_limits ? limits.min_position : -infinity,
      limits.has_position_limits ? limits.max_position : infinity);
  }

  if (interface_name == hardware_interface::HW_IF_VELOCITY)
  {
    if (limits.has_position_limits && actual.position.has_value())
    {
      const double expected_position =
        actual.position.value() + command * period_seconds;
      if (outside(expected_position, limits.min_position, limits.max_position))
      {
        return diagnostic(
          "position boundary reached by velocity command", expected_position,
          limits.min_position, limits.max_position);
      }
    }

    if (
      limits.has_velocity_limits &&
      outside(command, -limits.max_velocity, limits.max_velocity))
    {
      return diagnostic("velocity", command, -limits.max_velocity, limits.max_velocity);
    }

    if (limits.has_acceleration_limits && std::isfinite(previous_command))
    {
      const double requested_acceleration =
        (command - previous_command) / period_seconds;
      if (
        outside(
          requested_acceleration, -limits.max_acceleration,
          limits.max_acceleration))
      {
        return diagnostic(
          "acceleration", requested_acceleration, -limits.max_acceleration,
          limits.max_acceleration);
      }
    }

    return diagnostic(
      "soft or coupled velocity constraint", command,
      limits.has_velocity_limits ? -limits.max_velocity : -infinity,
      limits.has_velocity_limits ? limits.max_velocity : infinity);
  }

  if (interface_name == hardware_interface::HW_IF_EFFORT)
  {
    if (
      limits.has_effort_limits &&
      outside(command, -limits.max_effort, limits.max_effort))
    {
      return diagnostic("effort/torque", command, -limits.max_effort, limits.max_effort);
    }

    if (
      limits.has_position_limits && actual.position.has_value() &&
      actual.velocity.has_value())
    {
      const bool pushing_below_lower =
        actual.position.value() <= limits.min_position && actual.velocity.value() <= 0.0 &&
        command < 0.0;
      const bool pushing_above_upper =
        actual.position.value() >= limits.max_position && actual.velocity.value() >= 0.0 &&
        command > 0.0;
      if (pushing_below_lower || pushing_above_upper)
      {
        return diagnostic(
          "position safety guard on effort", actual.position.value(), limits.min_position,
          limits.max_position);
      }
    }

    if (limits.has_velocity_limits && actual.velocity.has_value())
    {
      const bool pushing_below_lower =
        actual.velocity.value() < -limits.max_velocity && command < 0.0;
      const bool pushing_above_upper =
        actual.velocity.value() > limits.max_velocity && command > 0.0;
      if (pushing_below_lower || pushing_above_upper)
      {
        return diagnostic(
          "velocity safety guard on effort", actual.velocity.value(), -limits.max_velocity,
          limits.max_velocity);
      }
    }

    return diagnostic(
      "soft or coupled effort/torque constraint", command,
      limits.has_effort_limits ? -limits.max_effort : -infinity,
      limits.has_effort_limits ? limits.max_effort : infinity);
  }

  if (interface_name == hardware_interface::HW_IF_ACCELERATION)
  {
    double maximum = infinity;
    const char * constraint = "acceleration";
    const bool is_decelerating =
      actual.velocity.has_value() &&
      ((command < 0.0 && actual.velocity.value() > 0.0) ||
       (command > 0.0 && actual.velocity.value() < 0.0));
    if (is_decelerating && limits.has_deceleration_limits)
    {
      maximum = limits.max_deceleration;
      constraint = "deceleration";
    }
    else if (limits.has_acceleration_limits)
    {
      maximum = limits.max_acceleration;
    }
    return diagnostic(constraint, command, -maximum, maximum);
  }

  return diagnostic("coupled or soft", command, -infinity, infinity);
}

}  // namespace detail
}  // namespace hardware_interface
