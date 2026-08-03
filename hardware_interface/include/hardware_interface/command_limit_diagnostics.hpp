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

#ifndef HARDWARE_INTERFACE__COMMAND_LIMIT_DIAGNOSTICS_HPP_
#define HARDWARE_INTERFACE__COMMAND_LIMIT_DIAGNOSTICS_HPP_

#include <string>

#include "joint_limits/data_structures.hpp"
#include "joint_limits/joint_limits.hpp"

namespace hardware_interface
{
namespace detail
{

/// Allocation-free explanation of the constraint responsible for a clamp.
struct CommandLimitDiagnostic
{
  const char * constraint = "coupled or soft";
  double evaluated_value = 0.0;
  double lower_limit = 0.0;
  double upper_limit = 0.0;
};

struct PositionBoundsResult
{
  double value = 0.0;
  bool limited = false;
};

/// Clamp a position command only to its hard position bounds.
PositionBoundsResult enforce_position_bounds_only(
  double command, const joint_limits::JointLimits & limits);

/**
 * Identify the physical constraint that caused a command-interface value to be clamped.
 *
 * This mirrors the hard-limit calculations in joint_limits. It is diagnostic only and never
 * modifies the command or the limiter's internal state.
 */
CommandLimitDiagnostic diagnose_command_limit(
  const std::string & interface_name, double command, double previous_command,
  const joint_limits::JointControlInterfacesData & actual,
  const joint_limits::JointLimits & limits, double period_seconds);

}  // namespace detail
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__COMMAND_LIMIT_DIAGNOSTICS_HPP_
