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

#include <gtest/gtest.h>

#include "hardware_interface/command_limit_diagnostics.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{

joint_limits::JointLimits make_limits()
{
  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -2.6;
  limits.max_position = -0.6;
  limits.has_velocity_limits = true;
  limits.max_velocity = 15.55;
  limits.has_effort_limits = true;
  limits.max_effort = 140.0;
  return limits;
}

TEST(CommandLimitDiagnostics, hybrid_position_command_uses_only_hard_position_bounds)
{
  const auto limits = make_limits();
  joint_limits::JointControlInterfacesData actual;
  actual.position = -1.15;
  actual.velocity = -0.18;

  const auto position = hardware_interface::detail::diagnose_command_limit(
    hardware_interface::HW_IF_POSITION, -0.5, -1.16, actual, limits, 0.002);
  EXPECT_STREQ(position.constraint, "position");
  EXPECT_DOUBLE_EQ(position.evaluated_value, -0.5);
  EXPECT_DOUBLE_EQ(position.lower_limit, -2.6);
  EXPECT_DOUBLE_EQ(position.upper_limit, -0.6);

  const auto within_bounds = hardware_interface::detail::enforce_position_bounds_only(
    -1.136317, limits);
  EXPECT_FALSE(within_bounds.limited);
  EXPECT_DOUBLE_EQ(within_bounds.value, -1.136317);

  const auto outside_bounds = hardware_interface::detail::enforce_position_bounds_only(-0.5, limits);
  EXPECT_TRUE(outside_bounds.limited);
  EXPECT_DOUBLE_EQ(outside_bounds.value, -0.6);
}

TEST(CommandLimitDiagnostics, reports_direct_velocity_and_effort_limits)
{
  const auto limits = make_limits();
  joint_limits::JointControlInterfacesData actual;
  actual.position = -1.2;
  actual.velocity = 0.0;

  const auto velocity = hardware_interface::detail::diagnose_command_limit(
    hardware_interface::HW_IF_VELOCITY, 16.0, 0.0, actual, limits, 0.002);
  EXPECT_STREQ(velocity.constraint, "velocity");
  EXPECT_DOUBLE_EQ(velocity.evaluated_value, 16.0);

  const auto effort = hardware_interface::detail::diagnose_command_limit(
    hardware_interface::HW_IF_EFFORT, 145.0, 0.0, actual, limits, 0.002);
  EXPECT_STREQ(effort.constraint, "effort/torque");
  EXPECT_DOUBLE_EQ(effort.lower_limit, -140.0);
  EXPECT_DOUBLE_EQ(effort.upper_limit, 140.0);
}

TEST(CommandLimitDiagnostics, does_not_mislabel_a_soft_position_clamp_as_velocity)
{
  const auto limits = make_limits();
  joint_limits::JointControlInterfacesData actual;
  actual.position = -1.2;
  actual.velocity = 0.0;

  const auto diagnostic = hardware_interface::detail::diagnose_command_limit(
    hardware_interface::HW_IF_POSITION, -1.19, -1.2, actual, limits, 0.002);
  EXPECT_STREQ(diagnostic.constraint, "soft or coupled position constraint");
  EXPECT_DOUBLE_EQ(diagnostic.evaluated_value, -1.19);
}

}  // namespace
