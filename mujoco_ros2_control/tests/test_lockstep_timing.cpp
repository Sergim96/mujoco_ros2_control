// Copyright 2026 ROS2-Control Development Team
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

#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "mujoco_ros2_control/lockstep_timing.hpp"

using namespace std::chrono_literals;

TEST(LockstepTimingTest, ScalesOnlyWallPeriod)
{
  const auto simulated_period = 2ms;
  EXPECT_EQ(
    mujoco_ros2_control::lockstep_wall_period(simulated_period, 1.0), 2ms);
  EXPECT_EQ(
    mujoco_ros2_control::lockstep_wall_period(simulated_period, 4.0), 8ms);
  EXPECT_EQ(simulated_period, 2ms);
}

TEST(LockstepTimingTest, RejectsSpeedupAndNonfiniteScales)
{
  EXPECT_FALSE(mujoco_ros2_control::valid_lockstep_wall_time_scale(0.999));
  EXPECT_FALSE(mujoco_ros2_control::valid_lockstep_wall_time_scale(
    std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(mujoco_ros2_control::valid_lockstep_wall_time_scale(
    std::numeric_limits<double>::quiet_NaN()));
  EXPECT_THROW(
    mujoco_ros2_control::lockstep_wall_period(2ms, 0.5),
    std::invalid_argument);
  EXPECT_THROW(
    mujoco_ros2_control::lockstep_wall_period(0ns, 4.0),
    std::invalid_argument);
}

TEST(LockstepTimingTest, PausedSolveDoesNotCreateCatchUpPhysicsBurst)
{
  using Clock = std::chrono::steady_clock;
  const Clock::time_point start{};
  EXPECT_EQ(mujoco_ros2_control::lockstep_next_wall_deadline(start, start + 1ms, 10ms),
            start + 10ms);
  const auto after_pause = mujoco_ros2_control::lockstep_next_wall_deadline(
    start, start + 3s, 10ms);
  EXPECT_EQ(after_pause, start + 3s + 10ms);
  EXPECT_EQ(mujoco_ros2_control::lockstep_next_wall_deadline(
    after_pause, after_pause + 1ms, 10ms), start + 3s + 20ms);
}
