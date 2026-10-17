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

#ifndef MUJOCO_ROS2_CONTROL__LOCKSTEP_TIMING_HPP_
#define MUJOCO_ROS2_CONTROL__LOCKSTEP_TIMING_HPP_

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mujoco_ros2_control
{

inline bool valid_lockstep_wall_time_scale(const double scale)
{
  return std::isfinite(scale) && scale >= 1.0;
}

inline std::chrono::nanoseconds lockstep_wall_period(
  const std::chrono::nanoseconds simulated_period, const double scale)
{
  if (simulated_period.count() <= 0)
  {
    throw std::invalid_argument("lockstep simulated period must be positive");
  }
  if (!valid_lockstep_wall_time_scale(scale))
  {
    throw std::invalid_argument(
            "lockstep wall-time scale must be finite and at least 1.0");
  }
  const double scaled_count =
    static_cast<double>(simulated_period.count()) * scale;
  if (scaled_count >
      static_cast<double>(std::numeric_limits<std::chrono::nanoseconds::rep>::max()))
  {
    throw std::overflow_error("scaled lockstep wall period is not representable");
  }
  return std::chrono::nanoseconds(std::llround(scaled_count));
}

// A paused optimizer must not create wall-clock debt that is repaid by a burst
// of physics steps. Preserve nominal pacing and rebase after a real overrun.
inline std::chrono::steady_clock::time_point lockstep_next_wall_deadline(
  const std::chrono::steady_clock::time_point previous_deadline,
  const std::chrono::steady_clock::time_point completed_at,
  const std::chrono::nanoseconds wall_period)
{
  const auto nominal = previous_deadline + wall_period;
  return nominal < completed_at ? completed_at + wall_period : nominal;
}

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__LOCKSTEP_TIMING_HPP_
