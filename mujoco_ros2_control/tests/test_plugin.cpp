/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <gtest/gtest.h>

#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control/data.hpp"

class MujocoSystemInterfaceLoadingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
        "hardware_interface", "hardware_interface::SystemInterface");
  }

  void TearDown() override
  {
    plugin_loader_.reset();
    rclcpp::shutdown();
  }

  std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> plugin_loader_;
};

TEST_F(MujocoSystemInterfaceLoadingTest, test_load_plugin)
{
  const std::string class_name = "mujoco_ros2_control/MujocoSystemInterface";

  // Pluginlib should find the class
  ASSERT_TRUE(plugin_loader_->isClassAvailable(class_name));

  // Construct a sim interface and verify it is not null
  std::shared_ptr<hardware_interface::SystemInterface> mujoco_sim = plugin_loader_->createSharedInstance(class_name);
  EXPECT_NE(mujoco_sim, nullptr);
}

TEST(MujocoHybridCommandTest, computes_unitree_motor_law)
{
  EXPECT_NEAR(mujoco_ros2_control::compute_hybrid_effort(1.2, -0.3, 4.0, 60.0, 2.0, 1.0, -0.1), 15.6, 1e-12);
}

TEST(MujocoImuNoiseTest, deterministic_seed_has_configured_bias_and_white_noise)
{
  mujoco_ros2_control::ImuNoiseConfig config;
  config.static_bias = Eigen::Vector3d(0.008, -0.006, 0.004);
  config.white_noise_stddev = Eigen::Vector3d::Constant(0.002);

  mujoco_ros2_control::ImuNoiseModel first;
  mujoco_ros2_control::ImuNoiseModel second;
  first.configure(config, 42);
  second.configure(config, 42);
  for (int sample = 0; sample < 20; ++sample)
  {
    EXPECT_TRUE(first.apply(Eigen::Vector3d::Zero(), 0.001)
                    .isApprox(second.apply(Eigen::Vector3d::Zero(), 0.001), 0.0));
  }

  mujoco_ros2_control::ImuNoiseModel statistics;
  statistics.configure(config, 42);
  constexpr int sample_count = 100000;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d square_sum = Eigen::Vector3d::Zero();
  for (int sample = 0; sample < sample_count; ++sample)
  {
    const Eigen::Vector3d value = statistics.apply(Eigen::Vector3d::Zero(), 0.001);
    sum += value;
    square_sum += value.cwiseProduct(value);
  }
  const Eigen::Vector3d mean = sum / sample_count;
  const Eigen::Vector3d variance = square_sum / sample_count - mean.cwiseProduct(mean);
  const Eigen::Vector3d stddev = variance.cwiseSqrt();
  for (Eigen::Index axis = 0; axis < 3; ++axis)
  {
    EXPECT_NEAR(mean[axis], config.static_bias[axis], 5e-5);
    EXPECT_NEAR(stddev[axis], config.white_noise_stddev[axis], 5e-5);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
