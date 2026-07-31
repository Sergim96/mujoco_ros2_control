// Copyright (C) 2026 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Julia Jia

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include <hardware_interface/version.h>
#include <hardware_interface/hardware_info.hpp>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

class HeadlessInitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // Create a simple MuJoCo model
    create_test_model();

    // Initialize hardware interface
    hardware_info_ = create_hardware_info();
    interface_ = std::make_shared<mujoco_ros2_control::MujocoSystemInterface>();
  }

  void TearDown() override
  {
    // Deactivate interface before destroying to stop threads cleanly
    if (interface_)
    {
      rclcpp_lifecycle::State inactive_state(0, "inactive");
      interface_->on_deactivate(inactive_state);
      interface_.reset();
    }

    // Clean up ROS
    rclcpp::shutdown();

    // Clean up test file
    if (std::filesystem::exists(test_model_path_))
    {
      std::filesystem::remove(test_model_path_);
    }
  }

  void create_test_model()
  {
    // Create a simple MuJoCo XML with two bodies and a floor
    test_model_path_ = "/tmp/test_headless_init_model.xml";
    std::ofstream file(test_model_path_);
    file << R"(<?xml version="1.0"?>
<mujoco model="test_headless_init">
  <option timestep="0.002"/>

  <size nconmax="100"/>

  <worldbody>
    <!-- Floor as infinite plane directly in worldbody (static by default) -->
    <geom name="floor_geom" type="plane" size="0 0 1"
          contype="1" conaffinity="1"/>

    <body name="box1" pos="0 0 0.1">
      <freejoint name="box1_joint"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box1_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>

    <body name="box2" pos="0.2 0 0.1">
      <freejoint name="box2_joint"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box2_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>

    <body name="link1" pos="0 0 0.5">
      <joint name="joint1" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="link1_geom" type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="joint1" joint="joint1"/>
  </actuator>
</mujoco>
)";
    file.close();
  }

  hardware_interface::HardwareInfo create_hardware_info()
  {
    hardware_interface::HardwareInfo info;
    info.name = "test_mujoco";
    info.type = "system";
    info.hardware_parameters["mujoco_model"] = test_model_path_;
    info.hardware_parameters["meshdir"] = "";
    info.hardware_parameters["headless"] = "true";           // Enable headless mode for CI compatibility
    info.hardware_parameters["disable_rendering"] = "true";  // Disable cameras/lidar to avoid OpenGL issues in tests
    info.hardware_parameters["lockstep"] = "true";

    hardware_interface::ComponentInfo joint;
    joint.name = "joint1";
    joint.type = "joint";
    auto make_interface = [](const std::string& name, const std::string& initial_value = "") {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      interface.initial_value = initial_value;
      interface.size = 1;
      interface.enable_limits = false;
      return interface;
    };
    joint.command_interfaces = { make_interface("position"), make_interface("velocity"), make_interface("effort"),
                                 make_interface("kp"), make_interface("kd") };
    joint.state_interfaces = { make_interface("position", "0.0"), make_interface("velocity", "0.0"),
                               make_interface("effort", "0.0") };
    info.joints.push_back(joint);

    return info;
  }

  std::string test_model_path_;
  hardware_interface::HardwareInfo hardware_info_;
  std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> interface_;
};

TEST_F(HeadlessInitTest, HeadlessInitialization)
{
  // Test that MujocoSystemInterface can be initialized in headless mode
#if ROS_DISTRO_HUMBLE
  auto result = interface_->on_init(hardware_info_);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;
  auto result = interface_->on_init(params);
#endif
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Check that the data and model are available, meaning initializing was successful.
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(1);
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Model failed to initialize within timeout";

  // Test that we can export state interfaces without crashing
  auto state_interfaces = interface_->export_state_interfaces();
  EXPECT_GE(state_interfaces.size(), 0);
}

TEST_F(HeadlessInitTest, HybridWriteAdvancesExactlyOneStep)
{
#if ROS_DISTRO_HUMBLE
  ASSERT_EQ(interface_->on_init(hardware_info_), hardware_interface::CallbackReturn::SUCCESS);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;
  ASSERT_EQ(interface_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);
#endif

  auto commands = interface_->export_command_interfaces();
  ASSERT_EQ(commands.size(), 5u);
  const std::vector<std::string> expected_names = { "joint1/position", "joint1/velocity", "joint1/effort",
                                                    "joint1/kp", "joint1/kd" };
  std::vector<std::string> actual_names;
  for (auto& command : commands)
  {
    actual_names.push_back(command.get_name());
  }
  EXPECT_EQ(actual_names, expected_names);
  EXPECT_EQ(interface_->perform_command_mode_switch({ "joint1/effort" }, {}),
            hardware_interface::return_type::ERROR);
  ASSERT_EQ(interface_->perform_command_mode_switch(expected_names, {}), hardware_interface::return_type::OK);

  const std::array<double, 5> values = {{ 0.1, 0.0, 0.0, 10.0, 1.0 }};
  for (std::size_t index = 0; index < commands.size(); ++index)
  {
    ASSERT_TRUE(commands[index].set_value(values[index]));
  }

  const auto period = rclcpp::Duration::from_seconds(0.002);
  ASSERT_EQ(interface_->write(rclcpp::Time(1, 0, RCL_ROS_TIME), period), hardware_interface::return_type::OK);

  mjData* data = nullptr;
  interface_->get_data(data);
  ASSERT_NE(data, nullptr);
  EXPECT_NEAR(data->time, 0.002, 1e-12);
  EXPECT_NEAR(data->ctrl[0], 1.0, 1e-12);
  mj_deleteData(data);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
