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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "control_toolbox/pid_ros.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace mujoco_ros2_control
{

inline constexpr char HW_IF_KP[] = "kp";
inline constexpr char HW_IF_KD[] = "kd";

inline double compute_hybrid_effort(double desired_position, double desired_velocity, double feedforward_effort,
                                    double kp, double kd, double measured_position, double measured_velocity)
{
  return feedforward_effort + kp * (desired_position - measured_position) +
         kd * (desired_velocity - measured_velocity);
}

struct ImuNoiseConfig
{
  Eigen::Vector3d static_bias{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d white_noise_stddev{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d dynamic_bias_stddev{ Eigen::Vector3d::Zero() };
  double dynamic_bias_correlation_time{ 0.0 };
};

/**
 * Deterministic IMU error model matching SDFormat's Gaussian sensor model.
 *
 * White noise is independent per sample. Dynamic bias is a first-order
 * Gauss-Markov process driven by dynamic_bias_stddev; a non-positive
 * correlation time falls back to a random walk. The fixed seed makes tests
 * and simulator comparisons repeatable.
 */
class ImuNoiseModel
{
public:
  void configure(const ImuNoiseConfig& config, const std::uint64_t seed)
  {
    config_ = config;
    generator_.seed(seed);
    normal_.reset();
    dynamic_bias_.setZero();
  }

  Eigen::Vector3d apply(const Eigen::Vector3d& ideal, const double dt)
  {
    Eigen::Vector3d result = ideal + config_.static_bias;
    const double safe_dt = std::isfinite(dt) && dt > 0.0 ? dt : 0.0;
    const double tau = config_.dynamic_bias_correlation_time;
    const double decay = safe_dt > 0.0 && tau > 0.0 ? std::exp(-safe_dt / tau) : 1.0;

    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      const double dynamic_stddev = config_.dynamic_bias_stddev[axis];
      if (safe_dt > 0.0 && dynamic_stddev > 0.0)
      {
        double innovation_stddev = dynamic_stddev * std::sqrt(safe_dt);
        if (tau > 0.0)
        {
          // Exact discretization of db = -b/tau dt + sigma dW.
          innovation_stddev =
              dynamic_stddev * std::sqrt(0.5 * tau * (1.0 - decay * decay));
          dynamic_bias_[axis] *= decay;
        }
        dynamic_bias_[axis] += innovation_stddev * normal_(generator_);
      }
      result[axis] += dynamic_bias_[axis] + config_.white_noise_stddev[axis] * normal_(generator_);
    }
    return result;
  }

  const Eigen::Vector3d& dynamic_bias() const
  {
    return dynamic_bias_;
  }

private:
  ImuNoiseConfig config_;
  Eigen::Vector3d dynamic_bias_{ Eigen::Vector3d::Zero() };
  std::mt19937_64 generator_{ 0 };
  std::normal_distribution<double> normal_{ 0.0, 1.0 };
};

/**
 * Maps to MuJoCo actuator types:
 *  - MOTOR for MuJoCo motor actuator
 *  - POSITION for MuJoCo position actuator
 *  - VELOCITY for MuJoCo velocity actuator
 *  - CUSTOM  for MuJoCo general actuator or other types
 *
 * \note the MuJoCo types are as per the MuJoCo documentation:
 * https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator
 */

enum class ActuatorType
{
  UNKNOWN,
  MOTOR,
  POSITION,
  VELOCITY,
  PASSIVE,
  CUSTOM
};

/**
 * Data structure for each command/state interface.
 */
struct InterfaceData
{
  explicit InterfaceData(const std::string& command_interface) : command_interface_(command_interface)
  {
  }

  std::string command_interface_;
  double command_ = std::numeric_limits<double>::quiet_NaN();
  double state_ = std::numeric_limits<double>::quiet_NaN();

  // this is the "sink" that will be part of the transmission Joint/Actuator handles
  double transmission_passthrough_ = std::numeric_limits<double>::quiet_NaN();
};

/**
 * Wrapper for MuJoCo actuators and relevant ROS HW interface data.
 * @param joint_name Name of the MuJoCo joint handled by the actuator.
 * @param position_interface Data for position command/state interface.
 * @param velocity_interface Data for velocity command/state interface.
 * @param effort_interface Data for effort command/state interface.
 * @param pos_pid Pointer to position PID controller if configured.
 * @param vel_pid Pointer to velocity PID controller if configured.
 * @param actuator_type Type of the MuJoCo actuator.
 * @param mj_joint_type MuJoCo joint type as per mjModel->jnt_type.
 * @param mj_pos_adr MuJoCo position address in mjData->qpos.
 * @param mj_vel_adr MuJoCo velocity address in mjData->qvel.
 * @param mj_actuator_id MuJoCo actuator id as per mjModel->actuator_id.
 * @param is_position_control_enabled Boolean flag indicating if position control is enabled.
 * @param is_position_pid_control_enabled Boolean flag indicating if position PID control is enabled.
 * @param is_velocity_pid_control_enabled Boolean flag indicating if velocity PID control is enabled.
 * @param is_velocity_control_enabled Boolean flag indicating if velocity control is enabled.
 * @param is_effort_control_enabled Boolean flag indicating if effort control is enabled.
 * @param has_pos_pid Boolean flag indicating if a position PID controller is configured.
 * @param has_vel_pid Boolean flag indicating if a velocity PID controller is configured.
 */
struct MuJoCoActuatorData
{
  std::string joint_name = "";
  InterfaceData position_interface{ hardware_interface::HW_IF_POSITION };
  InterfaceData velocity_interface{ hardware_interface::HW_IF_VELOCITY };
  InterfaceData effort_interface{ hardware_interface::HW_IF_EFFORT };
  InterfaceData kp_interface{ HW_IF_KP };
  InterfaceData kd_interface{ HW_IF_KD };
  std::shared_ptr<control_toolbox::PidROS> pos_pid{ nullptr };
  std::shared_ptr<control_toolbox::PidROS> vel_pid{ nullptr };
  ActuatorType actuator_type{ ActuatorType::UNKNOWN };
  int mj_joint_type = -1;
  int mj_pos_adr = -1;
  int mj_vel_adr = -1;
  int mj_actuator_id = -1;

  // Booleans record whether or not we should be writing commands to these interfaces
  // based on if they have been claimed.
  bool is_position_control_enabled{ false };
  bool is_position_pid_control_enabled{ false };
  bool is_velocity_pid_control_enabled{ false };
  bool is_velocity_control_enabled{ false };
  bool is_effort_control_enabled{ false };
  bool is_hybrid_control_enabled{ false };
  bool has_pos_pid{ false };
  bool has_vel_pid{ false };

  void copy_state_to_transmission()
  {
    position_interface.transmission_passthrough_ = position_interface.state_;
    velocity_interface.transmission_passthrough_ = velocity_interface.state_;
    effort_interface.transmission_passthrough_ = effort_interface.state_;
  }

  void copy_command_from_transmission()
  {
    position_interface.command_ = position_interface.transmission_passthrough_;
    velocity_interface.command_ = velocity_interface.transmission_passthrough_;
    effort_interface.command_ = effort_interface.transmission_passthrough_;
  }

  void copy_command_to_state()
  {
    position_interface.state_ = position_interface.command_;
    velocity_interface.state_ = velocity_interface.command_;
    effort_interface.state_ = effort_interface.command_;
  }
};

/**
 * Structure for the URDF joint data.
 * @param name Name of the joint.
 * @param position_interface Data for position command/state interface.
 * @param velocity_interface Data for velocity command/state interface.
 * @param effort_interface Data for effort command/state interface.
 * @param command_interfaces Vector of command interface names supported by the joint.
 * @param is_mimic Boolean flag indicating if the joint is a mimic joint.
 * @param mimicked_joint_index Index of the joint being mimicked.
 * @param mimic_multiplier Multiplier for the mimic joint.
 * @param is_position_control_enabled Boolean flag indicating if position control is enabled.
 * @param is_velocity_control_enabled Boolean flag indicating if velocity control is enabled.
 * @param is_effort_control_enabled Boolean flag indicating if effort control is enabled.
 */
struct URDFJointData
{
  std::string name = "";
  InterfaceData position_interface{ hardware_interface::HW_IF_POSITION };
  InterfaceData velocity_interface{ hardware_interface::HW_IF_VELOCITY };
  InterfaceData effort_interface{ hardware_interface::HW_IF_EFFORT };
  InterfaceData kp_interface{ HW_IF_KP };
  InterfaceData kd_interface{ HW_IF_KD };

  std::vector<std::string> command_interfaces = {};

  bool is_mimic{ false };
  long int mimicked_joint_index;
  double mimic_multiplier;

  bool is_position_control_enabled{ false };
  bool is_velocity_control_enabled{ false };
  bool is_effort_control_enabled{ false };
  bool is_hybrid_control_enabled{ false };

  void copy_state_from_transmission()
  {
    position_interface.state_ = position_interface.transmission_passthrough_;
    velocity_interface.state_ = velocity_interface.transmission_passthrough_;
    effort_interface.state_ = effort_interface.transmission_passthrough_;
  }

  void copy_command_to_transmission()
  {
    position_interface.transmission_passthrough_ = position_interface.command_;
    velocity_interface.transmission_passthrough_ = velocity_interface.command_;
    effort_interface.transmission_passthrough_ = effort_interface.command_;
  }

  void copy_state_to_command()
  {
    position_interface.command_ = position_interface.state_;
    velocity_interface.command_ = velocity_interface.state_;
    effort_interface.command_ = effort_interface.state_;
  }
};

template <typename T>
struct SensorData
{
  std::string name;
  T data;
  int mj_sensor_index;
};

struct FTSensorData
{
  std::string name;
  SensorData<Eigen::Vector3d> force;
  SensorData<Eigen::Vector3d> torque;
};

struct IMUSensorData
{
  std::string name;
  SensorData<Eigen::Quaternion<double>> orientation;
  SensorData<Eigen::Vector3d> angular_velocity;
  SensorData<Eigen::Vector3d> linear_acceleration;
  ImuNoiseModel angular_velocity_noise;
  ImuNoiseModel linear_acceleration_noise;

  // These are currently unused but added to support controllers that require them.
  std::vector<double> orientation_covariance;
  std::vector<double> angular_velocity_covariance;
  std::vector<double> linear_acceleration_covariance;
};

}  // namespace mujoco_ros2_control
