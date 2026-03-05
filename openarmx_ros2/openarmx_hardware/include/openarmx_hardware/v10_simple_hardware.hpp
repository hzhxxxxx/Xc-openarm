// Copyright 2025 Enactic, Inc.
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

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <openarmx/can/socket/openarm.hpp>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarmx_hardware/dynamics.hpp"
#include "openarmx_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarmx_hardware {

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArm_v10HW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10HW();
  ~OpenArm_v10HW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  // Default motor configuration for V10 - Robstride motor mapping
  // 最新版配置: DM8009 → RS04, DM4340 → RS03, DM4310 → RS00
  const std::vector<openarmx::robstride_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarmx::robstride_motor::MotorType::RS04,  // Joint 1 (新版: RS04替代DM8009)
      openarmx::robstride_motor::MotorType::RS04,  // Joint 2 (新版: RS04替代DM8009)
      openarmx::robstride_motor::MotorType::RS03,  // Joint 3 (新版: RS03替代DM4340)
      openarmx::robstride_motor::MotorType::RS03,  // Joint 4 (新版: RS03替代DM4340)
      openarmx::robstride_motor::MotorType::RS00,  // Joint 5 (was DM4310)
      openarmx::robstride_motor::MotorType::RS00,  // Joint 6 (was DM4310)
      openarmx::robstride_motor::MotorType::RS00   // Joint 7 (was DM4310)
  };

  // Robstride uses same CAN ID for send and receive
  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};

  // Gripper motor - DM4310 → RS00
  const openarmx::robstride_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarmx::robstride_motor::MotorType::RS00;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x08;  // Same ID for Robstride

// 电机最大KP值参考值（较大的KP值，可以获得更好的精度，但过大会导致电机噪声过大，甚至电机会有振动，
// 该特性为电机故有属性不同品牌电机均存在这种现象，因此该参数不建议调太大！）

// DEFAULT_KP_MAX = {5000.0, 5000.0, 5000.0, 5000.0, 500.0, 500.0, 500.0, 500.0};
// 高刚性模式的KP值参考
// const std::vector<double> DEFAULT_KP = {500.0, 500.0, 500.0, 500.0, 50, 50, 50, 50};
// 示教录制模式KP值参考
// const std::vector<double> DEFAULT_KP = {1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.1};
// 默认KP值 - 已改为动态参数，见下方kp_values_成员变量
// const std::vector<double> DEFAULT_KP = {50.0, 50.0, 50.0, 50.0, 10, 10, 10, 10};

// 电机最大KD值参考（较大的KD值，可以获得更好的阻尼，但过大会导致电机噪声过大）

// DEFAULT_KD_MAX = {100.0, 100.0, 100.0, 100.0, 5.0, 5.0, 5.0, 5.0};
// 高刚性模式的KD值参考
// const std::vector<double> DEFAULT_KD = {10.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0};
// 示教录制模式KD值参考
// const std::vector<double> DEFAULT_KD = {0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05};
// 默认KD值 - 已改为动态参数，见下方kd_values_成员变量
// const std::vector<double> DEFAULT_KD = {2.5, 2.5, 2.5, 2.5, 0.5, 0.5, 0.5, 0.5};



// 夹爪参数
  const double GRIPPER_JOINT_0_POSITION = 0.044; 
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
//   const double GRIPPER_MOTOR_1_RADIANS = -1.0472; //for damiao motor
  const double GRIPPER_MOTOR_1_RADIANS = 1.0472; // for robstride motor
//   const double GRIPPER_DEFAULT_KP = 5.0;
//   const double GRIPPER_DEFAULT_KD = 0.5;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  bool hand_;
  bool can_fd_;
  enum class ControlMode { MIT, CSP };
  ControlMode control_mode_ = ControlMode::MIT;

  // ROS2 node for dynamic parameter handling
  rclcpp::Node::SharedPtr param_node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Executor and thread for spinning param_node_
  rclcpp::executors::SingleThreadedExecutor::SharedPtr param_executor_;
  std::thread param_spin_thread_;
  std::atomic<bool> param_spin_thread_active_{false};

  // Dynamic KP and KD values (8 motors: 7 arm joints + 1 gripper)
  std::vector<double> kp_values_;
  std::vector<double> kd_values_;
  std::mutex kp_kd_mutex_;  // Protect concurrent access to KP/KD values

  // OpenArm instance
  std::unique_ptr<openarmx::can::socket::OpenArm> openarm_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();

  // Dynamic parameter callback
  rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter>& parameters);

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);

  // Motor direction correction based on URDF axis definitions
  std::vector<double> get_motor_direction_multipliers() const;

  // Transient debug: help diagnose wrong mapping/offset on some sites
  // We only print a few cycles after activation to avoid flooding logs.
  int debug_cycles_remaining_ = 0;  // set in on_activate

  // Gravity compensation
  std::unique_ptr<Dynamics> arm_dyn_;
  std::vector<double> tau_gravity_;   // 重力力矩缓存
  double g_scale_ = 0.9;              // 可通过 hardware_parameter 配置
  bool gravity_comp_enable_ = false;  // 默认关闭，dry-run 验证后再开启
  int gravity_log_counter_ = 0;       // 控制日志打印频率
};

}  // namespace openarmx_hardware
