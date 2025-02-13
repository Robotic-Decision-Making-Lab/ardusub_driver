// Copyright 2024, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <memory>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "mavros_odom_sensor/visibility_control.h"

#include "nav_msgs/msg/odometry.hpp"

namespace mavros_odom_sensor
{

class MavrosOdomSensor : public hardware_interface::SystemInterface
{
  RCLCPP_SHARED_PTR_DEFINITIONS(MavrosOdomSensor)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  MAVROS_ODOM_SENSOR_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  MAVROS_ODOM_SENSOR_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  MAVROS_ODOM_SENSOR_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  MAVROS_ODOM_SENSOR_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  MAVROS_ODOM_SENSOR_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MAVROS_ODOM_SENSOR_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MAVROS_ODOM_SENSOR_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MAVROS_ODOM_SENSOR_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  struct ThrusterConfig
  {
    rcl_interfaces::msg::Parameter param;
    int channel;
    int neutral_pwm = 1500;
  };

  void stop_thrusters();
  std::vector<std::string> split(const std::string &string_val, char delimiter);
  std::string concatenate_strings(const std::vector<std::string> &vec);

  // We need a node to interact with MAVROS
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> node_odom_;

  std::shared_ptr<rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>> override_rc_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>> rt_override_rc_pub_;

  std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> set_params_client_;
  std::vector<ThrusterConfig> thruster_configs_;

  int max_retries_;
  std::string namespace_;
  std::vector<double> hw_commands_pwm_;

  // Params for creating state interfaces from odometry
  struct StateInterfaceConfig
  {
    nav_msgs::msg::Odometry current_odom;
    geometry_msgs::msg::Twist euler_current_odom;
    std::vector<std::string> state_interface_names;
  };

  geometry_msgs::msg::Twist euler_from_quaternion(const nav_msgs::msg::Odometry odom_with_quaternion);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr topic_based_odometry_subscriber_;
  StateInterfaceConfig state_interface_config_;
  std::string is_odom_state_interface_configured_;
};

}  // namespace mavros_odom_sensor
