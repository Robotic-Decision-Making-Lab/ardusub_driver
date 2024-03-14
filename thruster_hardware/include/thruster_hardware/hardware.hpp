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
#include "thruster_hardware/visibility_control.h"

namespace thruster_hardware
{

class ThrusterHardware : public hardware_interface::SystemInterface
{
  RCLCPP_SHARED_PTR_DEFINITIONS(ThrusterHardware)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  THRUSTER_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  THRUSTER_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  THRUSTER_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  THRUSTER_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  THRUSTER_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  struct ThrusterConfig
  {
    rcl_interfaces::msg::Parameter param;
    int channel;
    int neutral_pwm = 1500;
  };

  void stop_thrusters();

  // We need a node to interact with MAVROS
  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>> override_rc_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>> rt_override_rc_pub_;

  std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> set_params_client_;
  std::vector<ThrusterConfig> thruster_configs_;

  int max_retries_;
  std::vector<double> hw_commands_pwm_;
};

}  // namespace thruster_hardware
