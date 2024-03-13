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

#include "thruster_hardware/hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace thruster_hardware
{

hardware_interface::CallbackReturn ThrusterHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("ThrusterHardware"), "Initializing ThrusterHardware system interface.");  // NOLINT

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ThrusterHardware"), "Failed to initialize the hardware interface.");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_pwm_.resize(info_.joints.size(), std::numeric_limits<int>::quiet_NaN());

  // TODO(rakesh): Load the parameters for each joint here
  // TODO(rakesh): We need the ardusub param name, default value, and the thruster channel number for each parameter
  // you might want to just make a struct for this including the parameter value itself and the channel number

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Joint '%s' has %d command interfaces. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Joint '%s' has command interface '%s'. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!joint.state_interfaces.empty()) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Joint '%s' has %d state interfaces. 0 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Construct a node to use for interacting with MAVROS
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=thruster_hardware" + info_.name});
  node_ = rclcpp::Node::make_shared("_", options);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("ThrusterHardware"), "Successfully initialized ThrusterHardware system interface!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrusterHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  override_rc_pub_ =
    node_->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", rclcpp::SystemDefaultsQoS());
  rt_override_rc_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(override_rc_pub_);

  rt_override_rc_pub_->lock();
  for (auto & channel : rt_override_rc_pub_->msg_.channels) {
    channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  }
  rt_override_rc_pub_->unlock();

  set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("mavros/set_parameters");
}

hardware_interface::CallbackReturn ThrusterHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrusterHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // TODO(anyone): Set the thruster parameter values to passthrough here
}

hardware_interface::CallbackReturn ThrusterHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  // TODO(anyone): Set the the thruster parameter values back to their defaults here
}

std::vector<hardware_interface::StateInterface> ThrusterHardware::export_state_interfaces()
{
  // TODO(anyone): haven't decided what to do here yet
}

std::vector<hardware_interface::CommandInterface> ThrusterHardware::export_command_interfaces()
{
  // TODO(anyone): haven't decided what to do here yet
}

hardware_interface::return_type ThrusterHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(anyone): Read the thruster pwm values to write to MAVROS
}

hardware_interface::return_type ThrusterHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(anyone): Publish the PWM values to MAVROS
}

}  // namespace thruster_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(thruster_hardware::ThrusterHardware, hardware_interface::SystemInterface)
