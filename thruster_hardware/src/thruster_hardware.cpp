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

#include "thruster_hardware/thruster_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace thruster_hardware
{

auto ThrusterHardware::on_init(const hardware_interface::HardwareInfo & info) -> hardware_interface::CallbackReturn
{
  RCLCPP_INFO(logger_, "Initializing the ThrusterHardware interface");  // NOLINT

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger_, "Failed to initialize the ThrusterHardware interface");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto retries_it = info_.hardware_parameters.find("max_set_param_attempts");
  if (retries_it == info_.hardware_parameters.cend()) {
    RCLCPP_ERROR(logger_, "Missing the required parameter 'max_set_param_attempts'");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }
  max_retries_ = std::stoi(retries_it->second);

  // retrieve and validate the joint parameters
  for (const auto & joint : info_.joints) {
    const auto channel_it = joint.parameters.find("channel");
    if (channel_it == joint.parameters.cend()) {
      RCLCPP_ERROR(logger_, "Joint %s is missing the required parameter 'channel'", joint.name.c_str());  // NOLINT
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto channel = std::stoi(channel_it->second);

    const auto name_it = joint.parameters.find("param_name");
    if (name_it == joint.parameters.cend()) {
      RCLCPP_ERROR(logger_, "Joint %s is missing the required parameter 'param_name'", joint.name.c_str());  // NOLINT
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto param_name = name_it->second;

    const auto default_value_it = joint.parameters.find("default_param_value");
    if (default_value_it == joint.parameters.cend()) {
      // NOLINTNEXTLINE
      RCLCPP_ERROR(logger_, "Joint %s is missing the required parameter 'default_param_value'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto default_value = std::stoi(default_value_it->second);

    // store the thruster configurations
    ThrusterConfig config;
    config.param.name = param_name;
    config.param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    config.param.value.integer_value = default_value;
    config.channel = channel;

    if (joint.parameters.find("neutral_pwm") != joint.parameters.cend()) {
      config.neutral_pwm = std::stoi(joint.parameters.at("neutral_pwm"));
    }

    thruster_configs_[joint.name] = config;
  }

  // construct a node to use for interacting with MAVROS
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=thruster_hardware" + info_.name});
  node_ = rclcpp::Node::make_shared("_", options);

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto ThrusterHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> hardware_interface::CallbackReturn
{
  // Reset the command values
  for (const auto & [name, desc] : joint_command_interfaces_) {
    set_command(name, std::numeric_limits<double>::quiet_NaN());
  }

  // configure the publisher for the override RC messages
  override_rc_pub_ =
    node_->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", rclcpp::SystemDefaultsQoS());
  rt_override_rc_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(override_rc_pub_);

  rt_override_rc_pub_->lock();
  for (auto & channel : rt_override_rc_pub_->msg_.channels) {
    channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  }
  rt_override_rc_pub_->unlock();

  // configure a service client to set the ardusub thruster parameters
  using namespace std::chrono_literals;
  set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("mavros/param/set_parameters");
  while (!set_params_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the `mavros/set_parameters` service");  // NOLINT
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "Waiting for the `mavros/set_parameters` service to be available...");  // NOLINT
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
auto ThrusterHardware::stop_thrusters() -> void
{
  if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock()) {
    for (const auto & [name, config] : thruster_configs_) {
      rt_override_rc_pub_->msg_.channels[config.channel - 1] = config.neutral_pwm;
    }
    rt_override_rc_pub_->unlockAndPublish();
  }
}

auto ThrusterHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> hardware_interface::CallbackReturn
{
  std::vector<rcl_interfaces::msg::Parameter> params;
  params.reserve(thruster_configs_.size());

  for (const auto & [name, config] : thruster_configs_) {
    rcl_interfaces::msg::Parameter param;
    param.name = config.param.name;
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    param.value.integer_value = 1;  // Set the thruster parameter values to RC passthrough here
    params.emplace_back(param);
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters = params;

  for (int i = 0; i < max_retries_; ++i) {
    RCLCPP_WARN(logger_, "Attempting to set thruster parameters to RC passthrough...");  // NOLINT

    // Wait until the result is available
    auto future = set_params_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      const auto responses = future.get()->results;
      for (const auto & response : responses) {
        if (!response.successful) {
          RCLCPP_ERROR(logger_, "Failed to set thruster parameter %s.", response.reason.c_str());  // NOLINT
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      RCLCPP_INFO(logger_, "Successfully set thruster parameters to RC passthrough!");  // NOLINT

      // Stop the thrusters before switching to an external controller
      stop_thrusters();

      // start sending rc override messages from the write loop
      is_active_ = true;

      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }

  RCLCPP_ERROR(  // NOLINT
    logger_,
    "Failed to set thruster parameters to passthrough mode after %d attempts. Make sure that the MAVROS parameter "
    "plugin is fully running and configured.",
    max_retries_);

  // don't send rc override messages if we failed to set the parameters
  is_active_ = false;

  return hardware_interface::CallbackReturn::ERROR;
}

auto ThrusterHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> hardware_interface::CallbackReturn
{
  // stop sending rc override messages from the write loop
  is_active_ = false;

  // stop the thrusters before switching out of passthrough mode
  stop_thrusters();

  std::vector<rcl_interfaces::msg::Parameter> params;
  params.reserve(thruster_configs_.size());

  for (const auto & [name, config] : thruster_configs_) {
    params.emplace_back(config.param);
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters = params;

  for (int i = 0; i < max_retries_; ++i) {
    RCLCPP_WARN(logger_, "Attempting to leave RC passthrough mode...");  // NOLINT

    auto future = set_params_client_->async_send_request(request);

    // Wait until the result is available
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      const auto responses = future.get()->results;
      for (const auto & response : responses) {
        if (!response.successful) {
          RCLCPP_ERROR(logger_, "Failed to set thruster parameter %s.", response.reason.c_str());  // NOLINT
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      RCLCPP_INFO(logger_, "Successfully restored the default thruster values!");  // NOLINT

      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }

  RCLCPP_ERROR(  // NOLINT
    logger_,
    "Failed to fully leave passthrough mode after %d attempts. Make sure that the MAVROS parameter plugin is fully "
    "running and configured.",
    max_retries_);

  return hardware_interface::CallbackReturn::ERROR;
}

auto ThrusterHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hardware_interface::return_type
{
  return hardware_interface::return_type::OK;
}

auto ThrusterHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hardware_interface::return_type
{
  if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock() && is_active_) {
    for (const auto & [name, desc] : joint_command_interfaces_) {
      const auto command = get_command(name);
      if (std::isnan(command)) {
        continue;
      }

      const auto config = thruster_configs_.at(desc.prefix_name);
      rt_override_rc_pub_->msg_.channels[config.channel - 1] = static_cast<int>(command);
    }
    rt_override_rc_pub_->unlockAndPublish();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace thruster_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(thruster_hardware::ThrusterHardware, hardware_interface::SystemInterface)
