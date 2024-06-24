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
#include "rclcpp/rclcpp.hpp"

namespace thruster_hardware
{

hardware_interface::CallbackReturn ThrusterHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("ThrusterHardware"), "Initializing ThrusterHardware system interface.");  // NOLINT

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ThrusterHardware"), "Failed to initialize the hardware interface.");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_pwm_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Verify that the command and state interface configurations are correct
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Joint '%s' has %ld command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != "pwm") {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Joint '%s' has command interface '%s'. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), "pwm");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!joint.state_interfaces.empty()) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Joint '%s' has %ld state interfaces. 0 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Get the maximum number of attempts that can be made to set the thruster parameters before failing
  if (info_.hardware_parameters.find("max_set_param_attempts") == info_.hardware_parameters.cend()) {
    RCLCPP_ERROR(  // NOLINT
      rclcpp::get_logger("ThrusterHardware"), "The 'max_set_param_attempts' parameter is required.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  max_retries_ = std::stoi(info_.hardware_parameters.at("max_set_param_attempts"));

  // Store the thruster configurations
  thruster_configs_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    // Make sure the the joint-level parameters exist
    if (
      joint.parameters.find("param_name") == joint.parameters.cend() ||
      joint.parameters.find("default_param_value") == joint.parameters.cend() ||
      joint.parameters.find("channel") == joint.parameters.cend()) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"),
        "Joint %s missing required configurations. Ensure that the `param_name`, `default_param_value`, and "
        "`channel` are provided for each joint.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    ThrusterConfig config;

    config.param.name = joint.parameters.at("param_name");
    config.param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    config.param.value.integer_value = std::stoi(joint.parameters.at("default_param_value"));
    config.channel = std::stoi(joint.parameters.at("channel"));

    if (joint.parameters.find("neutral_pwm") != joint.parameters.cend()) {
      config.neutral_pwm = std::stoi(joint.parameters.at("neutral_pwm"));
    }

    thruster_configs_.emplace_back(config);
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
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("ThrusterHardware"), "Configuring the ThrusterHardware system interface.");

  override_rc_pub_ =
    node_->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", rclcpp::SystemDefaultsQoS());
  rt_override_rc_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(override_rc_pub_);

  rt_override_rc_pub_->lock();
  for (auto & channel : rt_override_rc_pub_->msg_.channels) {
    channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  }
  rt_override_rc_pub_->unlock();

  set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("mavros/param/set_parameters");

  using namespace std::chrono_literals;
  while (!set_params_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Interrupted while waiting for the `mavros/set_parameters` service.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(  // NOLINT
      rclcpp::get_logger("ThrusterHardware"), "Waiting for the `mavros/set_parameters` service to be available...");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrusterHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ThrusterHardware::stop_thrusters()
{
  if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock()) {
    for (size_t i = 0; i < hw_commands_pwm_.size(); ++i) {
      rt_override_rc_pub_->msg_.channels[thruster_configs_[i].channel - 1] = thruster_configs_[i].neutral_pwm;
    }
    rt_override_rc_pub_->unlockAndPublish();
  }
}

hardware_interface::CallbackReturn ThrusterHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("ThrusterHardware"), "Activating the ThrusterHardware system interface.");

  std::vector<rcl_interfaces::msg::Parameter> params;
  params.reserve(thruster_configs_.size());

  for (const auto & config : thruster_configs_) {
    rcl_interfaces::msg::Parameter param;
    param.name = config.param.name;
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    param.value.integer_value = 1;  // Set the thruster parameter values to RC passthrough here
    params.emplace_back(param);
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters = params;

  for (int i = 0; i < max_retries_; ++i) {
    RCLCPP_WARN(  // NOLINT
      rclcpp::get_logger("ThrusterHardware"), "Attempting to set thruster parameters to RC passthrough...");

    auto future = set_params_client_->async_send_request(request);

    // Wait until the result is available
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      const auto responses = future.get()->results;
      for (const auto & response : responses) {
        if (!response.successful) {
          RCLCPP_ERROR(  // NOLINT
            rclcpp::get_logger("ThrusterHardware"), "Failed to set thruster parameter '%s'.", response.reason.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      RCLCPP_INFO(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Successfully set thruster parameters to RC passthrough!");

      // Stop the thrusters before switching to an external controller
      stop_thrusters();

      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }

  RCLCPP_ERROR(  // NOLINT
    rclcpp::get_logger("ThrusterHardware"),
    "Failed to set thruster parameters to passthrough mode after %d attempts. Make sure that the MAVROS parameter "
    "plugin is fully running and configured.",
    max_retries_);

  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn ThrusterHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ThrusterHardware"), "Activating the ThrusterHardware system interface.");  // NOLINT

  // Stop the thrusters before switching out of passthrough mode
  stop_thrusters();

  std::vector<rcl_interfaces::msg::Parameter> params;
  params.reserve(thruster_configs_.size());

  for (const auto & config : thruster_configs_) {
    params.emplace_back(config.param);
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters = params;

  for (int i = 0; i < max_retries_; ++i) {
    RCLCPP_WARN(rclcpp::get_logger("ThrusterHardware"), "Attempting to leave RC passthrough mode...");  // NOLINT

    auto future = set_params_client_->async_send_request(request);

    // Wait until the result is available
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      const auto responses = future.get()->results;
      for (const auto & response : responses) {
        if (!response.successful) {
          RCLCPP_ERROR(  // NOLINT
            rclcpp::get_logger("ThrusterHardware"), "Failed to set thruster parameter '%s'.", response.reason.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      RCLCPP_INFO(  // NOLINT
        rclcpp::get_logger("ThrusterHardware"), "Successfully restored the default thruster values!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }

  RCLCPP_ERROR(  // NOLINT
    rclcpp::get_logger("ThrusterHardware"),
    "Failed to fully leave passthrough mode after %d attempts. Make sure that the MAVROS parameter plugin is fully "
    "running and configured.",
    max_retries_);

  return hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> ThrusterHardware::export_state_interfaces()
{
  // There are no state interfaces to export
  return std::vector<hardware_interface::StateInterface>();
}

std::vector<hardware_interface::CommandInterface> ThrusterHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "pwm", &hw_commands_pwm_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type ThrusterHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThrusterHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock()) {
    for (size_t i = 0; i < hw_commands_pwm_.size(); ++i) {
      rt_override_rc_pub_->msg_.channels[thruster_configs_[i].channel - 1] = static_cast<int>(hw_commands_pwm_[i]);
    }
    rt_override_rc_pub_->unlockAndPublish();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace thruster_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(thruster_hardware::ThrusterHardware, hardware_interface::SystemInterface)
