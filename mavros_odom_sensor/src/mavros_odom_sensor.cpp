#include "mavros_odom_sensor/mavros_odom_sensor.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace mavros_odom_sensor
{

hardware_interface::CallbackReturn MavrosOdomSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("MavrosOdomSensor"), "Initializing MavrosOdomSensor system interface.");  // NOLINT

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("MavrosOdomSensor"), "Failed to initialize the hardware interface.");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_pwm_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Verify that the command and state interface configurations are correct
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("MavrosOdomSensor"), "Joint '%s' has %ld command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != "pwm") {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("MavrosOdomSensor"), "Joint '%s' has command interface '%s'. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), "pwm");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!joint.state_interfaces.empty()) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("MavrosOdomSensor"), "Joint '%s' has %ld state interfaces. 0 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Get the maximum number of attempts that can be made to set the thruster parameters before failing
  if (info_.hardware_parameters.find("max_set_param_attempts") == info_.hardware_parameters.cend()) {
    RCLCPP_ERROR(  // NOLINT
      rclcpp::get_logger("MavrosOdomSensor"), "The 'max_set_param_attempts' parameter is required.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  max_retries_ = std::stoi(info_.hardware_parameters.at("max_set_param_attempts"));
  namespace_ = info_.hardware_parameters.at("namespace");
  if (namespace_ != "") {
    namespace_ = "/" + namespace_ ;
  }

  // Store the thruster configurations
  thruster_configs_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    // Make sure the the joint-level parameters exist
    if (
      joint.parameters.find("param_name") == joint.parameters.cend() ||
      joint.parameters.find("default_param_value") == joint.parameters.cend() ||
      joint.parameters.find("channel") == joint.parameters.cend()) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("MavrosOdomSensor"),
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
  if (namespace_ != "")
  {
    std::string node_name_ = namespace_ + "mavros_odom_sensor" + "/" + info_.name;
    std::vector<std::string> node_name_vec_ = split(node_name_, '/');
    std::string ns_ = concatenate_strings(std::vector<std::string>(node_name_vec_.begin(), node_name_vec_.end() - 1));
    options.arguments({"--ros-args", "-r","__ns:=" + ns_, "-r", "__node:=" + node_name_vec_.back()});
  }
  else
  {
    options.arguments({"--ros-args", "-r", "__node:=mavros_odom_sensor" + info_.name});
  }
  node_ = rclcpp::Node::make_shared("_", options);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Working");


  rclcpp::NodeOptions options_odom;
  options_odom.arguments({"--ros-args", "-r", "__node:=mavros_odom_sensor_odom"});
  node_odom_ = rclcpp::Node::make_shared("_", options_odom);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Successfully initialized MavrosOdomSensor system interface!");


  // Check and Initialize the state interface configuration
  if (info_.hardware_parameters.find("is_odom_si_present") != info_.hardware_parameters.cend()) {
    RCLCPP_INFO(  // NOLINT
      rclcpp::get_logger("MavrosOdomSensor"), "The 'is_odom_si_present' parameter is set to %s.", info_.hardware_parameters.at("is_odom_si_present").c_str());
    is_odom_state_interface_configured_ = info_.hardware_parameters.at("is_odom_si_present");
  } else {
    RCLCPP_INFO(  // NOLINT
      rclcpp::get_logger("MavrosOdomSensor"), "The 'is_odom_si_present' parameter is not set.");
    is_odom_state_interface_configured_ = "false";
}
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MavrosOdomSensor::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Configuring the MavrosOdomSensor system interface.");

  std::string node_name_;
  if (namespace_ != "")
  {
    node_name_ = namespace_ + "rc/override";
  }
  else
  {
    node_name_ = "mavros/rc/override";
  }

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Creating publisher for topic %s", node_name_.c_str());
  override_rc_pub_ =
    node_->create_publisher<mavros_msgs::msg::OverrideRCIn>(node_name_, rclcpp::SystemDefaultsQoS());
  rt_override_rc_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(override_rc_pub_);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "node Pub working");

  rt_override_rc_pub_->lock();
  for (auto & channel : rt_override_rc_pub_->msg_.channels) {
    channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  }
  rt_override_rc_pub_->unlock();
  if (namespace_!= "")
  {
    RCLCPP_INFO(  // NOLINT
      rclcpp::get_logger("MavrosOdomSensor"), "Setting parameters for namespace: %s", namespace_.c_str());
    node_name_ = namespace_ + "param/set_parameters";}
  else{
    node_name_ = "mavros/param/set_parameters";
  }

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Creating publisher for topic %s", node_name_.c_str());

  set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(node_name_);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "node Pub working");

  using namespace std::chrono_literals;
  while (!set_params_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(  // NOLINT
        rclcpp::get_logger("MavrosOdomSensor"), "Interrupted while waiting for the `%s` service.", node_name_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(  // NOLINT
      rclcpp::get_logger("MavrosOdomSensor"), "Waiting for the `%s` service to be available...", node_name_.c_str());
  }

  if (namespace_ != "")
  {
    node_name_ = "model" + namespace_ + "bluerov2/odometry";
  }
  else
  {
    node_name_ = "/model/bluerov2/odometry";
  }

  RCLCPP_INFO(rclcpp::get_logger("MavrosOdomSensor"), "Subscribing to odometry topic: %s",node_name_.c_str());  // NOLINT

  topic_based_odometry_subscriber_ = node_odom_->create_subscription<nav_msgs::msg::Odometry>(
      node_name_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr odometer_state) { state_interface_config_.current_odom = *odometer_state; });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MavrosOdomSensor::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MavrosOdomSensor::stop_thrusters()
{
  if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock()) {
    for (size_t i = 0; i < hw_commands_pwm_.size(); ++i) {
      rt_override_rc_pub_->msg_.channels[thruster_configs_[i].channel - 1] = thruster_configs_[i].neutral_pwm;
    }
    rt_override_rc_pub_->unlockAndPublish();
  }
}

std::vector<std::string> MavrosOdomSensor::split(const std::string &str, char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

std::string MavrosOdomSensor::concatenate_strings(const std::vector<std::string> &vec) {
  std::string result;
  for (size_t i = 0; i < vec.size(); ++i) {
    result += vec[i];
    if (i != vec.size()-1){
      result += "/";
    }
  }
  return result;
}

hardware_interface::CallbackReturn MavrosOdomSensor::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Activating the MavrosOdomSensor system interface.");

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
      rclcpp::get_logger("MavrosOdomSensor"), "Attempting to set thruster parameters to RC passthrough...");

    auto future = set_params_client_->async_send_request(request);

    // Wait until the result is available
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      const auto responses = future.get()->results;
      for (const auto & response : responses) {
        if (!response.successful) {
          RCLCPP_ERROR(  // NOLINT
            rclcpp::get_logger("MavrosOdomSensor"), "Failed to set thruster parameter '%s'.", response.reason.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      RCLCPP_INFO(  // NOLINT
        rclcpp::get_logger("MavrosOdomSensor"), "Successfully set thruster parameters to RC passthrough!");

      // Stop the thrusters before switching to an external controller
      stop_thrusters();

      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }

  RCLCPP_ERROR(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"),
    "Failed to set thruster parameters to passthrough mode after %d attempts. Make sure that the MAVROS parameter "
    "plugin is fully running and configured.",
    max_retries_);

  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn MavrosOdomSensor::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MavrosOdomSensor"), "Activating the MavrosOdomSensor system interface.");  // NOLINT

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
    RCLCPP_WARN(rclcpp::get_logger("MavrosOdomSensor"), "Attempting to leave RC passthrough mode...");  // NOLINT

    auto future = set_params_client_->async_send_request(request);

    // Wait until the result is available
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      const auto responses = future.get()->results;
      for (const auto & response : responses) {
        if (!response.successful) {
          RCLCPP_ERROR(  // NOLINT
            rclcpp::get_logger("MavrosOdomSensor"), "Failed to set thruster parameter '%s'.", response.reason.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      RCLCPP_INFO(  // NOLINT
        rclcpp::get_logger("MavrosOdomSensor"), "Successfully restored the default thruster values!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }

  RCLCPP_ERROR(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"),
    "Failed to fully leave passthrough mode after %d attempts. Make sure that the MAVROS parameter plugin is fully "
    "running and configured.",
    max_retries_);

  return hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> MavrosOdomSensor::export_state_interfaces()
{
  // There are no state interfaces to export
  std::vector<hardware_interface::StateInterface> state_interfaces;

  if (is_odom_state_interface_configured_ == "true")
  {
    state_interfaces.emplace_back(namespace_ + "joint_x", hardware_interface::HW_IF_POSITION, &state_interface_config_.euler_current_odom.linear.x);
    state_interfaces.emplace_back(namespace_ + "joint_y", hardware_interface::HW_IF_POSITION, &state_interface_config_.euler_current_odom.linear.y);
    state_interfaces.emplace_back(namespace_ + "joint_z", hardware_interface::HW_IF_POSITION, &state_interface_config_.euler_current_odom.linear.z);
    state_interfaces.emplace_back(namespace_ + "joint_rx", hardware_interface::HW_IF_POSITION, &state_interface_config_.euler_current_odom.angular.x);
    state_interfaces.emplace_back(namespace_ + "joint_ry", hardware_interface::HW_IF_POSITION, &state_interface_config_.euler_current_odom.angular.y);
    state_interfaces.emplace_back(namespace_ + "joint_rz", hardware_interface::HW_IF_POSITION, &state_interface_config_.euler_current_odom.angular.z);

    state_interfaces.emplace_back(namespace_ + "joint_x", hardware_interface::HW_IF_VELOCITY, &state_interface_config_.current_odom.twist.twist.linear.x);
    state_interfaces.emplace_back(namespace_ + "joint_y", hardware_interface::HW_IF_VELOCITY, &state_interface_config_.current_odom.twist.twist.linear.y);
    state_interfaces.emplace_back(namespace_ + "joint_z", hardware_interface::HW_IF_VELOCITY, &state_interface_config_.current_odom.twist.twist.linear.z);
    state_interfaces.emplace_back(namespace_ + "joint_rx", hardware_interface::HW_IF_VELOCITY, &state_interface_config_.current_odom.twist.twist.angular.x);
    state_interfaces.emplace_back(namespace_ + "joint_ry", hardware_interface::HW_IF_VELOCITY, &state_interface_config_.current_odom.twist.twist.angular.y);
    state_interfaces.emplace_back(namespace_ + "joint_rz", hardware_interface::HW_IF_VELOCITY, &state_interface_config_.current_odom.twist.twist.angular.z);
  }
  return state_interfaces;

}

std::vector<hardware_interface::CommandInterface> MavrosOdomSensor::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "pwm", &hw_commands_pwm_[i]);
  }

  return command_interfaces;
}

geometry_msgs::msg::Twist MavrosOdomSensor::euler_from_quaternion(const nav_msgs::msg::Odometry odom_with_quaternion)
{

  tf2::Quaternion const quat(
      odom_with_quaternion.pose.pose.orientation.x,
      odom_with_quaternion.pose.pose.orientation.y,
      odom_with_quaternion.pose.pose.orientation.z,
      odom_with_quaternion.pose.pose.orientation.w);

  geometry_msgs::msg::Twist euler;

  euler.linear.x = odom_with_quaternion.pose.pose.position.x;
  euler.linear.y = odom_with_quaternion.pose.pose.position.y;
  euler.linear.z = odom_with_quaternion.pose.pose.position.z;

  tf2::Matrix3x3 const rotation_mat(quat);
  rotation_mat.getRPY(euler.angular.x , euler.angular.y, euler.angular.z );

  return euler;
}

hardware_interface::return_type MavrosOdomSensor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (rclcpp::ok())
  {
    if (is_odom_state_interface_configured_ == "true")
    {
      rclcpp::spin_some(node_odom_);
      state_interface_config_.euler_current_odom = euler_from_quaternion(state_interface_config_.current_odom);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MavrosOdomSensor::write(
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

}  // namespace mavros_odom_sensor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mavros_odom_sensor::MavrosOdomSensor, hardware_interface::SystemInterface)
