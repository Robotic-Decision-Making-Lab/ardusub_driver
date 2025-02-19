#include "mavros_odom_sensor/mavros_odom_sensor.hpp"

namespace mavros_odom_sensor
{
hardware_interface::CallbackReturn MavrosOdomSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("MavrosOdomSensor"), "Initializing MavrosOdomSensor sensor interface.");  // NOLINT

  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("MavrosOdomSensor"), "Failed to initialize the sensor interface.");  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  namespace_ = info_.hardware_parameters.at("namespace");
  if (namespace_ != "") {
    namespace_ = "/" + namespace_ ;
  }

  // Construct a node to use for interacting with MAVROS
  rclcpp::NodeOptions options_odom;
  if (namespace_ != "")
  {
    std::string ns_ = namespace_.substr(0, namespace_.size() - 1);
    options_odom.arguments({"--ros-args", "-r","__ns:=" + ns_, "-r", "__node:=mavros_odom"});
  }
  else
  {
    options_odom.arguments({"--ros-args", "-r", "__node:=mavros_odom"});
  }
  node_odom_ = rclcpp::Node::make_shared("_", options_odom);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Successfully initialized MavrosOdomSensor sensor interface!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MavrosOdomSensor::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("MavrosOdomSensor"), "Configuring the MavrosOdomSensor sensor interface.");

  mavros_odometry_topic_ = info_.hardware_parameters.at("mavros_odom_topic");
  RCLCPP_INFO(rclcpp::get_logger("MavrosOdomSensor"), "Subscribing to odometry topic: %s", mavros_odometry_topic_.c_str());  // NOLINT

  topic_based_odometry_subscriber_ = node_odom_->create_subscription<nav_msgs::msg::Odometry>(
      mavros_odometry_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr odometer_state) { state_interface_config_.current_odom = *odometer_state; });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MavrosOdomSensor::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MavrosOdomSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(namespace_ + "joint_x", "position", &state_interface_config_.euler_current_odom.linear.x);
  state_interfaces.emplace_back(namespace_ + "joint_y", "position", &state_interface_config_.euler_current_odom.linear.y);
  state_interfaces.emplace_back(namespace_ + "joint_z", "position", &state_interface_config_.euler_current_odom.linear.z);
  state_interfaces.emplace_back(namespace_ + "joint_rx", "position", &state_interface_config_.euler_current_odom.angular.x);
  state_interfaces.emplace_back(namespace_ + "joint_ry", "position", &state_interface_config_.euler_current_odom.angular.y);
  state_interfaces.emplace_back(namespace_ + "joint_rz", "position", &state_interface_config_.euler_current_odom.angular.z);

  state_interfaces.emplace_back(namespace_ + "joint_x", "velocity", &state_interface_config_.current_odom.twist.twist.linear.x);
  state_interfaces.emplace_back(namespace_ + "joint_y", "velocity", &state_interface_config_.current_odom.twist.twist.linear.y);
  state_interfaces.emplace_back(namespace_ + "joint_z", "velocity", &state_interface_config_.current_odom.twist.twist.linear.z);
  state_interfaces.emplace_back(namespace_ + "joint_rx", "velocity", &state_interface_config_.current_odom.twist.twist.angular.x);
  state_interfaces.emplace_back(namespace_ + "joint_ry", "velocity", &state_interface_config_.current_odom.twist.twist.angular.y);
  state_interfaces.emplace_back(namespace_ + "joint_rz", "velocity", &state_interface_config_.current_odom.twist.twist.angular.z);

  return state_interfaces;

}

hardware_interface::CallbackReturn MavrosOdomSensor::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MavrosOdomSensor"), "Activating the MavrosOdomSensor sensor interface.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MavrosOdomSensor::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MavrosOdomSensor"), "Deactivating the MavrosOdomSensor sensor interface.");

  return hardware_interface::CallbackReturn::SUCCESS;
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
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_odom_);
    state_interface_config_.euler_current_odom = euler_from_quaternion(state_interface_config_.current_odom);
  }

  return hardware_interface::return_type::OK;
}
} // namespace mavros_odom_sensor

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mavros_odom_sensor::MavrosOdomSensor,
  hardware_interface::SensorInterface)
