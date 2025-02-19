// mavros_odom_sensor.hpp
#ifndef SAMPLE_SENSOR_HPP_
#define SAMPLE_SENSOR_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace mavros_odom_sensor
{
class MavrosOdomSensor : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MavrosOdomSensor)  // Remove semicolon, it's included in the macro
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  std::shared_ptr<rclcpp::Node> node_odom_;

  std::string namespace_;
  std::string mavros_odometry_topic_;

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
};

} // namespace mavros_odom_sensor

#endif // SAMPLE_SENSOR_HPP_
