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

#include "ardusub_manager.hpp"

#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace ardusub_manager
{

namespace
{

/**
 * @brief Wait for a future to be ready or for a timeout to occur
 *
 * @note This has been retrieved from the following source:
 * https://github.com/ros2/demos/blob/619c4bc2c9b49fa2073ed19597fc42e882324b2f/lifecycle/src/lifecycle_service_client.cpp#L46
 */
template <typename T>
std::future_status wait_for_result(T & future, std::chrono::seconds timeout)
{
  auto end_t = std::chrono::steady_clock::now() + timeout;
  const std::chrono::milliseconds wait_time(100);
  std::future_status status = std::future_status::timeout;

  do {
    auto current_t = std::chrono::steady_clock::now();
    auto time_left = end_t - current_t;

    if (time_left <= std::chrono::seconds(0)) {
      break;
    }

    status = future.wait_for((time_left < wait_time) ? time_left : wait_time);

  } while (rclcpp::ok() && status != std::future_status::ready);

  return status;
}

}  // namespace

ArduSubManager::ArduSubManager()  // NOLINT
: rclcpp_lifecycle::LifecycleNode("ardusub_manager")
{
}

CallbackReturn ArduSubManager::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring the ArduSub manager...");  // NOLINT

  try {
    param_listener_ = std::make_shared<ardusub_manager::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(  // NOLINT
      this->get_logger(), "An exception occurred while initializing the ArduSub manager: %s\n", e.what());
    return CallbackReturn::ERROR;
  }

  set_ekf_origin_ = params_.set_ekf_origin;

  // Use a reentrant callback group so that we can call the service from within the on_activate/deactivate functions
  set_intervals_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  if (!params_.message_intervals.ids.empty()) {
    if (params_.message_intervals.rates.size() != params_.message_intervals.ids.size()) {
      fprintf(stderr, "The number of message IDs does not match the number of message rates\n");
      return CallbackReturn::ERROR;
    }

    set_message_intervals_client_ = this->create_client<mavros_msgs::srv::MessageInterval>(
      "/mavros/set_message_interval", rclcpp::SystemDefaultsQoS(), set_intervals_callback_group_);
  }

  ekf_origin_pub_ = this->create_publisher<geographic_msgs::msg::GeoPointStamped>(
    "/mavros/global_position/set_gp_origin", rclcpp::SystemDefaultsQoS());

  // Publish the system TF
  if (params_.publish_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {  // NOLINT
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = msg->pose.position.x;
        transform.transform.translation.y = msg->pose.position.y;
        transform.transform.translation.z = msg->pose.position.z;
        transform.transform.rotation = msg->pose.orientation;

        tf_broadcaster_->sendTransform(transform);
      });
  }

  RCLCPP_INFO(this->get_logger(), "Successfully configured the ArduSub manager!");  // NOLINT

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduSubManager::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating the ArduSub manager...");  // NOLINT

  // Set the EKF origin
  if (set_ekf_origin_) {
    while (ekf_origin_pub_->get_subscription_count() == 0) {
      if (!rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the EKF origin subscriber to exist");  // NOLINT
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the EKF origin subscriber to exist...");  // NOLINT
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "Setting the EKF origin");  // NOLINT

    geographic_msgs::msg::GeoPointStamped ekf_origin;
    ekf_origin.header.stamp = this->get_clock()->now();
    ekf_origin.position.latitude = params_.ekf_origin.latitude;
    ekf_origin.position.longitude = params_.ekf_origin.longitude;
    ekf_origin.position.altitude = params_.ekf_origin.altitude;

    ekf_origin_pub_->publish(ekf_origin);
  }

  // Wait for the message interval client to exist
  if (!params_.message_intervals.ids.empty()) {
    while (!set_message_intervals_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_INFO(  // NOLINT
          this->get_logger(), "Interrupted while waiting for the message interval service to exist");
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the message interval service to exist...");  // NOLINT
    }
  }

  // Set the message intervals
  for (size_t i = 0; i < params_.message_intervals.ids.size(); ++i) {
    auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    request->message_id = params_.message_intervals.ids[i];
    request->message_rate = params_.message_intervals.rates[i];

    auto future_result = set_message_intervals_client_->async_send_request(std::move(request)).future.share();
    auto future_status = wait_for_result(future_result, std::chrono::seconds(5));

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(  // NOLINT
        this->get_logger(), "A timeout occurred while attempting to set the message interval for message ID %ld",
        params_.message_intervals.ids[i]);
      RCLCPP_INFO(this->get_logger(), "Failed to activate the ArduSub manager");  // NOLINT
      return CallbackReturn::ERROR;
    }

    if (!future_result.get()->success) {
      RCLCPP_ERROR(  // NOLINT
        this->get_logger(), "Failed to set the message interval for message ID %ld", params_.message_intervals.ids[i]);
      RCLCPP_INFO(this->get_logger(), "Failed to activate the ArduSub manager");  // NOLINT
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(  // NOLINT
      this->get_logger(), "Message %ld set to publish at a rate of %f hz", params_.message_intervals.ids[i],
      params_.message_intervals.rates[i]);
  }

  RCLCPP_INFO(this->get_logger(), "Successfully activated the ArduSub manager!");  // NOLINT

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduSubManager::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduSubManager::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduSubManager::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace ardusub_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<ardusub_manager::ArduSubManager>();
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
