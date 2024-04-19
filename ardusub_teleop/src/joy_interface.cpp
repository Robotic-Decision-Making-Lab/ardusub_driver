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

#include "joy_interface.hpp"

namespace ardusub_teleop
{

namespace
{

int scale_cmd(float value, int old_min, int old_max, int new_min, int new_max)
{
  return static_cast<int>((((value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min);
}

}  // namespace

JoyInterface::JoyInterface()
: Node("joy_interface")
{
  this->declare_parameter("min_pwm", 1100);
  this->declare_parameter("max_pwm", 1900);

  pwm_range_ = std::make_tuple(this->get_parameter("min_pwm").as_int(), this->get_parameter("max_pwm").as_int());

  enable_pwm_service_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_pwm_control", [this](
                              const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response) {    // NOLINT
      pwm_enabled_ = request->data;
      response->success = true;
      return;
    });

  const auto qos_profile = rclcpp::SystemDefaultsQoS();
  velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);
  rc_override_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", qos_profile);

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {  // NOLINT
      // If not using PWM control, publish the velocity message directly
      if (!pwm_enabled_) {
        velocity_pub_->publish(*msg);
      }

      mavros_msgs::msg::OverrideRCIn rc_msg;

      for (uint16_t & channel : rc_msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }

      // Scale the velocity commands to the PWM range
      rc_msg.channels[4] = scale_cmd(msg->linear.x, -1.0, 1.0, std::get<0>(pwm_range_), std::get<1>(pwm_range_));
      rc_msg.channels[5] = scale_cmd(-1 * msg->linear.y, -1.0, 1.0, std::get<0>(pwm_range_), std::get<1>(pwm_range_));
      rc_msg.channels[2] = scale_cmd(msg->linear.z, -1.0, 1.0, std::get<0>(pwm_range_), std::get<1>(pwm_range_));
      rc_msg.channels[3] = scale_cmd(-1 * msg->angular.z, -1.0, 1.0, std::get<0>(pwm_range_), std::get<1>(pwm_range_));

      rc_override_pub_->publish(rc_msg);
    });
}

}  // namespace ardusub_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ardusub_teleop::JoyInterface>());
  rclcpp::shutdown();
  return 0;
}
