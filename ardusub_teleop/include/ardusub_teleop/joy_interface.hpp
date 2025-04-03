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

#include <memory>
#include <tuple>

#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace ardusub_teleop
{

class JoyInterface : public rclcpp::Node
{
public:
  JoyInterface();

private:
  bool pwm_enabled_{true};
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> enable_pwm_service_;

  std::tuple<int, int> pwm_range_{1100, 1900};
  std::shared_ptr<rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>> rc_override_pub_;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_sub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> velocity_pub_;
};

}  // namespace ardusub_teleop
