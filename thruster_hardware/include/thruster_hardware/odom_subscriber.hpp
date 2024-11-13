#ifndef ODOM_SUBSCRIBER_HPP
#define ODOM_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class OdomSubscriber : public rclcpp::Node
{
public:
  OdomSubscriber()
  : Node("test_odom_subscriber")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/model/bluerov2/odometry", 10, std::bind(&OdomSubscriber::odom_callback, this, _1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received odometry data: Position(%f, %f, %f), Orientation(%f, %f, %f, %f)",
         msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         msg.pose.pose.position.z,
         msg.pose.pose.orientation.x,
         msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

#endif // ODOM_SUBSCRIBER_HPP
