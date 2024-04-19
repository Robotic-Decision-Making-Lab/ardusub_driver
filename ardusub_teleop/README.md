# ardusub_teleop

This package provides tools for controlling an ArduSub-enabled vehicle with a
gamepad.

## joy_interface

joy_interface is a simple node that converts `geometry_msgs/Twist` messages
into `mavros_msgs/OverrideRCIn` messages and publishes them on the topic
`/mavros/rc/override`.

### Subscribers

* /joy_interface/cmd [geometry_msgs::msg::Twist]

### Services

* /joy_interface/enable_pwm_control [std_srvs::srv::SetBool]

### Publishers

* /cmd_vel [geometry_msgs::msg::Twist]
* /mavros/rc/override [mavros_msgs::msg::OverrideRCIn]

## Launch Files

* `teleop.launch.yaml`: launches [joy_interface](#joy_interface), [joy_teleop](https://github.com/ros-teleop/teleop_tools),
  and [joy_linux](https://github.com/ros-drivers/joystick_drivers/tree/ros2).

## Configuration Files

* `joy_teleop.yaml`: specifies gamepad configurations for switching between
  common ArduSub flight modes, toggling velocity and PWM control, and
  performing manual control with a joystick.

> [!IMPORTANT]
> The current configuration uses the left trigger as a deadman switch for
> manual control. This button must be pressed and released to enable manual
> control.
