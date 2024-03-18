# ardusub_driver

ardusub_driver is a collection of packages designed to enable integration of
ROS 2 into marine vehicles that use ArduSub (e.g., the [BlueROV](https://bluerobotics.com/)).

## Main features

The main features of the ArduSub driver include

* thruster_hardware: A ros2_control hardware interface that enables individual
  thruster-level control. This can be used alongside projects like
  [auv_controllers](https://github.com/Robotic-Decision-Making-Lab/auv_controllers)
  to control marine robots.
* ardusub_manager: A high-level node that simplifies ROS 2 integration with
  marine vehicles by providing interfaces for tasks like setting the interval
  at which MAVLINK messages are published by the autopilot.

## Installation

ardusub_driver is currently supported on Linux and is available for the ROS 2
Iron distribution. Prior to installing this project, first install [MAVROS](https://github.com/mavlink/mavros).
Once MAVROS has been successfully installed, clone ardusub_driver to the
`src` directory of your ROS workspace:

```bash
git clone git@github.com:Robotic-Decision-Making-Lab/ardusub_driver.git
```

Finally, install all ROS dependencies using `rosdep`:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src
```

## Getting help

If you have questions regarding usage of ardusub_driver or regarding
contributing to this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/ardusub_driver/discussions)
board!

## License

ardusub_driver is released under the MIT license.
