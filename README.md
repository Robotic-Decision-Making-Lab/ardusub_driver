# ardusub_driver

ardusub_driver is a collection of packages designed to enable integration of
ROS 2 into marine vehicles that use ArduSub (e.g., the [BlueROV](https://bluerobotics.com/)).
You can learn more about the features that ardusub_driver offers in the
documentation provided by each package.

## Installation

ardusub_driver is currently supported on Linux and is available for the ROS 2
Iron and Humble distributions. Prior to installing this project, first install [MAVROS](https://github.com/mavlink/mavros)
and its dependencies. Once MAVROS has been successfully installed, clone
ardusub_driver to the `src` directory of your ROS workspace:

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
