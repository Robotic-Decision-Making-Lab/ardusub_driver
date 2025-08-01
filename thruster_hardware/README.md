# thruster_hardware

thruster_hardware implements a ros2_control system hardware interface for
individual thruster-level control. This is accomplished by dynamically setting
the ArduSub `SERVON_FUNCTION` parameters for each thruster to PWM RCIN mode
using MAVROS.

> [!IMPORTANT]
> Controllers that use the thruster hardware interface should be launched
> *after* MAVROS has fully loaded the "param", "command", and "rc_io" plugins.

> [!CAUTION]
> Please exercise caution when using PWM RCIN mode. This mode disables all
> ArduSub arming checks. It is also recommended that you store a backup of the
> ArduSub parameters should the hardware interface fail to restore the default
> parameters.

## Plugin Library

thruster_hardware/ThrusterHardware

## Command Interfaces

* <joint_name>/pwm

## Parameters

* param_name: The name of ArduSub servo function parameter associated to with
  a specific thruster (e.g., `SERVO1_FUNCTION`). [string]
* default_param_value: The default value of the servo function parameter. The
  thruster hardware will attempt to restore these parameters on deactivation.
  [integer]
* channel: The thruster channel number. It is recommended to avoid using
  channels 7 through 11, as these channels may conflict with other vehicle
  functionalities. [integer]
* desired_param_value: The desired value for the servo function parameter
  (e.g., `51` corresponds to `RCIN1`). [integer]
* neutral_pwm (optional): The neutral PWM value for the thruster. This
  defaults to 1500. [integer]
