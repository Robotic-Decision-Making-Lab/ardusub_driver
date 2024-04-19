# ardusub_manager

ardusub_manager provides a high-level interface for some common pieces
of functionality that are used by marine vehicles, including:

* Setting the message intervals for MAVLINK messages
* Setting the EKF origin of the vehicle
* Publishing the `map -> base_link` transformation. Support for this
  *technically* exists in MAVROS, but we have encountered difficulties
  enabling its use with ArduSub, which has encouraged this alternative.

> [!IMPORTANT]
> This node should be launched *after* MAVROS has fully loaded the "local_*"
> and "global_position" plugins.

## Parameters

* message_intervals: List of MAVLINK message IDs to request from the autopilot.
  [N-sized integer array]
* rates: The rates that each message should be published at by the autopilot.
  This should be provided in the same order as the message IDs. [N-sized double
  array]
* set_ekf_origin: Whether or not to set the EKF origin on startup. [bool]
* efk_origin (only required when `set_ekf_origin = True`):
  * latitude: The latitude of the EKF origin. [double]
  * longitude: The longitude of the EKF origin. [double]
  * altitude: The altitude of the EKF origin. [double]
* publish_tf: Whether or not to publish the TF frames for the vehicle. [bool]
