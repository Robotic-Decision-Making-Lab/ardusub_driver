#ifndef MAVROS_ODOM_SENSOR__VISIBILITY_CONTROL_H_
#define MAVROS_ODOM_SENSOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MAVROS_ODOM_SENSOR_EXPORT __attribute__ ((dllexport))
    #define MAVROS_ODOM_SENSOR_IMPORT __attribute__ ((dllimport))
  #else
    #define MAVROS_ODOM_SENSOR_EXPORT __declspec(dllexport)
    #define MAVROS_ODOM_SENSOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef MAVROS_ODOM_SENSOR_BUILDING_LIBRARY
    #define MAVROS_ODOM_SENSOR_PUBLIC MAVROS_ODOM_SENSOR_EXPORT
  #else
    #define MAVROS_ODOM_SENSOR_PUBLIC MAVROS_ODOM_SENSOR_IMPORT
  #endif
  #define MAVROS_ODOM_SENSOR_PUBLIC_TYPE MAVROS_ODOM_SENSOR_PUBLIC
  #define MAVROS_ODOM_SENSOR_LOCAL
#else
  #define MAVROS_ODOM_SENSOR_EXPORT __attribute__ ((visibility("default")))
  #define MAVROS_ODOM_SENSOR_IMPORT
  #if __GNUC__ >= 4
    #define MAVROS_ODOM_SENSOR_PUBLIC __attribute__ ((visibility("default")))
    #define MAVROS_ODOM_SENSOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MAVROS_ODOM_SENSOR_PUBLIC
    #define MAVROS_ODOM_SENSOR_LOCAL
  #endif
  #define MAVROS_ODOM_SENSOR_PUBLIC_TYPE
#endif

#endif  // MAVROS_ODOM_SENSOR__VISIBILITY_CONTROL_H_
