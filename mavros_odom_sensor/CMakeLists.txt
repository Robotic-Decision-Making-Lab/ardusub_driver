cmake_minimum_required(VERSION 3.8)
project(mavros_odom_sensor)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  pluginlib
  hardware_interface
  nav_msgs
  mavros_msgs
  tf2
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mavros_odom_sensor.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<INSTALL_INTERFACE:include/mavros_odom_sensor>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

target_compile_features(mavros_odom_sensor PUBLIC cxx_std_17)
target_link_libraries(mavros_odom_sensor
  PUBLIC
    ${rclcpp_LIBRARIES}
)

# Use dllexport instead of dllimport
target_compile_definitions(mavros_odom_sensor PRIVATE "MAVROS_ODOM_SENSOR_BUILDING_DLL")
ament_target_dependencies(mavros_odom_sensor
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface mavros_odom_sensor.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/mavros_odom_sensor
)

install(
  TARGETS
  mavros_odom_sensor
  EXPORT
    export_mavros_odom_sensor
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
ament_export_targets(export_mavros_odom_sensor HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
