cmake_minimum_required(VERSION 3.8)
project(thruster_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  mavros_msgs
  rcl_interfaces
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(thruster_hardware SHARED
  src/hardware.cpp
)

target_include_directories(thruster_hardware
  PUBLIC
    $<INSTALL_INTERFACE:include/thruster_hardware>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(thruster_hardware PUBLIC cxx_std_17)
target_link_libraries(thruster_hardware
  PUBLIC
    ${rclcpp_LIBRARIES}
)

# Use dllexport instead of dllimport
target_compile_definitions(thruster_hardware PRIVATE "THRUSTER_HARDWARE_BUILDING_DLL")
ament_target_dependencies(thruster_hardware
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface thruster_hardware.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/thruster_hardware
)

install(
  TARGETS
    thruster_hardware
  EXPORT
    export_thruster_hardware
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_thruster_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
