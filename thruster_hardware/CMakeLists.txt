cmake_minimum_required(VERSION 3.8)
project(thruster_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

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

add_library(thruster_hardware)
target_sources(
    thruster_hardware
    PRIVATE src/thruster_hardware.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/thruster_hardware/thruster_hardware.hpp
)
ament_target_dependencies(thruster_hardware
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_features(thruster_hardware PUBLIC cxx_std_23)

pluginlib_export_plugin_description_file(hardware_interface thruster_hardware.xml)

install(
    TARGETS thruster_hardware
    EXPORT export_thruster_hardware
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_thruster_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
