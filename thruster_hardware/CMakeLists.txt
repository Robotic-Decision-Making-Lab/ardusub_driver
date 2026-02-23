cmake_minimum_required(VERSION 3.23)
project(thruster_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(thruster_hardware SHARED)
target_sources(
    thruster_hardware
    PRIVATE src/thruster_hardware.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/thruster_hardware/thruster_hardware.hpp
)
target_compile_features(thruster_hardware PUBLIC cxx_std_23)
target_link_libraries(
    thruster_hardware
    PUBLIC
        ${mavros_msgs_TARGETS}
        ${rcl_interfaces_TARGETS}
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        pluginlib::pluginlib
        hardware_interface::hardware_interface
        realtime_tools::realtime_tools
)

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
ament_export_dependencies(
    "rclcpp"
    "rclcpp_lifecycle"
    "pluginlib"
    "hardware_interface"
    "mavros_msgs"
    "rcl_interfaces"
    "realtime_tools"
)

ament_package()
