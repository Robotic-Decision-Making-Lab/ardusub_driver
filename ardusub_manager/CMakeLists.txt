cmake_minimum_required(VERSION 3.23)
project(ardusub_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(ardusub_manager_parameters
  src/ardusub_manager_parameters.yaml
)

add_executable(ardusub_manager)
target_sources(ardusub_manager PRIVATE src/ardusub_manager.cpp)
target_link_libraries(
    ardusub_manager
    PUBLIC
        ${mavros_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${geographic_msgs_TARGETS}
        ardusub_manager_parameters
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        tf2::tf2
        tf2_ros::tf2_ros
)
target_compile_features(ardusub_manager PUBLIC cxx_std_23)

install(
    TARGETS
        ardusub_manager
        ardusub_manager_parameters
        DESTINATION
        lib/ardusub_manager
)

install(DIRECTORY launch DESTINATION share/ardusub_manager)

ament_package()
