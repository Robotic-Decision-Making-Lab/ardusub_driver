cmake_minimum_required(VERSION 3.8)
project(ardusub_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  generate_parameter_library
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mavros_msgs
  geometry_msgs
  geographic_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(ardusub_manager_parameters
  src/ardusub_manager_parameters.yaml
)

add_executable(ardusub_manager src/ardusub_manager.cpp)

target_compile_features(ardusub_manager PUBLIC cxx_std_17)
target_link_libraries(ardusub_manager PUBLIC ardusub_manager_parameters)

ament_target_dependencies(ardusub_manager PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    ardusub_manager
    ardusub_manager_parameters
  DESTINATION lib/ardusub_manager
)

install(
  DIRECTORY launch
  DESTINATION share/ardusub_manager
)

ament_package()
