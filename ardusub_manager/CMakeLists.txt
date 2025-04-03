cmake_minimum_required(VERSION 3.8)
project(ardusub_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
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

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(ardusub_manager_parameters
  src/ardusub_manager_parameters.yaml
)

add_library(ardusub_manager)
target_sources(
    ardusub_manager
    PRIVATE src/ardusub_manager.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/ardusub_manager/ardusub_manager.hpp
)

ament_target_dependencies(
    ardusub_manager
    PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(ardusub_manager PUBLIC ardusub_manager_parameters)
target_compile_features(ardusub_manager PUBLIC cxx_std_23)

install(
    TARGETS ardusub_manager ardusub_manager_parameters
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

install(DIRECTORY launch DESTINATION share/ardusub_manager)

ament_package()
