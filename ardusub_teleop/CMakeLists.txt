cmake_minimum_required(VERSION 3.8)
project(ardusub_teleop)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp std_srvs mavros_msgs geometry_msgs)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(joy_interface)
target_sources(
    joy_interface
    PRIVATE src/joy_interface.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/ardusub_teleop/joy_interface.hpp
)

ament_target_dependencies(joy_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(joy_interface PUBLIC cxx_std_23)
target_link_libraries(joy_interface PUBLIC)

install(
    TARGETS joy_interface
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)
install(DIRECTORY config launch DESTINATION share/ardusub_teleop)

ament_package()
