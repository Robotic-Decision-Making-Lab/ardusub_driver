cmake_minimum_required(VERSION 3.8)
project(ardusub_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/ardusub_bringup)

ament_package()
