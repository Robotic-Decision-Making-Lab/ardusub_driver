cmake_minimum_required(VERSION 3.8)
project(ardusub_bringup)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY launch
  DESTINATION share/ardusub_bringup
)

ament_package()
