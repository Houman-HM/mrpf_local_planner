cmake_minimum_required(VERSION 2.8.3)
project(mrpf_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  tf
  tf_conversions
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrpf_local_planner
#  CATKIN_DEPENDS base_local_planner costmap_2d geometry_msgs nav_core nav_msgs nav_msgs pluginlib roscpp std_msgs tf tf_conversions
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mrpf_local_planner
  src/mrpf_local_planner.cpp
 )

target_link_libraries(mrpf_local_planner
   ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES}
 )
