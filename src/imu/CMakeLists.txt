cmake_minimum_required(VERSION 2.8.3)
project(odom_imu)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odom_imu
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../../common/user_protocol/include
)

add_library(odom_imu
  src/odom_imu.cpp
)

add_dependencies(odom_imu
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_executable(odom_imu_node
  src/odom_imu_node.cpp
)

add_dependencies(odom_imu_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(odom_imu_node
  ${catkin_LIBRARIES}
  odom_imu
)

