cmake_minimum_required(VERSION 2.8.3)
project(lidar_freespace)
find_package(catkin REQUIRED COMPONENTS  roscpp roslib std_msgs sensor_msgs pcl_conversions
        pcl_ros cv_bridge )

find_package(Boost REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)

catkin_package( )

include_directories( ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

add_executable(freespaceNode src/freespace.cpp 
                        src/freespace.h 
                 )
target_link_libraries(freespaceNode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
