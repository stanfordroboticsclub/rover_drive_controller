cmake_minimum_required(VERSION 2.8.3)
project(rover_drive_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(drive_node src/drive.cpp)

target_link_libraries(drive_node ${catkin_LIBRARIES})

