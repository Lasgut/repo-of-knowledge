cmake_minimum_required(VERSION 3.0.2)
project(programming_robots_with_ros)

## catkin packages 
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    rospy
)

##
include_directories(
    ${catkin_INCLUDE_DIRS}
)