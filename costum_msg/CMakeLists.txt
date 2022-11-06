cmake_minimum_required(VERSION 3.0.2)
project(costum_msg)

## catkin packages 
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    rospy
    message_generation
)

## Generated messages in the 'msg' folder
add_message_files(
    FILES
    complex.msg
)

## Dependencies needed by the costume message
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

## Telling catkin we are using the costume message at runtime
catkin_package(
    CATKIN_DEPENDS message_runtime
)

##
include_directories(
    ${catkin_INCLUDE_DIRS}
)