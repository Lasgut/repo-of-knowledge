cmake_minimum_required(VERSION 3.0.2)
project(costum_msg)

## catkin packages 
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    rospy
    message_generation # Needed for costum msg and srv
    actionlib_msgs     # Needed for costum action
)

## Generated messages in the 'msg' directory
add_message_files(
    FILES
    complex.msg
)

## Generated services in the 'srv' directory
add_service_files(
    FILES
    add_two_ints.srv
    WordCount.srv
)

## Generated actions in the 'action' directory
add_action_files(
    DIRECTORY action
    FILES 
    Timer.action 
)

## Dependencies needed by the costume messages
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    actionlib_msgs
)

## Telling catkin we are using the costume message at runtime
catkin_package(
    CATKIN_DEPENDS 
    message_runtime
    actionlib_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)