cmake_minimum_required(VERSION 3.0.2)
project(move_to_pose_msg)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
)

add_action_files(
    DIRECTORY action
    FILES MoveToPose.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
)