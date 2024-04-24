cmake_minimum_required(VERSION 3.0.2)
project(move_to_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf
  tf2_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
    geometry_msgs
    rospy
    std_msgs
    tf
    tf2_ros
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)