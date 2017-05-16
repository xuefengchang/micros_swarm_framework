cmake_minimum_required(VERSION 2.8.3)
project(comm_interface)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
  message_generation
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS roscpp rospy std_msgs  message_runtime
)

## Mark executables and/or libraries for installation
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
