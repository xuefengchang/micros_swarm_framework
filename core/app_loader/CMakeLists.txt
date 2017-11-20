cmake_minimum_required(VERSION 2.8.3)
project(app_loader)

#use c++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
  regex
  serialization
)
    
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  AppLoad.srv
  AppUnload.srv
  RTDestroy.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp rospy std_msgs message_runtime
)

link_directories(
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARY_DIRS}
)

add_library(app_loader
  src/app_loader.cpp
)
add_dependencies(app_loader ${app_loader_EXPORTED_TARGETS})

add_executable(app_loader_node
  src/app_loader_node.cpp
)
target_link_libraries(app_loader_node app_loader)
target_link_libraries(app_loader_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS app_loader  app_loader_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
