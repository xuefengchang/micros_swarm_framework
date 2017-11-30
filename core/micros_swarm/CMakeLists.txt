cmake_minimum_required(VERSION 2.8.3)
project(micros_swarm)

#use c++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
  app_loader
  gsdf_msgs
  message_generation
  roscpp_serialization
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

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp rospy std_msgs message_runtime app_loader gsdf_msgs
)

link_directories(
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(runtime_core
  src/packet_parser.cpp
  src/runtime_handle.cpp
  src/runtime_core.cpp
  src/app_manager.cpp
  src/scds_pso_tuple.cpp
  src/scds_pso.cpp
  src/msg_queue_manager.cpp
  src/check_neighbor.cpp
  src/random.cpp
  src/swarm.cpp
  src/application.cpp
)
add_dependencies(runtime_core gsdf_msgs_gencpp)
add_dependencies(runtime_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(runtime_core
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(daemon_node
  src/daemon_node.cpp
)
target_link_libraries(daemon_node runtime_core)

## Mark executables and/or libraries for installation
install(TARGETS runtime_core daemon_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

