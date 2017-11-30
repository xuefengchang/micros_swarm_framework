cmake_minimum_required(VERSION 2.8.3)
project(udp_bc_broker)

#use c++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
  message_generation
  micros_swarm
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
  ${DDS_INCLUDE_DIRS}
  include
)

link_directories(
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARY_DIRS}
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp rospy message_runtime std_msgs micros_swarm
)

add_library(UDPBCBroker src/udp_bc_broker.cpp src/send.cpp src/recv.cpp)
target_link_libraries(UDPBCBroker
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARIES}
)

#add_executable(test_send src/test_send.cpp src/send.cpp)
#target_link_libraries(test_send ${catkin_LIBRARIES})

#add_executable(test_recv src/test_recv.cpp src/recv.cpp)
#target_link_libraries(test_recv ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS UDPBCBroker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES udp_bc_broker.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
