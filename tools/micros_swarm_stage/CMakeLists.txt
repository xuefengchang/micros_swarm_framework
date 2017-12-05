cmake_minimum_required(VERSION 2.8.3)
project(micros_swarm_stage)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS
  roscpp rospy std_msgs
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(rand_world src/rand_world.cpp)
target_link_libraries(rand_world ${catkin_LIBRARIES})

install(TARGETS rand_world
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
