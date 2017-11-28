cmake_minimum_required(VERSION 2.8.3)
project(opensplice_dds_broker)

#use c++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

option(USE_OPENSPLICE_DDS "Using OpenSplice DDS" OFF)
set(USE_OPENSPLICE_DDS OFF CACHE BOOL "Using OpenSplice DDS" FORCE)

if (${USE_OPENSPLICE_DDS})
  if (NOT DEFINED ENV{OSPL_HOME})
    message(FATAL_ERROR "OpenSplice DDS environment variable OSPL_HOME is not defined")
  endif()
  if (NOT EXISTS $ENV{OSPL_HOME})
    message(FATAL_ERROR "OpenSplice DDS home directory (OSPL_HOME) dosen't exist")
  endif()
  set(DDS_INCLUDE_DIRS $ENV{OSPL_HOME}/include $ENV{OSPL_HOME}/include/sys $ENV{OSPL_HOME}/include/dcps/C++/SACPP $ENV{OSPL_HOME}/include/dcps/C++/isocpp)
  set(DDS_LIBRARY_DIRS $ENV{OSPL_HOME}/lib)
  set(DDS_LIBRARIES dcpssacpp ddskernel)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
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

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp rospy micros_swarm
)

if (${USE_OPENSPLICE_DDS})
link_directories(
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARY_DIRS}
  ${DDS_LIBRARY_DIRS}
)

## Opensplice DDS pub/sub C++ library
add_library(opensplice_dds
  src/GSDFPacket.cpp
  src/GSDFPacketDcps.cpp
  src/GSDFPacketDcps_impl.cpp
  src/GSDFPacketSplDcps.cpp
  src/check_status.cpp
  src/GSDFPacket_listener.cpp
  src/publisher.cpp
  src/subscriber.cpp
)
#add_dependencies(OpenspliceDDSComm ${opensplice_dds_comm_EXPORTED_TARGETS})
target_link_libraries(opensplice_dds
  $ENV{OSPL_HOME}/lib/libddskernel.so
  $ENV{OSPL_HOME}/lib/libdcpsisocpp.so
)

add_library(OpenSpliceDDSBroker src/opensplice_dds_broker.cpp)
target_link_libraries(OpenSpliceDDSBroker opensplice_dds)
target_link_libraries(OpenSpliceDDSBroker ${catkin_LIBRARIES})

#add_executable(pub src/pub.cpp)
#target_link_libraries(pub OpenSpliceDDSBroker)
#target_link_libraries(pub ${catkin_LIBRARIES})

#add_executable(sub src/sub.cpp)
#target_link_libraries(sub OpenSpliceDDSBroker)
#target_link_libraries(sub ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS opensplice_dds OpenSpliceDDSBroker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES opensplice_dds_broker.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
endif()
