cmake_minimum_required(VERSION 2.8.3)
project(opensplice_dds_comm)

#use c++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# set communication mechanism
option(USE_ROS "Using ROS" OFF)
set(USE_ROS OFF CACHE BOOL "Using ROS" FORCE)
#message("ROS?" ${ROS})
option(USE_OPENSPLICE_DDS "Using OpenSplice DDS" OFF)
set(USE_OPENSPLICE_DDS ON CACHE BOOL "Using OpenSplice DDS" FORCE)
#message("OPENSPLICE_DDS?" ${OPENSPLICE_DDS})
option(USE_OPENDDS "Using OpenDDS" OFF)
set(USE_OPENDDS OFF CACHE BOOL "Using OpenDDS" FORCE)
#message("OPENSPLICE_DDS?" ${OPENSPLICE_DDS})

if(${USE_ROS})
  add_definitions(-DROS)
elseif (${USE_OPENSPLICE_DDS})
  if (NOT DEFINED ENV{OSPL_HOME})
    message(FATAL_ERROR "OpenSplice DDS environment variable OSPL_HOME is not defined")
  endif()
  if (NOT EXISTS $ENV{OSPL_HOME})
    message(FATAL_ERROR "OpenSplice DDS home directory (OSPL_HOME) dosen't exist")
  endif()
  set(DDS_INCLUDE_DIRS $ENV{OSPL_HOME}/include $ENV{OSPL_HOME}/include/sys $ENV{OSPL_HOME}/include/dcps/C++/SACPP $ENV{OSPL_HOME}/include/dcps/C++/isocpp)
  set(DDS_LIBRARY_DIRS $ENV{OSPL_HOME}/lib)
  add_definitions(-DOPENSPLICE_DDS)
elseif (${USE_OPENDDS})
  if (NOT DEFINED ENV{DDS_ROOT}) 
    message(FATAL_ERROR "OpenDDS environment variable DDS_ROOT is not defined")
  endif()
  if (NOT EXISTS $ENV{DDS_ROOT})
    message(FATAL_ERROR "OpenDDS home directory (DDS_ROOT) dosen't exist")
  endif()
  set(DDS_INCLUDE_DIRS $ENV{DDS_ROOT} $ENV{ACE_ROOT}/TAO $ENV{ACE_ROOT} $ENV{DDS_ROOT}/dds/InfoRepo $ENV{DDS_ROOT}/lib)
  set(DDS_LIBRARY_DIRS $ENV{DDS_ROOT}/lib $ENV{ACE_ROOT}/lib)
  add_definitions(-DOPEN_DDS)
else()
  message("communication mechanism are not configured correctly. Using ROS default.")
  add_definitions(-DROS)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
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
  ${DDS_INCLUDE_DIRS}
  include
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp rospy
)

link_directories(
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARY_DIRS}
  ${DDS_LIBRARY_DIRS}
)

## Opensplice DDS pub/sub C++ library
if (${USE_OPENSPLICE_DDS})
  add_library(opensplice_dds_comm
    src/MSFPPacket.cpp
    src/MSFPPacketDcps.cpp
    src/MSFPPacketDcps_impl.cpp
    src/MSFPPacketSplDcps.cpp
    src/check_status.cpp
    src/MSFPPacket_listener.cpp
    src/publisher.cpp
    src/subscriber.cpp
  )
  target_link_libraries(opensplice_dds_comm
    $ENV{OSPL_HOME}/lib/libddskernel.so
    $ENV{OSPL_HOME}/lib/libdcpsisocpp.so
  )
endif()

#add_executable(pub src/opensplice_dds/pub.cpp)
#target_link_libraries(pub
#    opensplice_dds_pub_sub
#)

#add_executable(sub src/opensplice_dds/sub.cpp)
#target_link_libraries(sub
#    opensplice_dds_pub_sub
#)

## Mark executables and/or libraries for installation
if (${USE_OPENSPLICE_DDS})
install(TARGETS opensplice_dds_comm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
endif()
