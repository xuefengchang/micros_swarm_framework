cmake_minimum_required(VERSION 2.8.3)
project(micros_swarm_framework)

#use c++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# set communication mechanism
option(USE_ROS "Using ROS" OFF)
set(USE_ROS ON CACHE BOOL "Using ROS" FORCE)
#message("ROS?" ${ROS})
option(USE_OPENSPLICE_DDS "Using OpenSplice DDS" OFF)
set(USE_OPENSPLICE_DDS OFF CACHE BOOL "Using OpenSplice DDS" FORCE)
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

## Opensplice DDS pub/sub C++ library
if (${USE_OPENSPLICE_DDS})
  add_library(opensplice_dds_comm
    src/opensplice_dds/MSFPPacket.cpp
    src/opensplice_dds/MSFPPacketDcps.cpp
    src/opensplice_dds/MSFPPacketDcps_impl.cpp
    src/opensplice_dds/MSFPPacketSplDcps.cpp
    src/opensplice_dds/check_status.cpp
    src/opensplice_dds/MSFPPacket_listener.cpp
    src/opensplice_dds/publisher.cpp
    src/opensplice_dds/subscriber.cpp
  )
  target_link_libraries(opensplice_dds_comm
    $ENV{OSPL_HOME}/lib/libddskernel.so
    $ENV{OSPL_HOME}/lib/libdcpsisocpp.so
  )
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
  swarm_lib
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  MSFPPacket.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  AppLoad.srv
  AppUnload.srv
  RTPDestroy.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs message_runtime
)

link_directories(
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARY_DIRS}
  ${DDS_LIBRARY_DIRS}
)

#Swarm library directory
add_subdirectory(swarm_lib)

## Declare a C++ library
add_library(rtp_core
  src/runtime_platform.cpp
  src/runtime_platform_core.cpp
)
add_dependencies(rtp_core ${micros_swarm_framework_EXPORTED_TARGETS})

target_link_libraries(rtp_core
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARIES}
)
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(rtp_core opensplice_dds_comm)
endif()

add_library(rtp_manager
  src/runtime_platform_manager.cpp
)
add_dependencies(rtp_manager ${micros_swarm_framework_EXPORTED_TARGETS})
target_link_libraries(rtp_manager rtp_core)

add_executable(rtp_manager_node
  src/rtp_manager_node.cpp
)
target_link_libraries(rtp_manager_node rtp_manager)

add_library(app_loader
  src/app_loader.cpp
)
add_dependencies(app_loader ${micros_swarm_framework_EXPORTED_TARGETS})

add_executable(app_loader_node
  src/app_loader_node.cpp
)
target_link_libraries(app_loader_node app_loader)
target_link_libraries(app_loader_node ${catkin_LIBRARIES})

add_library(App1
  apps/app1.cpp
)
add_dependencies(App1 ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(App1 opensplice_dds_comm)
endif()

add_library(App2
  apps/app2.cpp
)
add_dependencies(App2 ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(App2 opensplice_dds_comm)
endif()

add_library(App3
  apps/app3.cpp
)
add_dependencies(App3 ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(App3 opensplice_dds_comm)
endif()

add_library(TestNC
  test/testnc.cpp
)
add_dependencies(TestNC ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(TestNC opensplice_dds_comm)
endif()

add_library(TestVstig
  test/testvstig.cpp
)
add_dependencies(TestVstig ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(TestVstig opensplice_dds_comm)
endif()

add_library(TestBb
   test/testbb.cpp
)
add_dependencies(TestBb ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(TestBb opensplice_dds_comm)
endif()

#add_executable(testrtp test/testrtp.cpp src/runtime_platform.cpp)
#add_dependencies(testrtp ${micros_swarm_framework_EXPORTED_TARGETS})
#target_link_libraries(testrtp
#   ${Boost_THREAD_LIBRARY}
#   ${Boost_REGEX_LIBRARY}
#   ${Boost_SERIALIZATION_LIBRARY}
#   ${catkin_LIBRARIES}
#)

#add_executable(pub src/opensplice_dds/pub.cpp)
#target_link_libraries(pub
#    opensplice_dds_pub_sub
#)

#add_executable(sub src/opensplice_dds/sub.cpp)
#target_link_libraries(sub
#    opensplice_dds_pub_sub
#)

## Mark executables and/or libraries for installation
install(TARGETS rtp_core rtp_manager rtp_manager_node
  app_loader  app_loader_node
  App1 App2 App3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES app_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY swarm_lib apps doc launch maps idl test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)