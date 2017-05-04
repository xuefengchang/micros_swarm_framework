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
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${DDS_INCLUDE_DIRS}
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
  LIBRARIES micros_swarm_framework
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs message_runtime
)

link_directories(
  ${catkin_LIBRARY_DIRS} 
  ${DDS_LIBRARY_DIRS}
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
)

## Declare a C++ library
if (${USE_OPENSPLICE_DDS})
  add_library(opensplice_dds_pub_sub
    src/opensplice_dds/MSFPPacket.cpp 
    src/opensplice_dds/MSFPPacketDcps.cpp 
    src/opensplice_dds/MSFPPacketDcps_impl.cpp 
    src/opensplice_dds/MSFPPacketSplDcps.cpp
    src/opensplice_dds/check_status.cpp
    src/opensplice_dds/MSFPPacket_listener.cpp
    src/opensplice_dds/publisher.cpp
    src/opensplice_dds/subscriber.cpp
  )
endif()

## Declare a C++ library
add_library(micros_swarm_framework_rtp_core
  src/runtime_platform.cpp
  src/runtime_platform_core.cpp
)
add_dependencies(micros_swarm_framework_rtp_core ${micros_swarm_framework_EXPORTED_TARGETS})

target_link_libraries(micros_swarm_framework_rtp_core
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARIES}
)
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_rtp_core
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_rtp_manager
  src/runtime_platform_manager.cpp
)
add_dependencies(micros_swarm_framework_rtp_manager ${micros_swarm_framework_EXPORTED_TARGETS})
target_link_libraries(micros_swarm_framework_rtp_manager micros_swarm_framework_rtp_core)

add_executable(micros_swarm_framework_rtp_node
  src/rtp_manager_node.cpp
)
target_link_libraries(micros_swarm_framework_rtp_node micros_swarm_framework_rtp_manager)

add_library(app_loader
  src/app_loader.cpp
)
add_dependencies(app_loader ${micros_swarm_framework_EXPORTED_TARGETS})

add_executable(micros_swarm_framework_app_loader
  src/app_loader_node.cpp
)
target_link_libraries(micros_swarm_framework_app_loader app_loader)
target_link_libraries(micros_swarm_framework_app_loader ${catkin_LIBRARIES})

add_library(micros_swarm_framework_App1
  apps/app1.cpp
)
add_dependencies(micros_swarm_framework_App1 ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_App1
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_App2
  apps/app2.cpp
)
add_dependencies(micros_swarm_framework_App2 ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_App2
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_App3
  apps/app3.cpp
)
add_dependencies(micros_swarm_framework_App3 ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_App3
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_OlfatiSaberFlocking
  lib/olfati_saber_flocking.cpp
)
add_dependencies(micros_swarm_framework_OlfatiSaberFlocking ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_OlfatiSaberFlocking
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_TestNC
  test/testnc.cpp
)
add_dependencies(micros_swarm_framework_TestNC ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_TestNC
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_TestVstig
  test/testvstig.cpp
)
add_dependencies(micros_swarm_framework_TestVstig ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_TestVstig
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_TestBb
   test/testbb.cpp
)
add_dependencies(micros_swarm_framework_TestBb ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_TestBb
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
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
#    libddskernel.so
#    libdcpsisocpp.so
#)

#add_executable(sub src/opensplice_dds/sub.cpp)
#target_link_libraries(sub
#    opensplice_dds_pub_sub
#    libddskernel.so
#    libdcpsisocpp.so
#)

add_executable(testpso lib/testpso.cpp)
add_dependencies(testpso ${micros_swarm_framework_EXPORTED_TARGETS})
target_link_libraries(testpso
   ${Boost_THREAD_LIBRARY}
   ${Boost_REGEX_LIBRARY}
   ${Boost_SERIALIZATION_LIBRARY}
   ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS
  micros_swarm_framework_rtp_manager micros_swarm_framework_rtp_node  micros_swarm_framework_app_loader
  micros_swarm_framework_App1 micros_swarm_framework_App2 micros_swarm_framework_App3
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

install(DIRECTORY lib apps doc launch maps idl test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)