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
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  nodelet
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

## System dependencies are found with CMake's conventions
##find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  MSFPPacket.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs 
    nav_msgs
    geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES micros_swarm_framework
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs message_runtime 
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

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
add_library(micros_swarm_framework_rtp_kernel
  src/runtime_platform.cpp
  src/runtime_platform_kernel.cpp
)
add_dependencies(micros_swarm_framework_rtp_kernel ${micros_swarm_framework_EXPORTED_TARGETS})

target_link_libraries(micros_swarm_framework_rtp_kernel
  ${Boost_THREAD_LIBRARY}
  ${Boost_REGEX_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${catkin_LIBRARIES}
)
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_rtp_kernel
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_App1Broker
  apps/app1.cpp
  apps/brokers/app1_broker.cpp
)
add_dependencies(micros_swarm_framework_App1Broker ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_App1Broker
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_App2Broker
  apps/app2.cpp
  apps/brokers/app2_broker.cpp
)
add_dependencies(micros_swarm_framework_App2Broker ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_App2Broker
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_App3Broker
  apps/app3.cpp
  apps/brokers/app3_broker.cpp
)
add_dependencies(micros_swarm_framework_App3Broker ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_App3Broker
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_TestNCBroker
  test/testnc.cpp
  test/brokers/testnc_broker.cpp
)
add_dependencies(micros_swarm_framework_TestNCBroker ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_TestNCBroker
    opensplice_dds_pub_sub
    libddskernel.so
    libdcpsisocpp.so
  )
endif()

add_library(micros_swarm_framework_TestVstigBroker
  test/testvstig.cpp
  test/brokers/testvstig_broker.cpp
)
add_dependencies(micros_swarm_framework_TestVstigBroker ${micros_swarm_framework_EXPORTED_TARGETS})

if (${USE_OPENSPLICE_DDS})
  target_link_libraries(micros_swarm_framework_TestVstigBroker
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

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(micros_swarm_framework ${${PROJbut not all) of the required Boost libraries were found.  You may
#need to install these additional Boost librarieECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(micros_swarm_framework_node src/micros_swarm_framework_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(micros_swarm_framework_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(micros_swarm_framework_node
#   ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS micros_swarm_framework_rtp_kernel micros_swarm_framework_App1Broker micros_swarm_framework_App2Broker micros_swarm_framework_App3Broker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY apps doc launch maps idl test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

############# CMake can not determine linker language for target: /usr/lib/x86_64-linux-gnu/libboost_thread.so
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_micros_swarm_framework.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
