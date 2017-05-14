SET(APPLICATION_NAME testbb)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
aux_source_directory(src DIR_LIB_SRCS)

add_library(TestBb ${DIR_LIB_SRCS})
add_dependencies(TestBb ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(TestBb opensplice_dds_comm)
endif()

install(TARGETS TestBb
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES testbb_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${APPLICATION_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #PATTERN ".svn" EXCLUDE
)
