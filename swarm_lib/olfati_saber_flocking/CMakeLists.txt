
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
aux_source_directory(src DIR_LIB_SRCS)
add_library (OlfatiSaberFlocking ${DIR_LIB_SRCS})
add_dependencies(OlfatiSaberFlocking ${micros_swarm_framework_EXPORTED_TARGETS})
if (${USE_OPENSPLICE_DDS})
  target_link_libraries(OlfatiSaberFlocking opensplice_dds_comm)
endif()