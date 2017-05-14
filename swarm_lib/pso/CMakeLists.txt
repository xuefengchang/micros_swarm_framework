include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
aux_source_directory(src DIR_LIB_SRCS)

add_executable(testpso ${DIR_LIB_SRCS})
add_dependencies(testpso ${micros_swarm_framework_EXPORTED_TARGETS})
target_link_libraries(testpso
   ${Boost_THREAD_LIBRARY}
   ${Boost_REGEX_LIBRARY}
   ${Boost_SERIALIZATION_LIBRARY}
   ${catkin_LIBRARIES}
)
