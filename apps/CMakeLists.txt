#Application demos based on micros_swarm_framework
MYSUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
message("-- find applications in apps sub-directory of micros_swarm_framework:")
message("+++++++++++++++++++++++++++++++++++++++++++++")
FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
    message("add app: " ${subdir})
ENDFOREACH()
message("+++++++++++++++++++++++++++++++++++++++++++++")
