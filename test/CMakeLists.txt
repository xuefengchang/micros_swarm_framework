#Test
MYSUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
message("-- find tests:")
message("+++++++++++++++++++++++++++++++++++++++++++++")
FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
    message("add test: " ${subdir})
ENDFOREACH()
message("+++++++++++++++++++++++++++++++++++++++++++++")
