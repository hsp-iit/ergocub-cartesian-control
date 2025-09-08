# List the subdirectory
# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Get subdirs list
subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/conf/)

# Install configuration files
foreach (dir ${subdirs})
  file(GLOB scripts conf/${dir}/*.ini)
  yarp_install(FILES ${scripts} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME}/${dir})
endforeach ()
