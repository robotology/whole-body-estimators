################################################################################
#                                                                              #
# Copyright (C) 2018 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# @authors: Prashanth Ramadoss <prashanth.ramadoss@iit.it>
#           Giulio Romualdi    <giulio.romualdi@iit.it>
#           Silvio Traversaro  <silvio.traversaro@iit.it>
#           Daniele Pucci      <daniele.pucci@iit.it>
#
# Thanks to Stefano Dafarra <stefano.dafarra@iit.it> for this CMakeLists.txt



# List the subdirectory (http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory)
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

# Get list of models
subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/robots/)
# Install each model
foreach (dir ${subdirs})
  yarp_install(DIRECTORY robots/${dir} DESTINATION ${YARP_ROBOTS_INSTALL_DIR})
endforeach ()
