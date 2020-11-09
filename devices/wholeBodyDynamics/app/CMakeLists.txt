# Copyright: (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

include(CMakeParseArguments)

# Macro for installing configuration files
macro(install_wbd_robots_files)
  set(_options NO_ROBOT)
  set(_oneValueArgs YARP_ROBOT_NAME)
  set(_multiValueArgs EXTERNAL_WBD_CONF ROBOT_WBD_CONF EXTERNAL_YRI_CONF)
  cmake_parse_arguments(WBD "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" ${ARGN} )

  # Configure and install files for using wholebodydynamics in the robot's yarprobotinterface
  if(NOT ${WBD_NO_ROBOT})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${WBD_ROBOT_WBD_CONF}
                   ${CMAKE_CURRENT_BINARY_DIR}/${WBD_YARP_ROBOT_NAME}/wholebodydynamics.xml
                   @ONLY)

    yarp_install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${WBD_YARP_ROBOT_NAME}/wholebodydynamics.xml
                 DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/${WBD_YARP_ROBOT_NAME}/estimators)
  endif()

  # Configure and install files for using wholebodydynamics externally
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${WBD_EXTERNAL_WBD_CONF}
                 ${CMAKE_CURRENT_BINARY_DIR}/${WBD_YARP_ROBOT_NAME}/wholebodydynamics-external.xml
                 @ONLY)

  yarp_install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${WBD_YARP_ROBOT_NAME}/wholebodydynamics-external.xml
               DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/${WBD_YARP_ROBOT_NAME}/estimators)

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${WBD_EXTERNAL_YRI_CONF}
                 ${CMAKE_CURRENT_BINARY_DIR}/${WBD_YARP_ROBOT_NAME}/launch-wholebodydynamics.xml
                 @ONLY)

  yarp_install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${WBD_YARP_ROBOT_NAME}/launch-wholebodydynamics.xml
               DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/${WBD_YARP_ROBOT_NAME})
endmacro()


# The wholebodydynamics device is a generic device, but at the moment is used mainly in iCub robots.
#
# For this reason, there is a need to have robot-specific configuration files to deal with the different configurations
# in the robots (different number of FT sensors, etc, etc). However most of this files are shared across robots,
# and so we just have a fixed number of configuration files that are installed for several robots.
#
# The main differences across iCub's with both arms and legs are:
#   * The number of the FT sensors, either 4 or 6.
#     As every F/T sensor is mapped to a single YARP device, it is necessary to change the attach list of the device.
#   * Depending on whether the wholebodydynamics is running in the main yarprobotinterface of an ETH icub,
#     or on the yarprobotinterface of a CAN robot, or running externally, the devices that export the IVirtualAnalogSensor
#     interface change: for ETH robots this interfaces are exposed by the same devices that expose the controlboard interfaces,
#     while for wbd running externally and CAN robots this interfaces are exposed by specific devices.
#

# A good list of robots are eth with 6 ft sensors, fortunatly
set(ICUBS_ETH_SIX_FTS iCubDarmstadt01;iCubGenova02;iCubGenova04;iCubNancy01)
foreach(YARP_ROBOT_NAME ${ICUBS_ETH_SIX_FTS})
  message(STATUS ${YARP_ROBOT_NAME})
  install_wbd_robots_files(YARP_ROBOT_NAME ${YARP_ROBOT_NAME}
                           EXTERNAL_WBD_CONF wholebodydynamics-icub-external-and-can-six-fts.xml
                           EXTERNAL_YRI_CONF launch-wholebodydynamics-icub-six-fts.xml
                           ROBOT_WBD_CONF wholebodydynamics-icub-eth-six-fts.xml)   
endforeach()

# We have also several robots that are CAN with 6 ft sensors
# For them, there is no way to work because CanBusVirtualAnalogSensor does not support the IAxisInfo interface
set(ICUBS_CAN_SIX_FTS iCubGenova01;iCubGenova03;iCubParis01;iCubParis02)
foreach(YARP_ROBOT_NAME ${ICUBS_CAN_SIX_FTS})
  message(STATUS ${YARP_ROBOT_NAME})
  install_wbd_robots_files(YARP_ROBOT_NAME ${YARP_ROBOT_NAME}
                           EXTERNAL_WBD_CONF wholebodydynamics-icub-external-and-can-six-fts.xml
                           EXTERNAL_YRI_CONF launch-wholebodydynamics-icub-six-fts.xml
                           NO_ROBOT)
endforeach()

#
# iCubHeidelberg01 is one of a kind (a iCub with only legs and torso) and so it has specific configuration files
#
install_wbd_robots_files(YARP_ROBOT_NAME iCubHeidelberg01
                         EXTERNAL_WBD_CONF wholebodydynamics-icubheidelberg01-external.xml
                         EXTERNAL_YRI_CONF launch-wholebodydynamics-icubheidelberg01.xml
                         ROBOT_WBD_CONF    wholebodydynamics-icubheidelberg01-robot.xml)

#
# In the Gazebo simulation no yarprobotinterface is explicitly instantiated, so we install only the remote version
#
install_wbd_robots_files(YARP_ROBOT_NAME icubGazeboSim
                         EXTERNAL_WBD_CONF wholebodydynamics-icub-external-six-fts-sim.xml
                         EXTERNAL_YRI_CONF launch-wholebodydynamics-icub-six-fts-sim.xml
                         NO_ROBOT)

install_wbd_robots_files(YARP_ROBOT_NAME iCubGazeboV2_5
                         EXTERNAL_WBD_CONF wholebodydynamics-icub-external-six-fts-sim.xml
                         EXTERNAL_YRI_CONF launch-wholebodydynamics-icub-six-fts-sim.xml
                         NO_ROBOT)
                         
install_wbd_robots_files(YARP_ROBOT_NAME iCubGazeboV3
                         EXTERNAL_WBD_CONF wholebodydynamics-icub-external-eight-fts-sim.xml
                         EXTERNAL_YRI_CONF launch-wholebodydynamics-icub-eight-fts-sim.xml
                         NO_ROBOT)
                         
                         
