################################################################################
#                                                                              #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# @authors: Prashanth Ramadoss <prashanth.ramadoss@iit.it>
#           Giulio Romualdi    <giulio.romualdi@iit.it>
#           Silvio Traversaro  <silvio.traversaro@iit.it>
#           Daniele Pucci      <daniele.pucci@iit.it>


cmake_minimum_required(VERSION 3.5)
project(icubFloatingBaseEstimatorV1
        VERSION 1.0
        LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

set(YARP_REQUIRED_VERSION 3.0.1)
set(iDynTree_REQUIRED_VERSION 0.11.0)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

find_package(YARP REQUIRED)
if(${YARP_VERSION} VERSION_LESS ${YARP_REQUIRED_VERSION})
    message(FATAL_ERROR "YARP version ${YARP_VERSION} not sufficient, at least version ${YARP_REQUIRED_VERSION} is required.")
endif()


find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
if(${iDynTree_VERSION} VERSION_LESS ${iDynTree_REQUIRED_VERSION})
    message(FATAL_ERROR "iDyntree version ${iDynTree_VERSION} not sufficient, at least version ${iDynTree_REQUIRED_VERSION} is required.")
endif()

find_package(ICUB REQUIRED)
find_package(codyco-modules REQUIRED)


set(FBE_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/icubFloatingBaseEstimatorV1.h
                ${CMAKE_CURRENT_SOURCE_DIR}/include/WalkingLogger.tpp
                ${CMAKE_CURRENT_SOURCE_DIR}/include/WalkingLogger.hpp
                ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils.tpp
                ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils.hpp)

set(FBE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/icubFloatingBaseEstimatorV1.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/fbeRobotInterface.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/configureEstimator.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/WalkingLogger.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/Utils.cpp)

yarp_prepare_plugin(icubFloatingBaseEstimatorV1 CATEGORY device
                                                TYPE yarp::dev::icubFloatingBaseEstimatorV1
                                                INCLUDE ${FBE_HEADERS}
                                                DEFAULT ON)

yarp_add_idl(THRIFT "${CMAKE_CURRENT_SOURCE_DIR}/thrifts/floatingBaseEstimationRPC.thrift")
add_library(floatingBaseEstimationRPC-service STATIC ${THRIFT})
target_include_directories(floatingBaseEstimationRPC-service PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/include)
target_link_libraries(floatingBaseEstimationRPC-service YARP::YARP_init YARP::YARP_OS)
set_property(TARGET floatingBaseEstimationRPC-service PROPERTY POSITION_INDEPENDENT_CODE ON)

yarp_add_plugin(icubFloatingBaseEstimatorV1  ${FBE_SOURCES} ${FBE_HEADERS})

target_include_directories(icubFloatingBaseEstimatorV1 PRIVATE
                                                       ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                       ${Eigen3_INCLUDE_DIRS})

target_link_libraries(icubFloatingBaseEstimatorV1 ${YARP_LIBRARIES}
                                                  ${iDynTree_LIBRARIES}
                                                  floatingBaseEstimationRPC-service
                                                  ${codyco-modules_LIBRARIES})

yarp_install(TARGETS icubFloatingBaseEstimatorV1
             COMPONENT Runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}/
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}/)

yarp_install(FILES icubFloatingBaseEstimatorV1.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}/)

add_subdirectory(app)
