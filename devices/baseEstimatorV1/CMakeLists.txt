# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(iDynTree_REQUIRED_VERSION 0.11.0)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
if(${iDynTree_VERSION} VERSION_LESS ${iDynTree_REQUIRED_VERSION})
    message(FATAL_ERROR "iDyntree version ${iDynTree_VERSION} not sufficient, at least version ${iDynTree_REQUIRED_VERSION} is required.")
endif()

find_package(ICUB REQUIRED)

set(FBE_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/baseEstimatorV1.h
                ${CMAKE_CURRENT_SOURCE_DIR}/include/WalkingLogger.tpp
                ${CMAKE_CURRENT_SOURCE_DIR}/include/WalkingLogger.hpp
                ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils.tpp
                ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils.hpp)

set(FBE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/baseEstimatorV1.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/fbeRobotInterface.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/configureEstimator.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/WalkingLogger.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/Utils.cpp)

yarp_prepare_plugin(baseEstimatorV1 CATEGORY device
                    TYPE yarp::dev::baseEstimatorV1
                    INCLUDE ${FBE_HEADERS}
                    DEFAULT ON)

yarp_add_idl(THRIFT "${CMAKE_CURRENT_SOURCE_DIR}/thrifts/floatingBaseEstimationRPC.thrift")
add_library(floatingBaseEstimationRPC-service STATIC ${THRIFT})
target_include_directories(floatingBaseEstimationRPC-service PUBLIC ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/include)
target_link_libraries(floatingBaseEstimationRPC-service YARP::YARP_init YARP::YARP_OS)
set_property(TARGET floatingBaseEstimationRPC-service PROPERTY POSITION_INDEPENDENT_CODE ON)

yarp_add_plugin(baseEstimatorV1  ${FBE_SOURCES} ${FBE_HEADERS})

target_include_directories(baseEstimatorV1 PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${Eigen3_INCLUDE_DIRS})

target_link_libraries(baseEstimatorV1 ${YARP_LIBRARIES}
                                      ${iDynTree_LIBRARIES}
                                      floatingBaseEstimationRPC-service)

yarp_install(TARGETS baseEstimatorV1
             COMPONENT Runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}/
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}/)

yarp_install(FILES baseEstimatorV1.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}/)

add_subdirectory(app)

