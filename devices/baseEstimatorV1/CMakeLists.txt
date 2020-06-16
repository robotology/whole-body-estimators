# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(iDynTree_REQUIRED_VERSION 0.11.0)

find_package(iDynTree ${iDynTree_VERSION} REQUIRED)

find_package(ICUB REQUIRED)

find_package(YARP REQUIRED COMPONENTS
             eigen
             os
             sig)

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
                           ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(baseEstimatorV1 PUBLIC ${YARP_LIBRARIES}
                                      YARP::YARP_eigen
                                      YARP::YARP_os
                                      YARP::YARP_sig
                                      ${iDynTree_LIBRARIES}
                                      floatingBaseEstimationRPC-service
                                      PRIVATE Eigen3::Eigen)

yarp_install(TARGETS baseEstimatorV1
             COMPONENT Runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}/
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}/)

yarp_install(FILES baseEstimatorV1.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}/)

add_subdirectory(app)

