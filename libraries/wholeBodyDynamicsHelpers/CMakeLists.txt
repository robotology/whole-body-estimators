# Copyright (C) 2016 Istituto Italiano di Tecnologia  iCub Facility
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.11)

project(wholeBodyDynamicsHelpers)

set(WHOLEBODYDYNAMICS_HELPERS_HDRS include/wholeBodyDynamics/)

set(WHOLEBODYDYNAMICS_HELPERS_SRCS src/)

add_library(${PROJECT_NAME} ${WHOLEBODYDYNAMICS_HELPERS_HDRS} ${WHOLEBODYDYNAMICS_HELPERS_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
                                           "$<BUILD_INTERFACE:include>"
                                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/wholeBodyDynamics>")
target_include_directories(${PROJECT_NAME} PUBLIC  ${YARP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY PUBLIC_HEADER ${WHOLEBODYDYNAMICS_HELPERS_SRCS})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wholeBodyDynamics)

