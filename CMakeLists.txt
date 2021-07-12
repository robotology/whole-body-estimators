# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(whole-body-estimators LANGUAGES CXX
                              VERSION 0.5.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## avoid having M_PI error by importing math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)
# and export all symbols in Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)


option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

set(YARP_REQUIRED_VERSION 3.0.1)

find_package(YARP ${YARP_REQUIRED_VERSION} REQUIRED)
find_package(Eigen3 3.2.92 REQUIRED)

yarp_configure_plugins_installation(${PROJECT_NAME})

add_subdirectory(libraries)
add_subdirectory(idl)
add_subdirectory(devices)
add_subdirectory(scripts)

include(AddUninstallTarget)

