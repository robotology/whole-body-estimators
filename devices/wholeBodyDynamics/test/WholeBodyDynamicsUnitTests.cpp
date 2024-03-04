// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <catch2/catch_test_macros.hpp>

// yarprobotinterface
#include <yarp/os/Network.h>
#include <yarp/robotinterface/XMLReader.h>
#include <yarp/dev/PolyDriver.h>

#include <cstdlib>
#include <iostream>

TEST_CASE("WholeBodyDynamicsSmokeTest")
{
    // Add PROJECT_BINARY_DIR/share to YARP_DATA_DIRS to find fakeFTs and wholebodydynamics and fakeFTs devices
    // from the build directory
    std::string oldyarpdatadirs = getenv("YARP_DATA_DIRS");

    #ifdef _WIN32
    std::string pathsep = ";";
    #else
    std::string pathsep = ":";
    #endif
    std::string newyarpdatadirs = std::string("YARP_DATA_DIRS=")+std::string(PROJECT_BINARY_DIR)+"/share/yarp"+pathsep+oldyarpdatadirs;
    #ifdef _WIN32
    _putenv(newyarpdatadirs.c_str());
    #else
    putenv(const_cast<char*>(newyarpdatadirs.c_str()));
    #endif

    // Set YARP_ROBOT_NAME to ensure that the correct ergocub file is found by wholebodydynamics
    std::string newyarprobotname = std::string("YARP_ROBOT_NAME=ergoCubSN001");
    #ifdef _WIN32
    _putenv(newyarprobotname.c_str());
    #else
    putenv(const_cast<char*>(newyarprobotname.c_str()));
    #endif

    // Avoid to need yarpserver
    yarp::os::NetworkBase::setLocalMode(true);

    // Load the yarprobotinterface ergocub_test_all.xml file, and check for errors
    yarp::os::Network yarpInit;
    yarp::robotinterface::XMLReader xmlRobotInterfaceReader;
    yarp::robotinterface::XMLReaderResult xmlRobotInterfaceResult;
    std::string location_ergocub_test_all = std::string(CMAKE_CURRENT_SOURCE_DIR) + std::string("/ergocub_test_all.xml");

    xmlRobotInterfaceResult = xmlRobotInterfaceReader.getRobotFromFile(location_ergocub_test_all);

    REQUIRE(xmlRobotInterfaceResult.parsingIsSuccessful);

    // Enter the startup phase, that will open all the devices and  call attach if necessary
    REQUIRE(xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup));
    REQUIRE(xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseRun));

    // If the robotinterface started, we are good to go, we can close it 
    REQUIRE(xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt1));
    REQUIRE(xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt2));
    REQUIRE(xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt3));
    REQUIRE(xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseShutdown));
}
