# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+


yarp_prepare_plugin(wholebodydynamics CATEGORY device
                                      TYPE yarp::dev::WholeBodyDynamicsDevice
                                      INCLUDE "WholeBodyDynamicsDevice.h"
                                      INTERNAL
                                      ADVANCED)

if(ENABLE_wholebodydynamics)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    include_directories(SYSTEM
                        ${EIGEN3_INCLUDE_DIR}
                        ${skinDynLib_INCLUDE_DIRS})

    yarp_add_plugin(wholeBodyDynamicsDevice WholeBodyDynamicsDevice.h WholeBodyDynamicsDevice.cpp
                                            SixAxisForceTorqueMeasureHelpers.h SixAxisForceTorqueMeasureHelpers.cpp
                                            GravityCompensationHelpers.h GravityCompensationHelpers.cpp)

    target_link_libraries(wholeBodyDynamicsDevice   wholeBodyDynamicsSettings
                                                    wholeBodyDynamics_IDLServer
                                                    VectorsCollection
                                                    ctrlLibRT
                                                    ${YARP_LIBRARIES}
                                                    skinDynLib
                                                    ${iDynTree_LIBRARIES})

    if(MSVC)
        add_definitions(-D_USE_MATH_DEFINES)
    endif()


    yarp_install(TARGETS wholeBodyDynamicsDevice
                 EXPORT Runtime
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES wholebodydynamics.ini DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

    # Install configuration files
    add_subdirectory(app)

    if(BUILD_TESTING)
        add_subdirectory(test)
    endif()
endif()
