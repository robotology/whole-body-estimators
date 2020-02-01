# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+

find_package(YARP REQUIRED)


yarp_prepare_plugin(virtualAnalogClient CATEGORY device
                                        TYPE yarp::dev::VirtualAnalogClient
                                        INCLUDE "VirtualAnalogClient.h"
                                        DEFAULT ON
                                        ADVANCED)

if(ENABLE_virtualAnalogClient)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    include_directories(SYSTEM
                        ${YARP_INCLUDE_DIRS})

    yarp_add_plugin(virtualAnalogClient VirtualAnalogClient.h VirtualAnalogClient.cpp)

    target_link_libraries(virtualAnalogClient ${YARP_LIBRARIES})

    yarp_install(TARGETS virtualAnalogClient
                 EXPORT ${PROJECT_NAME}
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES virtualAnalogClient.ini DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
