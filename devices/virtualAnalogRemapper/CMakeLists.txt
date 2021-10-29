# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+

find_package(YARP REQUIRED)

yarp_prepare_plugin(virtualAnalogRemapper CATEGORY device
                                          TYPE yarp::dev::VirtualAnalogRemapper
                                          INCLUDE "VirtualAnalogRemapper.h"
                                          INTERNAL
                                          ADVANCED)

if(ENABLE_virtualAnalogRemapper)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})


    yarp_add_plugin(virtualAnalogRemapper VirtualAnalogRemapper.h VirtualAnalogRemapper.cpp)

    target_link_libraries(virtualAnalogRemapper ${YARP_LIBRARIES})

    yarp_install(TARGETS virtualAnalogRemapper
                 EXPORT ${PROJECT_NAME}
                 COMPONENT runtime
                 LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES virtualAnalogRemapper.ini DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()

