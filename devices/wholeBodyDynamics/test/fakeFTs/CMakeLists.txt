yarp_prepare_plugin(fakeFTs
  CATEGORY device
  TYPE fakeFTs
  INCLUDE fakeFTs.h
  DEFAULT ON
  INTERNAL
  ADVANCED
)

yarp_add_plugin(fakeFTs)
target_sources(fakeFTs
  PRIVATE
    fakeFTs.cpp
    fakeFTs.h
  )
target_link_libraries(fakeFTs
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    YARP::YARP_math
)

yarp_install(TARGETS fakeFTs
             EXPORT Runtime
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})