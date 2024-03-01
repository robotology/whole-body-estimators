include(WBEAddUnitTest)

add_subdirectory(fakeFTs)

wbe_add_unit_test(NAME WholeBodyDynamics
                  SOURCES WholeBodyDynamicsUnitTests.cpp
                  LINKS YARP::YARP_robotinterface)

target_compile_definitions(WholeBodyDynamicsUnitTests 
                           PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
                                   PROJECT_BINARY_DIR="${PROJECT_BINARY_DIR}")