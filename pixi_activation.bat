set CMAKE_INSTALL_PREFIX=%CONDA_PREFIX%\Library
set CMAKE_PREFIX_PATH=%CMAKE_INSTALL_PREFIX%;%CMAKE_PREFIX_PATH%
set PYTHONPATH=""
set YARP_DATA_DIRS=%CMAKE_INSTALL_PREFIX%\share\yarp;%CMAKE_INSTALL_PREFIX%\share\ergoCub
