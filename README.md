# whole-body-estimators ![C++ CI Workflow](https://github.com/robotology/whole-body-estimators/workflows/C++%20CI%20Workflow/badge.svg)
YARP-based estimators for humanoid robots.

# Overview
 - [:orange_book: Implementation](#orange_book-implementation)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)
 - [:hammer: Build the suite](#hammer-build-the-suite)

# :orange_book: Implementation
The current implementations available in the `devices` folder include,
- `baseEstimatorV1` includes a simple humanoid floating base estimation algorithm fusing legged odometry and IMU attitude estimation, aimed to be used along side walking controllers.

![Floating Base Estimation Algorithm V1](doc/resources/fbeV1.png)

- `wholeBodyDynamics` primarily implements external contact wrench and internal joint torques estimation using various sensor modalities of the robot. Please see the document [Force Control on iCub](doc/howto/force_control_on_icub.md) for more details (This is ported from its original archived repository [robotology-legacy/codyco-modules](https://github.com/robotology-legacy/codyco-modules)). It additionally includes supporting algorithms for  gravity compensation torques estimation, and force-torque sensors calibration.


# :page_facing_up: Dependencies
* [YCM](https://github.com/robotology/ycm) extra CMake Modules for YARP and friends.
* [YARP](http://www.yarp.it/): to handle the comunication with the robot;
* [ICUB](https://github.com/robotology/icub-main): to use the utilities like low pass filters from the `ctrLib` library
* [iDynTree](https://github.com/robotology/idyntree/tree/devel): to setup the floating base estimation algorithm. Please compile iDynTree with CMake option IDYNTREE_USES_ICUB_MAIN ON (depends on ICUB).
* [Gazebo](http://gazebosim.org/): for the simulation (tested Gazebo 8 and 9).

  ## Optional Dependencies
  * [walking-controllers](https://github.com/robotology/walking-controllers): to test the floating base estimation along side walking controllers

It must be noted that all of these dependencies can be directly installed together in one place using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), in particular enabling its `Dynamics` component. 

# :hammer: Build the suite
## Linux

```sh
git clone https://github.com/robotology/whole-body-estimators.git
cd whole-body-estimators
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. In this case it is necessary to add in the `.bashrc` or `.bash_profile` the following lines:
``` sh
export WBDEstimator_INSTALL_DIR=/path/where/you/installed
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${WBDEstimator_INSTALL_DIR}/share/yarp
```
Note that this is not necessary if you install `whole-body-estimators` via the `robotology-superbuild` .

# Using the estimators
- [`baseEstimatorV1`](devices/baseEstimatorV1/README.md) Please follow the documentation available here to run the floating base estimator.
- [`wholeBodyDynamics`](devices/wholeBodyDynamics/README.md) Please follow the documentation for a description of features/parameters of the device and please follow the documentation in [Force Control on iCub](doc/howto/force_control_on_icub.md) for running the whole body dynamics device.

# Authors
```
Hosameldin Awadalla Omer Mohamed <hosameldin.mohamed@iit.it>
Francisco Javier Andrade Chavez <FranciscoJavier.AndradeChavez@iit.it>
Prashanth Ramadoss <prashanth.ramadoss@iit.it>
Giulio Romualdi    <giulio.romualdi@iit.it>
Silvio Traversaro  <silvio.traversaro@iit.it>
Daniele Pucci      <daniele.pucci@iit.it>
```
