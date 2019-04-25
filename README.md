# whole-body-estimators
YARP-based estimators for humanoid robots.

Reported below are the details for using a simple bipedal floating base estimation algorithm alongside walking controllers.

# Overview
 - [:orange_book: Implementation](#orange_book-implementation)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)
 - [:hammer: Build the suite](#hammer-build-the-suite)
 - [:computer: How to run the simulation](#computer-how-to-run-the-simulation)
 - [:running: How to test on iCub](#running-how-to-test-on-icub)

# :orange_book: Implementation

![Floating Base Estimation Algorithm V1](doc/resources/fbeV1.png)

# :page_facing_up: Dependencies
* [YARP](http://www.yarp.it/): to handle the comunication with the robot;
* [iDynTree](https://github.com/robotology/idyntree/tree/devel): to setup the floating base estimation algorithm;
* [codyco-modules](https://github.com/robotology/codyco-modules): to get contacts information through the whole body dynamics estimation algorithm
* [ICUB](https://github.com/robotology/icub-main): to use the utilities like low pass filters from the `ctrLib` library
* [Gazebo](http://gazebosim.org/): for the simulation (tested Gazebo 8 and 9).

  ## Optional Dependencies
  * [walking-controllers](https://github.com/robotology/walking-controllers): to test the floating base estimation along side walking controllers

It must be noted that all of these dependencies can be directly installed together in one place using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild).


# :hammer: Build the suite
## Linux/macOs

```sh
git clone https://github.com/robotology/walking-controllers.git
cd walking-controllers
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. In this case it is necessary to add in the `.bashrc` or `.bash_profile` the following lines:
``` sh
export BaseEstimator_INSTALL_DIR=/path/where/you/installed
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${BaseEstimator_INSTALL_DIR}/lib/yarp
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${BaseEstimator_INSTALL_DIR}/share/yarp:${BaseEstimator_INSTALL_DIR}/lib/yarp
```

# :computer: How to run the simulation
1. Set the `YARP_ROBOT_NAME` environment variable according to the chosen Gazebo model:
   ```sh
   export YARP_ROBOT_NAME="iCubGazeboV2_5"
   ```
2. Run `yarpserver`
   ``` sh
   yarpserver --write
   ```
3. Run gazebo and drag and drop iCub (e.g. icubGazeboSim or iCubGazeboV2_5):

    ``` sh
    gazebo -slibgazebo_yarp_clock.so
    ```
4. Run `yarprobotinterface`

    ``` sh
     YARP_CLOCK=/clock yarprobotinterface --config launch-fbe-analogsens.xml
    ```
    This launches both the floating base estimation device and the whole body dynamics device.
5. Reset the offset of the FT sensors. Open a terminal and write

   ```
   yarp rpc /wholeBodyDynamics/rpc
   >> resetOffset all
   ```

6. communicate with the `base-estimator` through RPC service calls:
   ```
   yarp rpc /base-estimator/rpc
   ```
   the following commands are allowed:
   * `startFloatingBaseFilter`: fill this;
   * `setContactSchmittThreshold lbreak lmake rbreak rmake`: fill this;
   * `setPrimaryFoot foot`: fill this;
   * `useJointVelocityLPF flag`: fill this;
   * `setJointVelocityLPFCutoffFrequency freq`: fill this;
   * `resetLeggedOdometry`: fill this;
   * `resetLeggedOdometryWithRefFrame frame x y z roll pitch yaw`: fill this;
   * `getRefFrameForWorld`: fill this;

## How to dump data
Before run `yarprobotinterface` check if [`dump_data`](app/robots/iCubGazeboV2_5/fbe-analogsens.xml#L14) is set to `true`

If `true`, run the Logger Module
``` sh
YARP_CLOCK=/clock WalkingLoggerModule
```

All the data will be saved in the current folder inside a `txt` named `Dataset_YYYY_MM_DD_HH_MM_SS.txt`

# :running: How to test on iCub
You can follow the same instructions of the simulation section without using `YARP_CLOCK=/clock`. Make sure your `YARP_ROBOT_NAME` is coherent with the name of the robot (e.g. iCubGenova04)
## :warning: Warning
Currently the supported robots are only:
- ``iCubGenova04``
- ``icubGazeboV2_5``
