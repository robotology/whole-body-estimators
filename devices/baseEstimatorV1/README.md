# Overview
- [:computer: How to run the simulation](#computer-how-to-run-the-simulation)
- [:running: How to test on iCub](#running-how-to-test-on-icub)

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
4. Run `yarprobotinterface` with corresponding device configuration file.
  - To run `baseEstimatorV1`,

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
