# Overview
This estimator uses the kinematics, IMU mounted on the head and the contact forces information to estimate the floating base of the robot. The pose of floating base with respect to the inertial frame is computed through legged odometry and is fused along with the attitude estimates from the head IMU. The base velocity is obtained from the contact Jacobian computed using the kinematics information by enforcing the unilateral constraint of the foot.

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
   * `startFloatingBaseFilter`: starts the estimator, this needs to be run after the FT sensors are reset;

   other optional commands include,

   * `setContactSchmittThreshold lbreak lmake rbreak rmake`: used to set contact force thresholds for feet contact detection;
   * `setPrimaryFoot foot`: set the foot to the one that does not break the contact initially during walking, `foot` can be `left` or `right`;
   * `useJointVelocityLPF flag`: use a low pass filter on the joint velocities, `flag` can be `true` or `false`;
   * `setJointVelocityLPFCutoffFrequency freq`: set the cut-off frequency for the low pass filter on the joint velocities;
   * `resetLeggedOdometry`: reset the floating base pose and reset the legged odometry to the inital state;
   * `resetLeggedOdometryWithRefFrame frame x y z roll pitch yaw`: reset the legged odometry by mentioning an intial reference to the world frame with respect to the initial fixed frame;
   * `getRefFrameForWorld`: get the initial fixed frame with respect to which the world frame was set;

## Configuration

The configuration file for the estimator can be found `app/robots/${YARP_ROBOT_NAME}/fbe-analogsens.xml`.
The attitude estimation for the IMU can be chosen to be either a QuaternionEKF or a non-linear complementary filter. The gains should be tuned accordingly. The choice of which IMU to use can also be chosen, however please read the note below.

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

---
**Note**
If you would like to use root link IMU instead of head IMU (default is Head IMU), please uncomment the following lines in `launch-fbe-analogsens.xml`
``` xml
<device name="xsens_inertial" type="genericSensorClient">
        <param name="remote">/icubSim/xsens_inertial</param>
        <param name="local">/baseestimation/waist/imu:i</param>
</device>
```

and the following line in `fbe-analogsens.xml`
```xml
<elem name="root_link_imu_acc">xsens_inertial</elem>
```
---
