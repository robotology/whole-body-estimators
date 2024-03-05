# Overview

For an overview on `wholeBodyDynamics` and to understand how to run the device, please see [Force Control on iCub](../../doc/howto/force_control_on_icub.md). Below, you will find the description of the features/parameters included by the `wholeBodyDynamics` device.



### wholeBodyDynamicsDevice

  A device that takes a list of axes and estimates the joint torques for each one of this axes.

   The parameters taken in input by this device are:

| Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
|:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
| axesNames      |      -         | vector of strings |   -   |   -           | Yes      | Ordered list of the axes that are part of the remapped device.    |       |
| additionalConsideredFixedJoints      |      -         | vector of strings |   -   |   -           | No      | Optional list of fixed joints being omitted by the URDF model parser but are needed to be considered in the joint list.    |       |
| modelFile      |      -         | path to file      |   -   | model.urdf    | No       | Path to the URDF file used for the kinematic and dynamic model.   |       |
| assume_fixed    |                | frame name        |   -   |     -         | No       | If it is present, the initial kinematic source used for estimation will be that specified frame is fixed, and its gravity is specified by fixedFrameGravity. The IMU related parameters from the configuration are not used if this parameter exists. Otherwise, the default IMU will be used. | |
| fixedFrameGravity  |      -     | vector of doubles | m/s^2 | -             | Yes      | Gravity of the frame that is assumed to be fixed, if the kinematic source used is the fixed frame. | |
| imuFrameName   |       -        | string            |   -   |      -        | Yes      | Name of the frame (in the robot model) with respect to which the IMU broadcast its sensor measurements. |  |
| imuFilterCutoffInHz |     -     | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter IMU measures. | The used filter is a simple first order filter. |
| forceTorqueFilterCutoffInHz | - | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter FT measures.  |  The used filter is a simple first order filter. |
| jointVelFilterCutoffInHz    | - | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter joint velocities measures. | The used filter is a simple first order filter. |
| jointAccFilterCutoffInHz    | - | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter joint accelerations measures. | The used filter is a simple first order filter. |
| startWithZeroFTSensorOffsets    | - | bool            | -    |      False       | No      | Use zero FT sensor offsets at startup. If this flag is set to false, the device estimates the offsets of FT sensors at startup||
| disableSensorReadCheckAtStartup | - | bool            | -    |      False       | No      | If this flag is set to true, the read from the sensors is skipped at startup ||
| defaultContactFrames      | -   | vector of strings (name of frames ) |-| - |  Yes     | Vector of default contact frames. If no external force read from the skin is found on a given submodel, the defaultContactFrames list is scanned and the first frame found on the submodel is the one at which origin the unknown contact force is assumed to be. | - |
| alwaysUpdateAllVirtualTorqueSensors | -     |  bool |  -    |      -        |  Yes     | Enforce that a virtual sensor for each estimated axes is available. | Tipically this is set to false when the device is running in the robot, while to true if it is running outside the robot. |
| defaultContactFrames |      -   | vector of strings |  -    |    -          | Yes      | If not data is read from the skin, specify the location of the default contacts | For each submodel induced by the FT sensor, the first not used frame that belongs to that submodel is selected from the list. An error is raised if not suitable frame is found for a submodel. |
| overrideContactFrames |      -   | vector of strings |  -    |    -          | No      | If not data is read from the skin, and this parameter exists, it will override 'defaultContactFrames'  | For each submodel induced by the FT sensor, the frames that belong to that submodel are selected from the list with constraint that the number of variables does not exceed 6. An error is raised if not suitable frame is found for a submodel. |
| contactWrenchType |      -   | vector of strings |  -    |    -          | Yes if 'overrideContactFrames' exists      | It should contain one field for each override frame. The values can be either 'full', 'pure' or 'pureKnown'  | 'full' means the external wrench is full-wrench (6 unknown variables), 'pure' means pure force (3 unknown variables), and 'pureKnown' means pure force with known direction (1 unknown variable). |
| contactWrenchDirection |      -   | vector of doubles |  -    |    -          | Yes if 'overrideContactFrames' exists      | It should contain 3 fields for each override frame.  |     |
| contactWrenchPosition |      -   | vector of doubles |  -    |    -          | Yes if 'overrideContactFrames' exists      | It should contain 3 fields for each override frame.  |     |
| useJointVelocity     |        - | bool              |  -    |      true     |  No      | Select if the measured joint velocities (read from the getEncoderSpeeds method) are used for estimation, or if they should be forced to 0.0 . | The default value of true is deprecated, and in the future the parameter will be required. |
| useJointAcceleration |        - | bool              |  -    |      true     |  No      | Select if the measured joint accelerations (read from the getEncoderAccelerations method) are used for estimation, or if they should be forced to 0.0 . | The default value of true is deprecated, and in the future the parameter will be required. |
| streamFilteredFT     |        - | bool              |  -    |      false    |  No      | Select if the filtered and offset removed forces will be streamed or not. The name of the ports have the following syntax:  portname=(portPrefix+"/filteredFT/"+sensorName). Example: "myPrefix/filteredFT/l_leg_ft" | The value streamed by this ports is affected by the secondary calibration matrix, the estimated offset and temperature coefficients ( if any ). |
| publishNetExternalWrenches |        - | bool          |  -    |      false   |  No      | Flag to stream the net external wrenches acting on each link on the port portPrefix+"/netExternalWrenches:o". The content is a bottle of n pairs, where n is the number of links. The first element of each pair is the name of the link. The second element is yet another list of 6 elements containing the value of the net external wrench, defined in the link frame. The first three elements are the linear part, while the other three are the angular part.| |
| useSkinForContacts     |        - | bool              |  -    |      true    |  No      | Flag to skip using tactile skin sensors for updating contact points and external force (pressure) information | |
| estimateJointVelocityAcceleration     |        - | bool              |  -    |      false    |  No      | Flag to estimate the joint velocities and accelerations using Kalman filter. If true, the same measurements from the low level are ignored. | |
| processNoiseCovariance |      -   | vector of doubles |  -    |    -          | Yes if 'estimateJointVelocityAcceleration' is true      | It should contain 3 values per each joint, in total number_of_joints*3 values. The joint order follows the one in axesNames.  |     |
| measurementNoiseCovariance |      -   | vector of doubles |  -    |    -          | Yes if 'estimateJointVelocityAcceleration' is true      | It should contain number_of_joints values. The joint order follows the one in axesNames.  |     |
| stateCovariance |      -   | vector of doubles |  -    |    -          | Yes if 'estimateJointVelocityAcceleration' is true      | It should contain 3 values per each joint, in total number_of_joints*3 values. The joint order follows the one in axesNames. |     |
| IDYNTREE_SKINDYNLIB_LINKS |  -  | group             | -     | -             | Yes      |  Group describing the mapping between link names and skinDynLib identifiers. | |
|                |   linkName_1   | string (name of a link in the model) | - | - | Yes   | Bottle of three elements describing how the link with linkName is described in skinDynLib: the first element is the name of the frame in which the contact info is expressed in skinDynLib (tipically DH frames), the second a integer describing the skinDynLib BodyPart , and the third a integer describing the skinDynLib LinkIndex  | |
|                |   ...   | string (name of a link in the model) | - | -     | Yes      | Bottle of three elements describing how the link with linkName is described in skinDynLib: the first element is the name of the frame in which the contact info is expressed in skinDynLib (tipically DH frames), the second a integer describing the skinDynLib BodyPart , and the third a integer describing the skinDynLib LinkIndex  | |
|                |   linkName_n   | string (name of a link in the model) | - | - | Yes   | Bottle of three elements describing how the link with linkName is described in skinDynLib: the first element is the name of the frame in which the contact info is expressed in skinDynLib (tipically DH frames), the second a integer describing the skinDynLib BodyPart , and the third a integer describing the skinDynLib LinkIndex  | |
| WBD_OUTPUT_EXTERNAL_WRENCH_PORTS |  -  | group             | -     | -     | Yes       |  Group describing the external forces published on a YARP port by wholeBodyDynamics. | |
|                |   portName_1   | string (name of the port opened to stream the external wrench | - | - | Yes    | Bottle of three elements describing the wrench published on the port: the first element is the link of which the published external wrench is applied. This wrench is expressed around the origin of the frame named as second paramter, and with the orientation of the third parameter.  |  |
|                |   ...   | | - | ..                                        | Yes       | ..  |  |
|                |   portName_n   | .. | - | -                               | Yes       | ..  | |
| outputWrenchPortInfoType |   -   | string | - | netWrench | No       | Variable for choosing the type of information published in the YARP ports of the group "WBD_OUTPUT_EXTERNAL_WRENCH_PORTS". The user can select between "netWrench" and "contactWrenches". Selecting "netWrench" makes the group publish the net external wrench excerted by the environment on the link specified in the group. Selecting "contactWrenches" makes the group publish all the contact wrenches of the link specified in the group. In the latter case the format of the data published in the ports will be `<contactWrenchOfIndex_1> <contactWrenchOfIndex_2> ... <contactWrenchOfIndex_(N-1)>`, and the order of the contact wrenches indexes is the same of when they are added (tested with `overrideContactFrames`).  | If an invalid value is put, the device ignores it and "netWrench" is selected |
| GRAVITY_COMPENSATION |  -       | group             | -     | -            | No        |  Group for providing estimates of the torque necessary to compensate gravity. | Gravity calls setImpedanceOffset when the considered joints is in COMPLIANT_INTERACTION_MODE   |
|                      | enableGravityCompensation | bool | -  | -           | No        |  |  |
|                      | gravityCompensationBaseLink| string | - | -         | No        | ..  | |
|                      | gravityCompensationAxesNames | vector of strings | - | - | No   | Axes for which the gravity compensation is published. | |
| HW_USE_MAS_IMU |  -  | group             | -     | -     | Yes       |  Group to enable attach to Multiple Analog Sensor interface based IMU. | The group is compulsory since whole-body-estimators 0.11.0 . |
|                |   accelerometer   | string  | - | - | Yes    | Should match the sensor id used to open the device containing MAS IThreeAxisLinearAcccelerometers interface  |  |
|                |   gyroscope   | string  | - | - | Yes    | Should match the sensor id used to open the device containing MAS IThreeAxisGyroscopes interface  |  |

  The axes contained in the `axesNames` parameter are then mapped to the wrapped `controlboard` in the `attachAll` method, using `controlBoardRemapper` class.
  Furthermore are also used to match the yarp axes to the joint names found in the passed URDF file.

#### Gravity Compensation
  This device also provides gravity compensation torques (using the `IImpedanceControl::setImpedanceOffset` method) for axis that are in compliant interaction mode and in position/position direct/velocity control mode.

This estimates are obtained just with the model, assuming that there is a link (the `gravityCompensationRootLink`) at which all external forces are exerted.

Typically this estimates are provided only for the upper joints (arms and torso) of the robots, as the gravity compensation terms for the legs depends on the support state of the robot.

 #### Secondary Calibration Matrix
  This device support to specify a secondary calibration matrix to apply on the top of the (already calibrated) measure coming from the F/T sensors.
  This feature is meant to be experimental, and will be removed at any time.

  The group of options that regulated this group is the following:

| Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
|:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
| FT_SECONDARY_CALIBRATION |  -       | group         | -     | -             | No       |  Group for providing secondary calibration matrix for FT sensors. |   |
|                          | ftSensorName_1 | vector of 36 doubles| Unitless, 1/m or m depending on the element. | Identity matrix   | No  | Elements of the  6x6 secondary calibration matrix, specified in row-major order. | The messages coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |
|                          | ...            | vector of 36 doubles| Unitless, 1/m or m depending on the element. | Identity matrix   | No  | Elements of the  6x6 secondary calibration matrix, specified in row-major order. | The messages coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |
|                          | ftSensorName_n | vector of 36 doubles| Unitless, 1/m or m depending on the element. | Identity matrix   | No  | Elements of the  6x6 secondary calibration matrix, specified in row-major order. | The messages coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |

  All sensors not specified will use a 6x6 identity as a secondary calibration matrix.

  Example of part of a configuration file using `.xml` `yarprobotinterface` format (remember to put the fractional dot!).

  ``` xml
       <group name="FT_SECONDARY_CALIBRATION">
              <param name="l_arm_ft">(1.0,0.0,0.0,0.0,0.0,0.0,
                                             0.0,1.0,0.0,0.0,0.0,0.0,
                                             0.0,0.0,1.0,0.0,0.0,0.0,
                                             0.0,0.0,0.0,1.0,0.0,0.0,
                                             0.0,0.0,0.0,0.0,1.0,0.0,
                                             0.0,0.0,0.0,0.0,0.0,1.0)                   </param>
              <param name="r_arm_ft">(1.0,0.0,0.0,0.0,0.0,0.0,
                                             0.0,1.0,0.0,0.0,0.0,0.0,
                                             0.0,0.0,1.0,0.0,0.0,0.0,
                                             0.0,0.0,0.0,0.001,0.0,0.0,
                                             0.0,0.0,0.0,0.0,0.001,0.0,
                                             0.0,0.0,0.0,0.0,0.0,0.001)                 </param>
       </group>
  ```

#### Temperature Compensation Coefficients for FT Calibration and using Pre-estimated FT Offsets

##### Temperature Coefficients
  This device support to specify a set of temperature coefficients matrix to apply on the top of the (already calibrated) measure coming from the F/T sensors.
  :warning:  This feature is meant to be experimental, and will be removed at any time.

  The group of options that regulates this group is the following:

| Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
|:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
| FT_TEMPERATURE_COEFFICIENTS |  -       | group         | -     | -             | No       |  Group for providing secondary calibration matrix for FT sensors. |   |
|                             | ftSensorName_1 | vector of 7 doubles| Unitless, 1/m or m depending on the element. | Zero Vector   | No  | Elements of the  1x7 temperature coefficients matrix, specified in row-major order. | The temperature message coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |
|                             | ...            | vector of 7 doubles| Unitless, 1/m or m depending on the element. | Zero Vector   | No  | Elements of the  1x7 temperature coefficients matrix, specified in row-major order. | The temperature message coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |
|                             | ftSensorName_n | vector of 7 doubles| Unitless, 1/m or m depending on the element. | Zero Vector   | No  | Elements of the  1x7 temperature coefficients matrix, specified in row-major order. | The temperature message coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |

  All sensors not specified will use a 1x7 zero vector as a temperature coefficients matrix, where the first 6 are temperature correction for each axis and the 7th value is the temperature offset.

  Example of part of a configuration file using `.xml` `yarprobotinterface` format (remember to put the fractional dot!).
  ``` xml
       <group name="FT_TEMPERATURE_COEFFICIENTS">
              <param name="l_arm_ft">(0.0,0.0,0.0,0.0,0.0,0.0,0.0)</param>
              <param name="r_arm_ft">(0.0,0.0,0.0,0.0,0.0,0.0,0.0)</param>
       </group>
  ```

 ##### Force Torque Sensor Pre-estimated Offset
  This device support to specify a set an constant offset vector to apply on the top of the (already calibrated) measure coming from the F/T sensors.
  :warning:  This feature is meant to be experimental, and will be removed at any time.

  The group of options that regulates this group is the following:
| Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
|:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
| FT_OFFSET |  -       | group         | -     | -             | No       |  Group for providing secondary calibration matrix for FT sensors. |   |
|           | ftSensorName_1 | vector of 6 doubles| Unitless, 1/m or m depending on the element. | Zero Vector   | No  | Elements of the  1x6 temperature coefficients matrix, specified in row-major order. | The constant offset estimated offline will be substracted to the sensor measurements to get the actual measurement used.  |
|           | ...            | vector of 6 doubles| Unitless, 1/m or m depending on the element. | Zero Vector   | No  | Elements of the  1x6 temperature coefficients matrix, specified in row-major order. | The constant offset estimated offline will be substracted to the sensor measurements to get the actual measurement used.  |
|           | ftSensorName_n | vector of 6 doubles| Unitless, 1/m or m depending on the element. | Zero Vector   | No  | Elements of the  1x6 temperature coefficients matrix, specified in row-major order. | The constant offset estimated offline will be substracted to the sensor measurements to get the actual measurement used.  |

  All sensors not specified will use a 1x6 zero vector as a constant offset, where the 6 are offset for each axis.

  Example of part of a configuration file using `.xml` `yarprobotinterface` format (remember to put the fractional dot!).
  ``` xml
       <group name="FT_OFFSET">
              <param name="l_arm_ft">(0.0,0.0,0.0,0.0,0.0,0.0)</param>
              <param name="r_arm_ft">(0.0,0.0,0.0,0.0,0.0,0.0)</param>
       </group>
  ```

For a detailed explanation on their usage, please see the document [Using temperature coefficients and pre-estimated FT offsets](../../doc/howto/useTemperatureCoefficientsAndOffsetCompensationInWholeBodyDynamics.md).

  #### Filters

  All the filters used for the input measurements are using the `iCub::ctrl::realTime::FirstOrderLowPassFilter` class.

  #### Configuration Examples

  Example configuration file using `.ini` format.

  ```
   device wholebodydynamics
   axesNames (joint1 joint2 joint3)

  ...
  ```

  Example configuration file using `.xml` `yarprobotinterface` format.
  ``` xml
  <devices robot="iCubGenova02" build="1">
      <device name="wholebodydynamics" type="wholebodydynamics">
          <param name="axesNames">(torso_pitch,torso_roll,torso_yaw,neck_pitch, neck_roll,neck_yaw,l_shoulder_pitch,l_shoulder_roll,l_shoulder_yaw,l_elbow,r_shoulder_pitch,r_shoulder_roll,r_shoulder_yaw,r_elbow,l_hip_pitch,l_hip_roll,l_hip_yaw,l_knee,l_ankle_pitch,l_ankle_roll,r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)</param>
          <param name="modelFile">model.urdf</param>
          <param name="fixedFrameGravity">(0,0,-9.81)</param>
          <param name="defaultContactFrames">(l_hand,r_hand,root_link,l_sole,r_sole,l_lower_leg,r_lower_leg,l_elbow_1,r_elbow_1)</param>
          <param name="imuFrameName">imu_frame</param>
          <param name="useJointVelocity">true</param>
          <param name="useJointAcceleration">true</param>
          <!-- map between iDynTree links (identified by a name)
               and skinDynLib links (identified by their frame name, a BodyPart enum
               and a local (to the body part) index -->
          <group name="IDYNTREE_SKINDYNLIB_LINKS">
              <param name="root_link">(root_link,1,0)</param>
              <param name="chest"> (chest,1,2)</param>
              <param name="l_upper_arm">(l_upper_arm,3,2)</param>
              <param name="l_elbow_1">(l_elbow_1, 3, 4)</param>
              <param name="r_upper_arm">(r_upper_arm,4,2)</param>
              <param name="r_elbow_1">(r_elbow_1, 4, 4)</param>
              <param name="l_lower_leg">(l_lower_leg,5,3)</param>
              <param name="l_ankle_1">(l_ankle_1,5,4)</param>
              <param name="l_foot">(l_foot_dh_frame,5,5)</param>
              <param name="r_lower_leg">(r_lower_leg,6,3)</param>
              <param name="r_ankle_1">(r_ankle_1,6,4)</param>
              <param name="r_foot">(r_foot_dh_frame,6,5)</param>
          </group>

          <group name="WBD_OUTPUT_EXTERNAL_WRENCH_PORTS">
              <param name="/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o">(l_foot,l_sole,root_link)</param>
              <param name="/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o">(r_foot,r_sole,root_link)</param>
          </group>

          <action phase="startup" level="15" type="attach">
              <paramlist name="networks">
                  <!-- motorcontrol and virtual torque sensors -->
                  <elem name="left_lower_leg">left_lower_leg_mc</elem>
                  <elem name="right_lower_leg">right_lower_leg_mc</elem>
                  <elem name="left_upper_leg">left_upper_leg_mc</elem>
                  <elem name="right_upper_leg">right_upper_leg_mc</elem>
                  <elem name="torso">torso_mc</elem>
                  <elem name="right_lower_arm">right_lower_arm_mc</elem>
                  <elem name="left_lower_arm">left_lower_arm_mc</elem>
                  <elem name="right_upper_arm">right_upper_arm_mc</elem>
                  <elem name="left_upper_arm">left_upper_arm_mc</elem>
                  <elem name="head">head_mc</elem>
                  <!-- imu -->
                  <elem name="imu">inertial</elem>
                  <!-- ft -->
                  <elem name="l_arm_ft">left_upper_arm_strain</elem>
                  <elem name="r_arm_ft">right_upper_arm_strain</elem>
                  <elem name="l_leg_ft">left_upper_leg_strain</elem>
                  <elem name="r_leg_ft">right_upper_leg_strain</elem>
                  <elem name="l_foot_ft">left_lower_leg_strain</elem>
                  <elem name="r_foot_ft">right_lower_leg_strain</elem>
              </paramlist>
          </action>

          <action phase="shutdown" level="2" type="detach" />

      </device>
  </devices>
  ```

### RPC commands
You can access the device while running via `YARP RPC`. You can run the following command to access into the RPC mode of the device.

```sh
yarp rpc <portPrefix>/rpc
```


In particular, you can execute the following commands inside the RPC mode:
- `calib`: Calibrate the force/torque sensors, assuming only external forces acting on the robot are on the **torso/waist**.


```
calib <code> <noOfSamples>
```

Where `<code>` is an argument to specify the sensors to calibrate (all,arms,legs,feet) **AT THE MOMENT IT'S IGNORED**, and `<noOfSamples>` is the number of samples (if not set, it will assume a default value of **100**).

- `calibStanding`: Calibrate the force/torque sensors, assuming only external forces acting on the robot are on the **soles**. It assumes the symmetry of the robot around a vertical central line.

```
calibStanding <code> <noOfSamples>
```

- `calibStandingWithJetsiRonCub`: Works for iRonCub models only, it calibrates the force/torque sensors when on double support and with jet engines turned ON and on idle thrust. It also assumes robot symmetry.

```
calibStandingWithJetsiRonCub <ironcub_model> <code> <noOfSamples>
```

Where `<ironcub_model>` specifies the particular model of iRonCub (mk1, mk1.1).

- `calibStandingLeftFoot`: Calibrate the force/torque sensors when on single support on left foot.

```
calibStandingLeftFoot <code> <noOfSamples>
```

- `calibStandingRightFoot`: Calibrate the force/torque sensors when on single support on right foot.

```
calibStandingRightFoot <code> <noOfSamples>
```

- `calibStandingOnOneLink`: Calibrate the force/torque sensors offsets when the external forces are acting on only one link. This method is typically used when the robot is standing on only one feet, or when it is attached to a fixture that is acting on a single link (typically the chest or the waist).

```
calibStandingOnOneLink <standing_frame> <noOfSamples>
```

Where `standing_frame` Is a frame belonging to the link on which it is assumed that external forces are acting.

- `calibStandingOnTwoLinks`: Calibrate the force/torque sensors offsets when the external forces are acting on only two links.

```
calibStandingOnTwoLinks <first_standing_frame> <second_standing_frame> <noOfSamples>
```
Where `first_standing_frame` and `second_standing_frame` are frames belonging to the two links on which it is assumed that the external forces are acting.

- `resetOffset`: Reset the sensor offset to `0 0 0 0 0 0` (six zeros).

```
resetOffset <code>
```

- `usePreEstimatedOffset`: Use the offline estimated offset of the sensor.

```
usePreEstimatedOffset
```

- `quit`: Quit the module.

```
quit
```

- `resetSimpleLeggedOdometry`: Reset the odometry world to be (initially) a frame specified in the robot model, and specify a link that is assumed to be fixed in the odometry.

```
resetSimpleLeggedOdometry <initial_world_frame> <initial_fixed_link>
```

Where `initial_world_frame` is the frame of the robot model that is assume to be initially coincident with the world/inertial frame, and `initial_fixed_link` is the name of the link that should be initially fixed.

- `changeFixedLinkSimpleLeggedOdometry`: Change the link that is considered fixed by the odometry.

```
changeFixedLinkSimpleLeggedOdometry <new_fixed_link>
```

Where `new_fixed_link` is the name of the new link that should be considered fixed.

-  `set_imuFilterCutoffInHz`: Set the cutoff frequency (in Hz) for IMU measurements.

```
set_imuFilterCutoffInHz <newCutoff>
```

- `get_imuFilterCutoffInHz`: Get the cutoff frequency (in Hz) for IMU measurements.

```
get_imuFilterCutoffInHz
```

- `set_forceTorqueFilterCutoffInHz`: Set the cutoff frequency (in Hz) for FT measurements.

```
set_forceTorqueFilterCutoffInHz <newCutoff>
```

- `get_forceTorqueFilterCutoffInHz`: Get the cutoff frequency (in Hz) for FT measurements.

```
get_forceTorqueFilterCutoffInHz
```

- `set_jointVelFilterCutoffInHz`: Set the cutoff frequency (in Hz) for joint velocities measurements.

```
set_jointVelFilterCutoffInHz <newCutoff>
```

- `get_jointVelFilterCutoffInHz`: Get the cutoff frequency (in Hz) for joint velocities measurements.

```
get_jointVelFilterCutoffInHz
```

- `set_jointAccFilterCutoffInHz`: Set the cutoff frequency (in Hz) for joint acceleration measurements.

```
set_jointAccFilterCutoffInHz <newCutoff>
```

- `get_jointAccFilterCutoffInHz`: Get the cutoff frequency (in Hz) for joint acceleration measurements.

```
get_jointAccFilterCutoffInHz
```

- `useIMUAsKinematicSource`: Use the IMU as the kinematic source of information for the acceleration of one link.

```
useIMUAsKinematicSource
```

- `useFixedFrameAsKinematicSource`: Use a fixed frame (tipically `root_link`, `l_sole` or `r_sole`) as the source of kinematic information. The assumption is that the specified frame will remain fixed until the kinematic source is changing, and the gravity on this link is specified by the fixedFrameGravity (typically set to (0,0,-9.81) .

```
useFixedFrameAsKinematicSource <fixedFrame>
```

- `setUseOfJointVelocities`: Set if to use or not the joint velocities in estimation.

```
setUseOfJointVelocities <enable>
```

Where `enable` is a boolean variable, `true` for enable the using of joint velocities, and `false` for disabling it.

- `setUseOfJointAccelerations`: Set if to use or not the joint accelerations in estimation.

```
setUseOfJointAccelerations <enable>
```

Where `enable` is a boolean variable, `true` for enable the using of joint accelerations, and `false` for disabling it.

- `getCurrentSettingsString`: Get the current settings in the form of a string. Returns the current settings as a human readable string.

```
getCurrentSettingsString
```
