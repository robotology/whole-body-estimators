#wholeBodyDynamics.thrift

/**
* wholeBodyDynamics_IDLServer
*
* Interface.
*/

service wholeBodyDynamics_IDLServer
{
  /**
  * Calibrate the force/torque sensors
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calib(1:string calib_code, 2:i32 nr_of_samples=100)

  /**
  * Calibrate the force/torque sensors when on double support
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the sole).
  * For this calibration the strong assumption of simmetry of the robot and its pose is done.
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calibStanding(1:string calib_code, 2:i32 nr_of_samples=100)

 /**
  * Calibrate the force/torque sensors when on single support on left foot
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the left sole).
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calibStandingLeftFoot(1:string calib_code, 2:i32 nr_of_samples=100)

  /**
  * Calibrate the force/torque sensors when on single support on right foot
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the right sole).
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calibStandingRightFoot(1:string calib_code, 2:i32 nr_of_samples=100)

  /**
    * Calibrate the force/torque sensors offsets when the external forces are acting on only one link.
    *
    * This method is typically used when the robot is standing on only one feet,
    * or when it is attached to a fixture that is acting on a single link (typically the chest or the waist).
    *
    * @note This method calibrates the offsets of all the force-torque sensors.
    * @param standing_frame a frame belonging to the link on which it is assumed that external forces are acting.
    * @param nr_of_samples number of samples to use for calibration.
    * @return true/false on success/failure.
    */
  bool calibStandingOnOneLink(1:string standing_frame, 2:i32 nr_of_samples=100)

  /**
    * Calibrate the force/torque sensors offsets when the external forces are acting on only two links.
    *
    * This method is not in general guaranteed to work, and it works in practice only when the robot and its internal
    * forces are symmetric w.r.t. the two contact links. Note that the value obtaiend from this calibration depend
    * on the location of the origin of the specific frames of the contact links used for the calibration.
    *
    * @note This method calibrates the offsets of all the force-torque sensors.
    * @param first_standing_frame a frame belonging to one of the two links on which it is assumed that tue external forces are acting.
    * @param second_standing_frame a frame belonging to the other link on which it is assumed that tue external forces are acting.
    * @param nr_of_samples number of samples
    * @return true/false on success/failure
    */
  bool calibStandingOnTwoLinks(1:string first_standing_frame, 2:string second_standing_frame, 3:i32 nr_of_samples=100)

  /**
  * Reset the sensor offset to 0 0 0 0 0 0 (six zeros).
  * @param calib_code argument to specify the sensors to reset (all,arms,legs,feet)
  * @return true/false on success/failure
  */
  bool resetOffset(1:string calib_code)

  /**
   * Use the offline estimated offset of the sensor.
   * Only sensors with a specified offset in configuration file are affected by this method.s
   * @return true/false on success/failure
   */
  bool usePreEstimatedOffset()

  /**
  * Quit the module.
  * @return true/false on success/failure
  */
  bool quit();

  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a link that is assumed to be fixed in the odometry.
   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_link the name of the link that should be initially fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  bool resetSimpleLeggedOdometry(1:string initial_world_frame, 2:string initial_fixed_link)

  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_link the name of the new link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  bool changeFixedLinkSimpleLeggedOdometry(1:string new_fixed_link)

  /**
   * Set the cutoff frequency (in Hz) for IMU measurements
   * @return true/false on success/failure
   */
  bool set_imuFilterCutoffInHz(1:double newCutoff);

  /**
   * Get the cutoff frequency (in Hz) for IMU measurements
   * @return the cutoff frequency (in Hz)
   */
  double get_imuFilterCutoffInHz();

  /**
   * Set the cutoff frequency (in Hz) for FT measurements
   * @return true/false on success/failure
   */
  bool set_forceTorqueFilterCutoffInHz(1:double newCutoff);

  /**
   * Get the cutoff frequency (in Hz) for FT measurements
   * @return the cutoff frequency (in Hz)
   */
  double get_forceTorqueFilterCutoffInHz();

  /**
   * Set the cutoff frequency (in Hz) for joint velocities measurements
   * @return true/false on success/failure
   */
  bool set_jointVelFilterCutoffInHz(1:double newCutoff);

  /**
   * Get the cutoff frequency (in Hz) for joint velocities measurements
   * @return the cutoff frequency (in Hz)
   */
  double get_jointVelFilterCutoffInHz();

  /**
   * Set the cutoff frequency (in Hz) for joint acceleration measurements
   * @return true/false on success/failure
   */
  bool set_jointAccFilterCutoffInHz(1:double newCutoff);

  /**
   * Get the cutoff frequency (in Hz) for joint acceleration measurements
   * @return the cutoff frequency (in Hz)
   */
  double get_jointAccFilterCutoffInHz();

  /**
   * Use the IMU as the kinematic source of
   * information for the acceleration of one link.
   *
   */
  bool useIMUAsKinematicSource();

  /**
   * Use a fixed frame (tipically root_link, l_sole or r_sole)
   * as the source of kinematic information. The assumption
   * is that the specified frame will remain fixed until
   * the kinematic source is changing, and the gravity
   * on this link is specified by the fixedFrameGravity (tipically
   * set to (0,0,-9.81) .
   *
   *
   */
  bool useFixedFrameAsKinematicSource(1:string fixedFrame);

  /**
   * Set if to use or not the joint velocities in estimation.
   *
   */
  bool setUseOfJointVelocities(1:bool enable);

  /**
   * Set if to use or not the joint velocities in estimation.
   *
   */
  bool setUseOfJointAccelerations(1:bool enable);

  /**
   * Get the current settings in the form of a string.
   * @return the current settings as a human readable string.
   */
  string getCurrentSettingsString();
}




