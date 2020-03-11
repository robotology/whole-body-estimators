#floatingBaseEstimator_IDLServer.thrift
namespace yarp codyco

/**
 * Structure representing an homogeneous transform, i.e.
 * a 4x4 matrix with structure:
 *
 * B_H_C = ( xx xy xz x )
 *         ( yx yy yz y )
 *         ( zx zy zz z )
 *         (  0  0  0 1 )
 *
 * A matrix like that respresents the location of a frame C w.r.t. to a frame B.
 *
 * For more information on the semantics of such transformation, please
 * refer to http://repository.tue.nl/849895 .
 */
struct HomTransform
{
   1: double x;     /* x linear position [m] */
   2: double y;     /* y linear position [m] */
   3: double z;     /* z linear position [m] */
   4: double xx;    /* xx element of the rotation matrix */
   5: double xy;    /* xy element of the rotation matrix */
   6: double xz;    /* xz element of the rotation matrix */
   7: double yx;    /* yx element of the rotation matrix */
   8: double yy;    /* yy element of the rotation matrix */
   9: double yz;    /* yz element of the rotation matrix */
  10: double zx;    /* zx element of the rotation matrix */
  11: double zy;    /* zy element of the rotation matrix */
  12: double zz;    /* zz element of the rotation matrix */
}

/**
 * floatingBaseEstimatorRPC
 *
 * Interface.
 */
service floatingBaseEstimatorRPC
{
  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a frame that is assumed to be fixed in the odometry.

   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  bool resetSimpleLeggedOdometry(1:string initial_world_frame, 2:string initial_fixed_frame);

  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a frame that is assumed to be fixed in the odometry.

   * @param initial_reference_frame the frame of the robot model with respect to which we expressed the location of the world.
   * @param initial_reference_frame_H_world the initial location of the world w.r.t. the initial_reference_frame.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed.
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  bool resetSimpleLeggedOdometryToArbitraryFrame(1:string initial_reference_frame, 2:HomTransform initial_reference_frame_H_world, 3:string initial_fixed_frame);

  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame names are wrong)
   */
  bool changeFixedLinkSimpleLeggedOdometry(1:string new_fixed_frame);

  /**
   * Get the current settings in the form of a string.
   * @return the current settings as a human readable string.
   */
  string getCurrentSettingsString();
}




