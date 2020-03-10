/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_floatingBaseEstimatorRPC
#define YARP_THRIFT_GENERATOR_floatingBaseEstimatorRPC

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <codyco/HomTransform.h>

namespace codyco {
  class floatingBaseEstimatorRPC;
}


/**
 * floatingBaseEstimatorRPC
 * Interface.
 */
class codyco::floatingBaseEstimatorRPC : public yarp::os::Wire {
public:
  floatingBaseEstimatorRPC();
  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a frame that is assumed to be fixed in the odometry.
   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_frame);
  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a frame that is assumed to be fixed in the odometry.
   * @param initial_reference_frame the frame of the robot model with respect to which we expressed the location of the world.
   * @param initial_reference_frame_H_world the initial location of the world w.r.t. the initial_reference_frame.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed.
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  virtual bool resetSimpleLeggedOdometryToArbitraryFrame(const std::string& initial_reference_frame, const HomTransform& initial_reference_frame_H_world, const std::string& initial_fixed_frame);
  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame names are wrong)
   */
  virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_frame);
  /**
   * Get the current settings in the form of a string.
   * @return the current settings as a human readable string.
   */
  virtual std::string getCurrentSettingsString();
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
