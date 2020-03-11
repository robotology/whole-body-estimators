/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <KinematicSourceType.h>



int KinematicSourceTypeVocab::fromString(const std::string& input) {
  // definitely needs optimizing :-)
  if (input=="IMU") return (int)IMU;
  if (input=="FIXED_FRAME") return (int)FIXED_FRAME;
  return -1;
}
std::string KinematicSourceTypeVocab::toString(int input) const {
  switch((KinematicSourceType)input) {
  case IMU:
    return "IMU";
  case FIXED_FRAME:
    return "FIXED_FRAME";
  }
  return "";
}


