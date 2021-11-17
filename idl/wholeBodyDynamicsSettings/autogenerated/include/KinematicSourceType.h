/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ENUM_KINEMATICSOURCETYPE_H
#define YARP_THRIFT_GENERATOR_ENUM_KINEMATICSOURCETYPE_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

enum KinematicSourceType
{
    IMU = 0,
    FIXED_FRAME = 1
};

class KinematicSourceTypeVocab :
        public yarp::os::idl::WireVocab
{
public:
    int fromString(const std::string& input) override;
    std::string toString(int input) const override;
};

#endif // YARP_THRIFT_GENERATOR_ENUM_KINEMATICSOURCETYPE_H
