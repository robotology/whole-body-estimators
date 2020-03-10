/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <codyco/floatingBaseEstimatorRPC.h>
#include <yarp/os/idl/WireTypes.h>

namespace codyco {


class floatingBaseEstimatorRPC_resetSimpleLeggedOdometry : public yarp::os::Portable {
public:
  std::string initial_world_frame;
  std::string initial_fixed_frame;
  bool _return;
  void init(const std::string& initial_world_frame, const std::string& initial_fixed_frame);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class floatingBaseEstimatorRPC_resetSimpleLeggedOdometryToArbitraryFrame : public yarp::os::Portable {
public:
  std::string initial_reference_frame;
  HomTransform initial_reference_frame_H_world;
  std::string initial_fixed_frame;
  bool _return;
  void init(const std::string& initial_reference_frame, const HomTransform& initial_reference_frame_H_world, const std::string& initial_fixed_frame);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class floatingBaseEstimatorRPC_changeFixedLinkSimpleLeggedOdometry : public yarp::os::Portable {
public:
  std::string new_fixed_frame;
  bool _return;
  void init(const std::string& new_fixed_frame);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class floatingBaseEstimatorRPC_getCurrentSettingsString : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

bool floatingBaseEstimatorRPC_resetSimpleLeggedOdometry::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("resetSimpleLeggedOdometry",1,1)) return false;
  if (!writer.writeString(initial_world_frame)) return false;
  if (!writer.writeString(initial_fixed_frame)) return false;
  return true;
}

bool floatingBaseEstimatorRPC_resetSimpleLeggedOdometry::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void floatingBaseEstimatorRPC_resetSimpleLeggedOdometry::init(const std::string& initial_world_frame, const std::string& initial_fixed_frame) {
  _return = false;
  this->initial_world_frame = initial_world_frame;
  this->initial_fixed_frame = initial_fixed_frame;
}

bool floatingBaseEstimatorRPC_resetSimpleLeggedOdometryToArbitraryFrame::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(15)) return false;
  if (!writer.writeTag("resetSimpleLeggedOdometryToArbitraryFrame",1,1)) return false;
  if (!writer.writeString(initial_reference_frame)) return false;
  if (!writer.write(initial_reference_frame_H_world)) return false;
  if (!writer.writeString(initial_fixed_frame)) return false;
  return true;
}

bool floatingBaseEstimatorRPC_resetSimpleLeggedOdometryToArbitraryFrame::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void floatingBaseEstimatorRPC_resetSimpleLeggedOdometryToArbitraryFrame::init(const std::string& initial_reference_frame, const HomTransform& initial_reference_frame_H_world, const std::string& initial_fixed_frame) {
  _return = false;
  this->initial_reference_frame = initial_reference_frame;
  this->initial_reference_frame_H_world = initial_reference_frame_H_world;
  this->initial_fixed_frame = initial_fixed_frame;
}

bool floatingBaseEstimatorRPC_changeFixedLinkSimpleLeggedOdometry::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("changeFixedLinkSimpleLeggedOdometry",1,1)) return false;
  if (!writer.writeString(new_fixed_frame)) return false;
  return true;
}

bool floatingBaseEstimatorRPC_changeFixedLinkSimpleLeggedOdometry::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void floatingBaseEstimatorRPC_changeFixedLinkSimpleLeggedOdometry::init(const std::string& new_fixed_frame) {
  _return = false;
  this->new_fixed_frame = new_fixed_frame;
}

bool floatingBaseEstimatorRPC_getCurrentSettingsString::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getCurrentSettingsString",1,1)) return false;
  return true;
}

bool floatingBaseEstimatorRPC_getCurrentSettingsString::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void floatingBaseEstimatorRPC_getCurrentSettingsString::init() {
  _return = "";
}

floatingBaseEstimatorRPC::floatingBaseEstimatorRPC() {
  yarp().setOwner(*this);
}
bool floatingBaseEstimatorRPC::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_frame) {
  bool _return = false;
  floatingBaseEstimatorRPC_resetSimpleLeggedOdometry helper;
  helper.init(initial_world_frame,initial_fixed_frame);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool floatingBaseEstimatorRPC::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_frame)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool floatingBaseEstimatorRPC::resetSimpleLeggedOdometryToArbitraryFrame(const std::string& initial_reference_frame, const HomTransform& initial_reference_frame_H_world, const std::string& initial_fixed_frame) {
  bool _return = false;
  floatingBaseEstimatorRPC_resetSimpleLeggedOdometryToArbitraryFrame helper;
  helper.init(initial_reference_frame,initial_reference_frame_H_world,initial_fixed_frame);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool floatingBaseEstimatorRPC::resetSimpleLeggedOdometryToArbitraryFrame(const std::string& initial_reference_frame, const HomTransform& initial_reference_frame_H_world, const std::string& initial_fixed_frame)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool floatingBaseEstimatorRPC::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_frame) {
  bool _return = false;
  floatingBaseEstimatorRPC_changeFixedLinkSimpleLeggedOdometry helper;
  helper.init(new_fixed_frame);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool floatingBaseEstimatorRPC::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_frame)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string floatingBaseEstimatorRPC::getCurrentSettingsString() {
  std::string _return = "";
  floatingBaseEstimatorRPC_getCurrentSettingsString helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string floatingBaseEstimatorRPC::getCurrentSettingsString()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool floatingBaseEstimatorRPC::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  std::string tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "resetSimpleLeggedOdometry") {
      std::string initial_world_frame;
      std::string initial_fixed_frame;
      if (!reader.readString(initial_world_frame)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(initial_fixed_frame)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = resetSimpleLeggedOdometry(initial_world_frame,initial_fixed_frame);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "resetSimpleLeggedOdometryToArbitraryFrame") {
      std::string initial_reference_frame;
      HomTransform initial_reference_frame_H_world;
      std::string initial_fixed_frame;
      if (!reader.readString(initial_reference_frame)) {
        reader.fail();
        return false;
      }
      if (!reader.read(initial_reference_frame_H_world)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(initial_fixed_frame)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = resetSimpleLeggedOdometryToArbitraryFrame(initial_reference_frame,initial_reference_frame_H_world,initial_fixed_frame);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "changeFixedLinkSimpleLeggedOdometry") {
      std::string new_fixed_frame;
      if (!reader.readString(new_fixed_frame)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = changeFixedLinkSimpleLeggedOdometry(new_fixed_frame);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getCurrentSettingsString") {
      std::string _return;
      _return = getCurrentSettingsString();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT32, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    std::string next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> floatingBaseEstimatorRPC::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("resetSimpleLeggedOdometry");
    helpString.push_back("resetSimpleLeggedOdometryToArbitraryFrame");
    helpString.push_back("changeFixedLinkSimpleLeggedOdometry");
    helpString.push_back("getCurrentSettingsString");
    helpString.push_back("help");
  }
  else {
    if (functionName=="resetSimpleLeggedOdometry") {
      helpString.push_back("bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_frame) ");
      helpString.push_back("Reset the odometry world to be (initially) a frame specified in the robot model, ");
      helpString.push_back("and specify a frame that is assumed to be fixed in the odometry. ");
      helpString.push_back("@param initial_world_frame the frame of the robot model that is assume to be initially ");
      helpString.push_back("       coincident with the world/inertial frame. ");
      helpString.push_back("@param new_fixed_frame the name of a frame attached to the link that should be considered fixed ");
      helpString.push_back("@return true/false on success/failure (typically if the frame/link names are wrong) ");
    }
    if (functionName=="resetSimpleLeggedOdometryToArbitraryFrame") {
      helpString.push_back("bool resetSimpleLeggedOdometryToArbitraryFrame(const std::string& initial_reference_frame, const HomTransform& initial_reference_frame_H_world, const std::string& initial_fixed_frame) ");
      helpString.push_back("Reset the odometry world to be (initially) a frame specified in the robot model, ");
      helpString.push_back("and specify a frame that is assumed to be fixed in the odometry. ");
      helpString.push_back("@param initial_reference_frame the frame of the robot model with respect to which we expressed the location of the world. ");
      helpString.push_back("@param initial_reference_frame_H_world the initial location of the world w.r.t. the initial_reference_frame. ");
      helpString.push_back("@param new_fixed_frame the name of a frame attached to the link that should be considered fixed. ");
      helpString.push_back("@return true/false on success/failure (typically if the frame/link names are wrong) ");
    }
    if (functionName=="changeFixedLinkSimpleLeggedOdometry") {
      helpString.push_back("bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_frame) ");
      helpString.push_back("Change the link that is considered fixed by the odometry. ");
      helpString.push_back("@param new_fixed_frame the name of a frame attached to the link that should be considered fixed ");
      helpString.push_back("@return true/false on success/failure (typically if the frame names are wrong) ");
    }
    if (functionName=="getCurrentSettingsString") {
      helpString.push_back("std::string getCurrentSettingsString() ");
      helpString.push_back("Get the current settings in the form of a string. ");
      helpString.push_back("@return the current settings as a human readable string. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}
} // namespace


