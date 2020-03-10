/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_wholeBodyDynamicsSettings
#define YARP_THRIFT_GENERATOR_STRUCT_wholeBodyDynamicsSettings

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <Gravity.h>
#include <KinematicSourceType.h>

class wholeBodyDynamicsSettings;


class wholeBodyDynamicsSettings : public yarp::os::idl::WirePortable {
public:
  // Fields
  KinematicSourceType kinematicSource;
  /**
   * Specify the source of the kinematic information for one link, see KinematicSourceType information for more info.
   */
  std::string fixedFrameName;
  /**
   * If kinematicSource is FIXED_LINK, specify the frame of the robot that we know to be fixed (i.e. not moving with respect to an inertial frame)
   */
  Gravity fixedFrameGravity;
  /**
   * If kinematicSource is FIXED_LINK, specify the gravity vector (in m/s^2) in the fixedFrame
   */
  std::string imuFrameName;
  /**
   * If kinematicSource is IMU, specify the frame name of the imu
   */
  double imuFilterCutoffInHz;
  /**
   * Cutoff frequency (in Hz) of the first order filter of the IMU
   */
  double forceTorqueFilterCutoffInHz;
  /**
   * Cutoff frequency(in Hz) of the first order filter of the F/T sensors
   */
  double jointVelFilterCutoffInHz;
  /**
   * Cutoff frequency(in Hz) of the first order filter of the joint velocities
   */
  double jointAccFilterCutoffInHz;
  /**
   * Cutoff frequency(in Hz) of the first order filter of the joint accelerations
   */
  bool useJointVelocity;
  /**
   * Use the joint velocity measurement if this is true, assume they are zero otherwise.
   */
  bool useJointAcceleration;

  // Default constructor
  wholeBodyDynamicsSettings() : kinematicSource((KinematicSourceType)0), fixedFrameName(""), imuFrameName(""), imuFilterCutoffInHz(0), forceTorqueFilterCutoffInHz(0), jointVelFilterCutoffInHz(0), jointAccFilterCutoffInHz(0), useJointVelocity(0), useJointAcceleration(0) {
  }

  // Constructor with field values
  wholeBodyDynamicsSettings(const KinematicSourceType kinematicSource,const std::string& fixedFrameName,const Gravity& fixedFrameGravity,const std::string& imuFrameName,const double imuFilterCutoffInHz,const double forceTorqueFilterCutoffInHz,const double jointVelFilterCutoffInHz,const double jointAccFilterCutoffInHz,const bool useJointVelocity,const bool useJointAcceleration) : kinematicSource(kinematicSource), fixedFrameName(fixedFrameName), fixedFrameGravity(fixedFrameGravity), imuFrameName(imuFrameName), imuFilterCutoffInHz(imuFilterCutoffInHz), forceTorqueFilterCutoffInHz(forceTorqueFilterCutoffInHz), jointVelFilterCutoffInHz(jointVelFilterCutoffInHz), jointAccFilterCutoffInHz(jointAccFilterCutoffInHz), useJointVelocity(useJointVelocity), useJointAcceleration(useJointAcceleration) {
  }

  // Copy constructor
  wholeBodyDynamicsSettings(const wholeBodyDynamicsSettings& __alt) : WirePortable(__alt)  {
    this->kinematicSource = __alt.kinematicSource;
    this->fixedFrameName = __alt.fixedFrameName;
    this->fixedFrameGravity = __alt.fixedFrameGravity;
    this->imuFrameName = __alt.imuFrameName;
    this->imuFilterCutoffInHz = __alt.imuFilterCutoffInHz;
    this->forceTorqueFilterCutoffInHz = __alt.forceTorqueFilterCutoffInHz;
    this->jointVelFilterCutoffInHz = __alt.jointVelFilterCutoffInHz;
    this->jointAccFilterCutoffInHz = __alt.jointAccFilterCutoffInHz;
    this->useJointVelocity = __alt.useJointVelocity;
    this->useJointAcceleration = __alt.useJointAcceleration;
  }

  // Assignment operator
  const wholeBodyDynamicsSettings& operator = (const wholeBodyDynamicsSettings& __alt) {
    this->kinematicSource = __alt.kinematicSource;
    this->fixedFrameName = __alt.fixedFrameName;
    this->fixedFrameGravity = __alt.fixedFrameGravity;
    this->imuFrameName = __alt.imuFrameName;
    this->imuFilterCutoffInHz = __alt.imuFilterCutoffInHz;
    this->forceTorqueFilterCutoffInHz = __alt.forceTorqueFilterCutoffInHz;
    this->jointVelFilterCutoffInHz = __alt.jointVelFilterCutoffInHz;
    this->jointAccFilterCutoffInHz = __alt.jointAccFilterCutoffInHz;
    this->useJointVelocity = __alt.useJointVelocity;
    this->useJointAcceleration = __alt.useJointAcceleration;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader) override;
  bool read(yarp::os::ConnectionReader& connection) override;
  bool write(const yarp::os::idl::WireWriter& writer) const override;
  bool write(yarp::os::ConnectionWriter& connection) const override;

private:
  bool write_kinematicSource(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_kinematicSource(const yarp::os::idl::WireWriter& writer) const;
  bool write_fixedFrameName(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_fixedFrameName(const yarp::os::idl::WireWriter& writer) const;
  bool write_fixedFrameGravity(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_fixedFrameGravity(const yarp::os::idl::WireWriter& writer) const;
  bool write_imuFrameName(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_imuFrameName(const yarp::os::idl::WireWriter& writer) const;
  bool write_imuFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_imuFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool write_forceTorqueFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_forceTorqueFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool write_jointVelFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_jointVelFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool write_jointAccFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_jointAccFilterCutoffInHz(const yarp::os::idl::WireWriter& writer) const;
  bool write_useJointVelocity(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_useJointVelocity(const yarp::os::idl::WireWriter& writer) const;
  bool write_useJointAcceleration(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_useJointAcceleration(const yarp::os::idl::WireWriter& writer) const;
  bool read_kinematicSource(yarp::os::idl::WireReader& reader);
  bool nested_read_kinematicSource(yarp::os::idl::WireReader& reader);
  bool read_fixedFrameName(yarp::os::idl::WireReader& reader);
  bool nested_read_fixedFrameName(yarp::os::idl::WireReader& reader);
  bool read_fixedFrameGravity(yarp::os::idl::WireReader& reader);
  bool nested_read_fixedFrameGravity(yarp::os::idl::WireReader& reader);
  bool read_imuFrameName(yarp::os::idl::WireReader& reader);
  bool nested_read_imuFrameName(yarp::os::idl::WireReader& reader);
  bool read_imuFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool nested_read_imuFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool read_forceTorqueFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool nested_read_forceTorqueFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool read_jointVelFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool nested_read_jointVelFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool read_jointAccFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool nested_read_jointAccFilterCutoffInHz(yarp::os::idl::WireReader& reader);
  bool read_useJointVelocity(yarp::os::idl::WireReader& reader);
  bool nested_read_useJointVelocity(yarp::os::idl::WireReader& reader);
  bool read_useJointAcceleration(yarp::os::idl::WireReader& reader);
  bool nested_read_useJointAcceleration(yarp::os::idl::WireReader& reader);

public:

  std::string toString() const;

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<wholeBodyDynamicsSettings > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new wholeBodyDynamicsSettings;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(wholeBodyDynamicsSettings& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(wholeBodyDynamicsSettings& obj, bool dirty = true) {
      if (obj_owned) delete this->obj;
      this->obj = &obj;
      obj_owned = false;
      dirty_flags(dirty);
      return true;
    }

    virtual ~Editor() {
    if (obj_owned) delete obj;
    }

    bool isValid() const {
      return obj!=0/*NULL*/;
    }

    wholeBodyDynamicsSettings& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_kinematicSource(const KinematicSourceType kinematicSource) {
      will_set_kinematicSource();
      obj->kinematicSource = kinematicSource;
      mark_dirty_kinematicSource();
      communicate();
      did_set_kinematicSource();
    }
    void set_fixedFrameName(const std::string& fixedFrameName) {
      will_set_fixedFrameName();
      obj->fixedFrameName = fixedFrameName;
      mark_dirty_fixedFrameName();
      communicate();
      did_set_fixedFrameName();
    }
    void set_fixedFrameGravity(const Gravity& fixedFrameGravity) {
      will_set_fixedFrameGravity();
      obj->fixedFrameGravity = fixedFrameGravity;
      mark_dirty_fixedFrameGravity();
      communicate();
      did_set_fixedFrameGravity();
    }
    void set_imuFrameName(const std::string& imuFrameName) {
      will_set_imuFrameName();
      obj->imuFrameName = imuFrameName;
      mark_dirty_imuFrameName();
      communicate();
      did_set_imuFrameName();
    }
    void set_imuFilterCutoffInHz(const double imuFilterCutoffInHz) {
      will_set_imuFilterCutoffInHz();
      obj->imuFilterCutoffInHz = imuFilterCutoffInHz;
      mark_dirty_imuFilterCutoffInHz();
      communicate();
      did_set_imuFilterCutoffInHz();
    }
    void set_forceTorqueFilterCutoffInHz(const double forceTorqueFilterCutoffInHz) {
      will_set_forceTorqueFilterCutoffInHz();
      obj->forceTorqueFilterCutoffInHz = forceTorqueFilterCutoffInHz;
      mark_dirty_forceTorqueFilterCutoffInHz();
      communicate();
      did_set_forceTorqueFilterCutoffInHz();
    }
    void set_jointVelFilterCutoffInHz(const double jointVelFilterCutoffInHz) {
      will_set_jointVelFilterCutoffInHz();
      obj->jointVelFilterCutoffInHz = jointVelFilterCutoffInHz;
      mark_dirty_jointVelFilterCutoffInHz();
      communicate();
      did_set_jointVelFilterCutoffInHz();
    }
    void set_jointAccFilterCutoffInHz(const double jointAccFilterCutoffInHz) {
      will_set_jointAccFilterCutoffInHz();
      obj->jointAccFilterCutoffInHz = jointAccFilterCutoffInHz;
      mark_dirty_jointAccFilterCutoffInHz();
      communicate();
      did_set_jointAccFilterCutoffInHz();
    }
    void set_useJointVelocity(const bool useJointVelocity) {
      will_set_useJointVelocity();
      obj->useJointVelocity = useJointVelocity;
      mark_dirty_useJointVelocity();
      communicate();
      did_set_useJointVelocity();
    }
    void set_useJointAcceleration(const bool useJointAcceleration) {
      will_set_useJointAcceleration();
      obj->useJointAcceleration = useJointAcceleration;
      mark_dirty_useJointAcceleration();
      communicate();
      did_set_useJointAcceleration();
    }
    const KinematicSourceType get_kinematicSource() {
      return obj->kinematicSource;
    }
    const std::string& get_fixedFrameName() {
      return obj->fixedFrameName;
    }
    const Gravity& get_fixedFrameGravity() {
      return obj->fixedFrameGravity;
    }
    const std::string& get_imuFrameName() {
      return obj->imuFrameName;
    }
    double get_imuFilterCutoffInHz() {
      return obj->imuFilterCutoffInHz;
    }
    double get_forceTorqueFilterCutoffInHz() {
      return obj->forceTorqueFilterCutoffInHz;
    }
    double get_jointVelFilterCutoffInHz() {
      return obj->jointVelFilterCutoffInHz;
    }
    double get_jointAccFilterCutoffInHz() {
      return obj->jointAccFilterCutoffInHz;
    }
    bool get_useJointVelocity() {
      return obj->useJointVelocity;
    }
    bool get_useJointAcceleration() {
      return obj->useJointAcceleration;
    }
    virtual bool will_set_kinematicSource() { return true; }
    virtual bool will_set_fixedFrameName() { return true; }
    virtual bool will_set_fixedFrameGravity() { return true; }
    virtual bool will_set_imuFrameName() { return true; }
    virtual bool will_set_imuFilterCutoffInHz() { return true; }
    virtual bool will_set_forceTorqueFilterCutoffInHz() { return true; }
    virtual bool will_set_jointVelFilterCutoffInHz() { return true; }
    virtual bool will_set_jointAccFilterCutoffInHz() { return true; }
    virtual bool will_set_useJointVelocity() { return true; }
    virtual bool will_set_useJointAcceleration() { return true; }
    virtual bool did_set_kinematicSource() { return true; }
    virtual bool did_set_fixedFrameName() { return true; }
    virtual bool did_set_fixedFrameGravity() { return true; }
    virtual bool did_set_imuFrameName() { return true; }
    virtual bool did_set_imuFilterCutoffInHz() { return true; }
    virtual bool did_set_forceTorqueFilterCutoffInHz() { return true; }
    virtual bool did_set_jointVelFilterCutoffInHz() { return true; }
    virtual bool did_set_jointAccFilterCutoffInHz() { return true; }
    virtual bool did_set_useJointVelocity() { return true; }
    virtual bool did_set_useJointAcceleration() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection) override;
    bool write(yarp::os::ConnectionWriter& connection) const override;
  private:

    wholeBodyDynamicsSettings *obj;

    bool obj_owned;
    int group;

    void communicate() {
      if (group!=0) return;
      if (yarp().canWrite()) {
        yarp().write(*this);
        clean();
      }
    }
    void mark_dirty() {
      is_dirty = true;
    }
    void mark_dirty_kinematicSource() {
      if (is_dirty_kinematicSource) return;
      dirty_count++;
      is_dirty_kinematicSource = true;
      mark_dirty();
    }
    void mark_dirty_fixedFrameName() {
      if (is_dirty_fixedFrameName) return;
      dirty_count++;
      is_dirty_fixedFrameName = true;
      mark_dirty();
    }
    void mark_dirty_fixedFrameGravity() {
      if (is_dirty_fixedFrameGravity) return;
      dirty_count++;
      is_dirty_fixedFrameGravity = true;
      mark_dirty();
    }
    void mark_dirty_imuFrameName() {
      if (is_dirty_imuFrameName) return;
      dirty_count++;
      is_dirty_imuFrameName = true;
      mark_dirty();
    }
    void mark_dirty_imuFilterCutoffInHz() {
      if (is_dirty_imuFilterCutoffInHz) return;
      dirty_count++;
      is_dirty_imuFilterCutoffInHz = true;
      mark_dirty();
    }
    void mark_dirty_forceTorqueFilterCutoffInHz() {
      if (is_dirty_forceTorqueFilterCutoffInHz) return;
      dirty_count++;
      is_dirty_forceTorqueFilterCutoffInHz = true;
      mark_dirty();
    }
    void mark_dirty_jointVelFilterCutoffInHz() {
      if (is_dirty_jointVelFilterCutoffInHz) return;
      dirty_count++;
      is_dirty_jointVelFilterCutoffInHz = true;
      mark_dirty();
    }
    void mark_dirty_jointAccFilterCutoffInHz() {
      if (is_dirty_jointAccFilterCutoffInHz) return;
      dirty_count++;
      is_dirty_jointAccFilterCutoffInHz = true;
      mark_dirty();
    }
    void mark_dirty_useJointVelocity() {
      if (is_dirty_useJointVelocity) return;
      dirty_count++;
      is_dirty_useJointVelocity = true;
      mark_dirty();
    }
    void mark_dirty_useJointAcceleration() {
      if (is_dirty_useJointAcceleration) return;
      dirty_count++;
      is_dirty_useJointAcceleration = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_kinematicSource = flag;
      is_dirty_fixedFrameName = flag;
      is_dirty_fixedFrameGravity = flag;
      is_dirty_imuFrameName = flag;
      is_dirty_imuFilterCutoffInHz = flag;
      is_dirty_forceTorqueFilterCutoffInHz = flag;
      is_dirty_jointVelFilterCutoffInHz = flag;
      is_dirty_jointAccFilterCutoffInHz = flag;
      is_dirty_useJointVelocity = flag;
      is_dirty_useJointAcceleration = flag;
      dirty_count = flag ? 10 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_kinematicSource;
    bool is_dirty_fixedFrameName;
    bool is_dirty_fixedFrameGravity;
    bool is_dirty_imuFrameName;
    bool is_dirty_imuFilterCutoffInHz;
    bool is_dirty_forceTorqueFilterCutoffInHz;
    bool is_dirty_jointVelFilterCutoffInHz;
    bool is_dirty_jointAccFilterCutoffInHz;
    bool is_dirty_useJointVelocity;
    bool is_dirty_useJointAcceleration;
  };
};

#endif
