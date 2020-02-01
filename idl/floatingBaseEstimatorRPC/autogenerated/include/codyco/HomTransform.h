/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_HomTransform
#define YARP_THRIFT_GENERATOR_STRUCT_HomTransform

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace codyco {
  class HomTransform;
}


/**
 * Structure representing an homogeneous transform, i.e.
 * a 4x4 matrix with structure:
 * B_H_C = ( xx xy xz x )
 *         ( yx yy yz y )
 *         ( zx zy zz z )
 *         (  0  0  0 1 )
 * A matrix like that respresents the location of a frame C w.r.t. to a frame B.
 * For more information on the semantics of such transformation, please
 * refer to http://repository.tue.nl/849895 .
 */
class codyco::HomTransform : public yarp::os::idl::WirePortable {
public:
  // Fields
  double x;
  double y;
  double z;
  double xx;
  double xy;
  double xz;
  double yx;
  double yy;
  double yz;
  double zx;
  double zy;
  double zz;

  // Default constructor
  HomTransform() : x(0), y(0), z(0), xx(0), xy(0), xz(0), yx(0), yy(0), yz(0), zx(0), zy(0), zz(0) {
  }

  // Constructor with field values
  HomTransform(const double x,const double y,const double z,const double xx,const double xy,const double xz,const double yx,const double yy,const double yz,const double zx,const double zy,const double zz) : x(x), y(y), z(z), xx(xx), xy(xy), xz(xz), yx(yx), yy(yy), yz(yz), zx(zx), zy(zy), zz(zz) {
  }

  // Copy constructor
  HomTransform(const HomTransform& __alt) : WirePortable(__alt)  {
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
    this->xx = __alt.xx;
    this->xy = __alt.xy;
    this->xz = __alt.xz;
    this->yx = __alt.yx;
    this->yy = __alt.yy;
    this->yz = __alt.yz;
    this->zx = __alt.zx;
    this->zy = __alt.zy;
    this->zz = __alt.zz;
  }

  // Assignment operator
  const HomTransform& operator = (const HomTransform& __alt) {
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
    this->xx = __alt.xx;
    this->xy = __alt.xy;
    this->xz = __alt.xz;
    this->yx = __alt.yx;
    this->yy = __alt.yy;
    this->yz = __alt.yz;
    this->zx = __alt.zx;
    this->zy = __alt.zy;
    this->zz = __alt.zz;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader) override;
  bool read(yarp::os::ConnectionReader& connection) override;
  bool write(const yarp::os::idl::WireWriter& writer) const override;
  bool write(yarp::os::ConnectionWriter& connection) const override;

private:
  bool write_x(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_x(const yarp::os::idl::WireWriter& writer) const;
  bool write_y(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_y(const yarp::os::idl::WireWriter& writer) const;
  bool write_z(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_z(const yarp::os::idl::WireWriter& writer) const;
  bool write_xx(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_xx(const yarp::os::idl::WireWriter& writer) const;
  bool write_xy(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_xy(const yarp::os::idl::WireWriter& writer) const;
  bool write_xz(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_xz(const yarp::os::idl::WireWriter& writer) const;
  bool write_yx(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_yx(const yarp::os::idl::WireWriter& writer) const;
  bool write_yy(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_yy(const yarp::os::idl::WireWriter& writer) const;
  bool write_yz(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_yz(const yarp::os::idl::WireWriter& writer) const;
  bool write_zx(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_zx(const yarp::os::idl::WireWriter& writer) const;
  bool write_zy(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_zy(const yarp::os::idl::WireWriter& writer) const;
  bool write_zz(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_zz(const yarp::os::idl::WireWriter& writer) const;
  bool read_x(yarp::os::idl::WireReader& reader);
  bool nested_read_x(yarp::os::idl::WireReader& reader);
  bool read_y(yarp::os::idl::WireReader& reader);
  bool nested_read_y(yarp::os::idl::WireReader& reader);
  bool read_z(yarp::os::idl::WireReader& reader);
  bool nested_read_z(yarp::os::idl::WireReader& reader);
  bool read_xx(yarp::os::idl::WireReader& reader);
  bool nested_read_xx(yarp::os::idl::WireReader& reader);
  bool read_xy(yarp::os::idl::WireReader& reader);
  bool nested_read_xy(yarp::os::idl::WireReader& reader);
  bool read_xz(yarp::os::idl::WireReader& reader);
  bool nested_read_xz(yarp::os::idl::WireReader& reader);
  bool read_yx(yarp::os::idl::WireReader& reader);
  bool nested_read_yx(yarp::os::idl::WireReader& reader);
  bool read_yy(yarp::os::idl::WireReader& reader);
  bool nested_read_yy(yarp::os::idl::WireReader& reader);
  bool read_yz(yarp::os::idl::WireReader& reader);
  bool nested_read_yz(yarp::os::idl::WireReader& reader);
  bool read_zx(yarp::os::idl::WireReader& reader);
  bool nested_read_zx(yarp::os::idl::WireReader& reader);
  bool read_zy(yarp::os::idl::WireReader& reader);
  bool nested_read_zy(yarp::os::idl::WireReader& reader);
  bool read_zz(yarp::os::idl::WireReader& reader);
  bool nested_read_zz(yarp::os::idl::WireReader& reader);

public:

  std::string toString() const;

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<codyco::HomTransform > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new HomTransform;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(HomTransform& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(HomTransform& obj, bool dirty = true) {
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

    HomTransform& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_x(const double x) {
      will_set_x();
      obj->x = x;
      mark_dirty_x();
      communicate();
      did_set_x();
    }
    void set_y(const double y) {
      will_set_y();
      obj->y = y;
      mark_dirty_y();
      communicate();
      did_set_y();
    }
    void set_z(const double z) {
      will_set_z();
      obj->z = z;
      mark_dirty_z();
      communicate();
      did_set_z();
    }
    void set_xx(const double xx) {
      will_set_xx();
      obj->xx = xx;
      mark_dirty_xx();
      communicate();
      did_set_xx();
    }
    void set_xy(const double xy) {
      will_set_xy();
      obj->xy = xy;
      mark_dirty_xy();
      communicate();
      did_set_xy();
    }
    void set_xz(const double xz) {
      will_set_xz();
      obj->xz = xz;
      mark_dirty_xz();
      communicate();
      did_set_xz();
    }
    void set_yx(const double yx) {
      will_set_yx();
      obj->yx = yx;
      mark_dirty_yx();
      communicate();
      did_set_yx();
    }
    void set_yy(const double yy) {
      will_set_yy();
      obj->yy = yy;
      mark_dirty_yy();
      communicate();
      did_set_yy();
    }
    void set_yz(const double yz) {
      will_set_yz();
      obj->yz = yz;
      mark_dirty_yz();
      communicate();
      did_set_yz();
    }
    void set_zx(const double zx) {
      will_set_zx();
      obj->zx = zx;
      mark_dirty_zx();
      communicate();
      did_set_zx();
    }
    void set_zy(const double zy) {
      will_set_zy();
      obj->zy = zy;
      mark_dirty_zy();
      communicate();
      did_set_zy();
    }
    void set_zz(const double zz) {
      will_set_zz();
      obj->zz = zz;
      mark_dirty_zz();
      communicate();
      did_set_zz();
    }
    double get_x() {
      return obj->x;
    }
    double get_y() {
      return obj->y;
    }
    double get_z() {
      return obj->z;
    }
    double get_xx() {
      return obj->xx;
    }
    double get_xy() {
      return obj->xy;
    }
    double get_xz() {
      return obj->xz;
    }
    double get_yx() {
      return obj->yx;
    }
    double get_yy() {
      return obj->yy;
    }
    double get_yz() {
      return obj->yz;
    }
    double get_zx() {
      return obj->zx;
    }
    double get_zy() {
      return obj->zy;
    }
    double get_zz() {
      return obj->zz;
    }
    virtual bool will_set_x() { return true; }
    virtual bool will_set_y() { return true; }
    virtual bool will_set_z() { return true; }
    virtual bool will_set_xx() { return true; }
    virtual bool will_set_xy() { return true; }
    virtual bool will_set_xz() { return true; }
    virtual bool will_set_yx() { return true; }
    virtual bool will_set_yy() { return true; }
    virtual bool will_set_yz() { return true; }
    virtual bool will_set_zx() { return true; }
    virtual bool will_set_zy() { return true; }
    virtual bool will_set_zz() { return true; }
    virtual bool did_set_x() { return true; }
    virtual bool did_set_y() { return true; }
    virtual bool did_set_z() { return true; }
    virtual bool did_set_xx() { return true; }
    virtual bool did_set_xy() { return true; }
    virtual bool did_set_xz() { return true; }
    virtual bool did_set_yx() { return true; }
    virtual bool did_set_yy() { return true; }
    virtual bool did_set_yz() { return true; }
    virtual bool did_set_zx() { return true; }
    virtual bool did_set_zy() { return true; }
    virtual bool did_set_zz() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection) override;
    bool write(yarp::os::ConnectionWriter& connection) const override;
  private:

    HomTransform *obj;

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
    void mark_dirty_x() {
      if (is_dirty_x) return;
      dirty_count++;
      is_dirty_x = true;
      mark_dirty();
    }
    void mark_dirty_y() {
      if (is_dirty_y) return;
      dirty_count++;
      is_dirty_y = true;
      mark_dirty();
    }
    void mark_dirty_z() {
      if (is_dirty_z) return;
      dirty_count++;
      is_dirty_z = true;
      mark_dirty();
    }
    void mark_dirty_xx() {
      if (is_dirty_xx) return;
      dirty_count++;
      is_dirty_xx = true;
      mark_dirty();
    }
    void mark_dirty_xy() {
      if (is_dirty_xy) return;
      dirty_count++;
      is_dirty_xy = true;
      mark_dirty();
    }
    void mark_dirty_xz() {
      if (is_dirty_xz) return;
      dirty_count++;
      is_dirty_xz = true;
      mark_dirty();
    }
    void mark_dirty_yx() {
      if (is_dirty_yx) return;
      dirty_count++;
      is_dirty_yx = true;
      mark_dirty();
    }
    void mark_dirty_yy() {
      if (is_dirty_yy) return;
      dirty_count++;
      is_dirty_yy = true;
      mark_dirty();
    }
    void mark_dirty_yz() {
      if (is_dirty_yz) return;
      dirty_count++;
      is_dirty_yz = true;
      mark_dirty();
    }
    void mark_dirty_zx() {
      if (is_dirty_zx) return;
      dirty_count++;
      is_dirty_zx = true;
      mark_dirty();
    }
    void mark_dirty_zy() {
      if (is_dirty_zy) return;
      dirty_count++;
      is_dirty_zy = true;
      mark_dirty();
    }
    void mark_dirty_zz() {
      if (is_dirty_zz) return;
      dirty_count++;
      is_dirty_zz = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_x = flag;
      is_dirty_y = flag;
      is_dirty_z = flag;
      is_dirty_xx = flag;
      is_dirty_xy = flag;
      is_dirty_xz = flag;
      is_dirty_yx = flag;
      is_dirty_yy = flag;
      is_dirty_yz = flag;
      is_dirty_zx = flag;
      is_dirty_zy = flag;
      is_dirty_zz = flag;
      dirty_count = flag ? 12 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_x;
    bool is_dirty_y;
    bool is_dirty_z;
    bool is_dirty_xx;
    bool is_dirty_xy;
    bool is_dirty_xz;
    bool is_dirty_yx;
    bool is_dirty_yy;
    bool is_dirty_yz;
    bool is_dirty_zx;
    bool is_dirty_zy;
    bool is_dirty_zz;
  };
};

#endif
