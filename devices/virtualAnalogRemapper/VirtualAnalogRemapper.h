/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_DEV_VIRTUALANALOGREMAPPER_H
#define YARP_DEV_VIRTUALANALOGREMAPPER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/Wrapper.h>

#include <vector>

namespace yarp {
namespace dev {

/**
 * Structure of information relative to a remapped axis.
 */
struct virtualAnalogSensorRemappedAxis
{
    IVirtualAnalogSensor * dev;
    IAxisInfo            * devInfo;
    int localAxis;
    int devIdx;
    bool useVectorUpdateMeasure;
};

/**
 * Structure of information relative to a remapped subdevice.
 */
struct virtualAnalogSensorRemappedSubdevice
{
    IVirtualAnalogSensor * dev;
    yarp::sig::Vector measureBuffer;
    std::vector<int> local2globalIdx;
    bool useVectorUpdateMeasure;
};




/**
*  @ingroup dev_impl_wrapper
*
* \brief Device that remaps multiple device exposing a IVirtualAnalogSensor to a have them behave as a single device.
*
* \section VirtualAnalogRemapper Description of input parameters
*
*  This device will connect to multiple devices exposing the IVirtualAnalogSensor interface,
*  exposing them as a single device.
*
*  The remapping is done using the IAxisInfo interface.
*
*  The main difference with respect to the style of remapping provided by the ControlBoardRemapper
*  is that the IVirtualAnalogSensor does not provide updateMeasure methods for a subset of the axes.
*  Using just the single-axis version is tipically doable when this device is running on the robot, but
*  it may be prohibitly expensive to do when the measure are published on a port, as each call to the
*  single-axis updateMeasure may involve to send a packed.
*  Consequently if the VirtualAnalogRemapper detects that all channels in a subdevice are part
*  of the remapped device, the vector-value updateMeasure method will be used.
*
*
*  Parameters required by this device are:
* | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
* |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
* | axesNames     |      -          | vector of strings  | -   |   -           | Yes     | Ordered list of the axes that are part of the remapped device. |  |
*
* The axes are then mapped to the wrapped controlboard in the attachAll method, using the
* values returned by the getAxisName method of the attached devices.
*
* Configuration file using .ini format.
*
* \code{.unparsed}
*  device virtualAnalogRemapper
*  axesNames (joint1 joint2 joint3)
* ...
* \endcode
*
**/
class VirtualAnalogRemapper:  public DeviceDriver,
                              public IVirtualAnalogSensor,
                              public IMultipleWrapper,
                              public IAxisInfo
{
protected:

    std::vector<std::string> m_axesNames;

    /**
     * Vector containg the information about a specific axis remapped by this device.
     * This is configured during the attachAll method.
     * This vector will always be of size getChannels()
     */
    std::vector<virtualAnalogSensorRemappedAxis> remappedAxes;

    /**
     * Vector containing the information about a specific subdevice remapped by this device.
     * This vector will have the dimension of the number of mapper subdevices,
     * but will be used only if for a specific subdevice it would be detected that all the channels
     * are part of the remapper device.
     */
    std::vector<virtualAnalogSensorRemappedSubdevice> remappedSubdevices;

    /**
     * Get the number of remapped devices. 
     */
    int getNrOfSubDevices();


public:
    VirtualAnalogRemapper();
    virtual ~VirtualAnalogRemapper();

    /* DeviceDriver methods (documented in DeviceDriver class) */
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    /* IVirtualAnalogSensor methods (documented in IVirtualAnalogSensor class) */
    virtual VAS_status getVirtualAnalogSensorStatus(int ch);
    virtual int getVirtualAnalogSensorChannels();
    virtual bool updateVirtualAnalogSensorMeasure(yarp::sig::Vector &measure);
    virtual bool updateVirtualAnalogSensorMeasure(int ch, double &measure);
    
    /** IAxisInfo methods (documented in IVirtualAnalogSensor class) */
    virtual bool getAxisName(int axis, yarp::os::ConstString& name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);

    /** IMultipleWrapper methods (documented in IMultipleWrapper */
    virtual bool attachAll(const PolyDriverList &p);
    virtual bool detachAll();
};

}
}

#endif // YARP_DEV_VIRTUALANALOGREMAPPER_H
