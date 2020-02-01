/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_DEV_VIRTUALANALOGCLIENT_H
#define YARP_DEV_VIRTUALANALOGCLIENT_H

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>

#include <vector>

namespace yarp {
namespace dev {

/**
*  @ingroup dev_impl_wrapper
*
* \brief Device that connects to a virtualAnalogServer to publish torque estimates.
*
* \section VirtualAnalogClient Description of input parameters
*
*  This device will connect to a virtualAnalogServer to publish torque estimates.
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required  | Description   | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------: |:-------------:|:-----:|
* | local          | string |       |               | Yes       | full name if the port opened by the device  | must start with a '/' character |
* | remote         | string |       |               | Yes       | full name of the port the device need to connect to | must start with a '/' character |
* | carrier        | string |       | tcp           | No        | type of carrier to use, like tcp, udp and so on ...  | - |
* | AxisName       | vector of strings | - |    -   | Yes       | name of the axes in which the torque estimate is published | - |
* | AxisType       | vector of strings | - |revolute| No        | type of the axies in which the torque estimate is published | - |
* | virtualAnalogSensorInteger | int | - | -        | Yes       | A virtualAnalogServer specific integer, check the VirtualAnalogServer for more info.  | - |
* | autoconnect    |   bool    |   -   |    true  | No        | Specify if port should be connected or not | - |
*
*  The device will create a port with name <local> and will connect to a port colled <remote> at startup,
* ex: <b> /wholeBodyDynamics/left_leg/Torques:o  </b>, and will connect to a port called <b> /icub/joint_vsens/left_leg:i <b>.
*
* A new message is written on this YARP port as soon as the updateMeasure is called (so the multiaxis updateMeasure should
* always be prefered).
*
* For the single axis updateMeasure, the value sent for the not-update axis will be the one stored in a buffer, that is initialized to zero.
*
**/
class VirtualAnalogClient:    public DeviceDriver,
                              public IVirtualAnalogSensor,
                              public IAxisInfo
{
protected:
    yarp::os::ConstString m_local;
    yarp::os::ConstString m_remote;
    int m_virtualAnalogSensorInteger;

    std::vector<yarp::os::ConstString> m_axisName;
    std::vector<yarp::dev::JointTypeEnum> m_axisType;

    yarp::os::BufferedPort<yarp::os::Bottle> m_outputPort;

    yarp::sig::Vector measureBuffer;

    /**
     * Publish the data contained in the measureBuffer on the port.
     */
    void sendData();

public:
    VirtualAnalogClient();
    virtual ~VirtualAnalogClient();

    /* DevideDriver methods (documented in DeviceDriver class) */
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
};

}
}

#endif // YARP_DEV_VIRTUALANALOGCLIENT_H
