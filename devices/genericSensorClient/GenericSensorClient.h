/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_DEV_GENERICSENSORCLIENT_H
#define YARP_DEV_GENERICSENSORCLIENT_H


#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>

/**
 * Class copied from the InputPortProcessor class in AnalogSensorClient.
 * Once we port this in YARP we can merge this two classes.
 */
class GSInputPortProcessor : public yarp::os::BufferedPort<yarp::sig::Vector>
{
    yarp::os::Mutex mutex;
    yarp::sig::Vector lastVector;
    yarp::os::Stamp lastStamp;
    double deltaT;
    double deltaTMax;
    double deltaTMin;
    double prev;
    double now;

    bool dataAvailable;

    int count;

public:
    inline void resetStat();

    GSInputPortProcessor();

    using yarp::os::BufferedPort<yarp::sig::Vector>::onRead;
    virtual void onRead(yarp::sig::Vector &v);

    inline bool getLast(yarp::sig::Vector &data, yarp::os::Stamp &stmp);

    inline int getIterations();

    // time is in ms
    void getEstFrequency(int &ite, double &av, double &min, double &max);

    bool getState();

    int getChannels();
};


namespace yarp {
namespace dev {

/**
* @ingroup dev_impl_wrapper
*
* \brief Device that reads a sensor exposing a IGenericSensor interface from the YARP network.
*
* This device will connect to a port opened by the ServerInertial device and read the data broadcasted
* making them available to use for the user application.
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required  | Description   | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------: |:-------------:|:-----:|
* | local          | string |       |               | Yes       | full name if the port opened by the device  | must start with a '/' character |
* | remote         | string |       |               | Yes       | full name of the port the device need to connect to | must start with a '/' character |
* | carrier        | string |       | udp           | No        | type of carrier to use, like tcp, udp and so on ...  | - |
*
*  The device will create a port with name <local> and will connect to a port colled <remote> at startup,
* ex: <b> /myModule/linertial </b>, and will connect to a port called <b> /icub/inertial<b>.
*
**/
class GenericSensorClient: public yarp::dev::DeviceDriver,
                                      public yarp::dev::IPreciselyTimed,
                                      public yarp::dev::IGenericSensor
{
protected:
    GSInputPortProcessor inputPort;
    yarp::os::ConstString local;
    yarp::os::ConstString remote;
    yarp::os::Stamp lastTs; //used by IPreciselyTimed

    void  removeLeadingTrailingSlashesOnly(std::string &name);

public:

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config);
    bool close();

    /* IGenericSensor methods */
    virtual bool read(yarp::sig::Vector &out);
    virtual bool getChannels(int *nc);

    /**
     * Not implemeted in GenericSensorClient
     */
    virtual bool calibrate(int ch, double v);

    /* IPreciselyTimed methods */
    yarp::os::Stamp getLastInputStamp();
};

}

}

#endif // YARP_DEV_GENERICSENSORCLIENT_H
