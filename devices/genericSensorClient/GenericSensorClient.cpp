/*
* Copyright (C) 2010 RobotCub Consortium
* Author: Lorenzo Natale
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

#include <GenericSensorClient.h>

#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>


const int GS_ANALOG_TIMEOUT=100; //ms

using namespace yarp::os;

inline void GSInputPortProcessor::resetStat()
{
    yarp::os::LockGuard guard(mutex);

    dataAvailable = false;
    count=0;
    deltaT=0;
    deltaTMax=0;
    deltaTMin=1e22;
    now=Time::now();
    prev=now;
}

GSInputPortProcessor::GSInputPortProcessor()
{
    resetStat();
}

void GSInputPortProcessor::onRead(yarp::sig::Vector &v)
{
    now=Time::now();

    yarp::os::LockGuard guard(mutex);

    if (count>0)
    {
        double tmpDT=now-prev;

        deltaT+=tmpDT;

        if (tmpDT>deltaTMax)
        {
            deltaTMax=tmpDT;
        }

        if (tmpDT<deltaTMin)
        {
            deltaTMin=tmpDT;
        }

        //compare network time
        if (tmpDT*1000<GS_ANALOG_TIMEOUT)
        {
            dataAvailable=true;
        }
        else
        {
            dataAvailable=false;
        }
    }

    if( count == 0 )
    {
        dataAvailable = true;
    }

    prev=now;
    count++;

    lastVector=v;
    lastStamp.update();

}

inline bool GSInputPortProcessor::getLast(yarp::sig::Vector &data, Stamp &stmp)
{
    yarp::os::LockGuard guard(mutex);

    if (dataAvailable)
    {
        data=lastVector;
        stmp = lastStamp;
    }

    return dataAvailable;
}

inline int GSInputPortProcessor::getIterations()
{
    yarp::os::LockGuard guard(mutex);

    int ret=count;
    return ret;
}

// time is in ms
void GSInputPortProcessor::getEstFrequency(int &ite, double &av, double &min, double &max)
{
    yarp::os::LockGuard guard(mutex);

    ite=count;
    min=deltaTMin*1000;
    max=deltaTMax*1000;
    if (count<1)
    {
        av=0;
    }
    else
    {
        av=deltaT/count;
    }
    av=av*1000;
}

bool GSInputPortProcessor::getState()
{
    return dataAvailable;
}

int GSInputPortProcessor::getChannels()
{
    return (int)lastVector.size();
}


bool yarp::dev::GenericSensorClient::open(yarp::os::Searchable &config)
{
    ConstString carrier = config.check("carrier", Value("udp"), "default carrier for streaming robot state").asString().c_str();

    local.clear();
    remote.clear();

    local  = config.find("local").asString().c_str();
    remote = config.find("remote").asString().c_str();

    if (local=="")
    {
        yError("GenericSensorClient::open() error you have to provide valid local name");
        return false;
    }
    if (remote=="")
    {
        yError("GenericSensorClient::open() error you have to provide valid remote name\n");
        return false;
    }

    if (!inputPort.open(local.c_str()))
    {
        yError("GenericSensorClient::open() error could not open port %s, check network", local.c_str());
        return false;
    }
    inputPort.useCallback();

    bool ok=Network::connect(remote.c_str(), local.c_str(), carrier.c_str());
    if (!ok)
    {
        yError("GenericSensorClient::open() error could not connect to %s", remote.c_str());
        return false;
    }

    return true;
}

bool yarp::dev::GenericSensorClient::close()
{
    inputPort.close();
    return true;
}

bool yarp::dev::GenericSensorClient::read(yarp::sig::Vector &out)
{
    return inputPort.getLast(out, lastTs);
}

bool yarp::dev::GenericSensorClient::calibrate(int /*ch*/, double /*v*/)
{
    return false;
}

bool yarp::dev::GenericSensorClient::getChannels(int* nc)
{
    bool ret = inputPort.getState();

    *nc = inputPort.getChannels();

    return ret;
}

Stamp yarp::dev::GenericSensorClient::getLastInputStamp()
{
    return lastTs;
}