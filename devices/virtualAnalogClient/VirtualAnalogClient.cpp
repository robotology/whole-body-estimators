/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "VirtualAnalogClient.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

VirtualAnalogClient::VirtualAnalogClient()
{

}

VirtualAnalogClient::~VirtualAnalogClient()
{

}

bool VirtualAnalogClient::open(Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString());

    if( !( prop.check("local") && prop.find("local").isString() ) )
    {
        yError("VirtualAnalogClient: Missing required local string parameter");
        return false;
    }

    m_local = prop.find("local").asString();

    if( !( prop.check("remote") && prop.find("remote").isString() ) )
    {
        yError("VirtualAnalogClient: Missing required remote string parameter");
        return false;
    }

    m_remote = prop.find("remote").asString();

    yarp::os::ConstString carrier;
    if( ( prop.check("carrier") && prop.find("carrier").isString() ) )
    {
        carrier = prop.find("carrier").asString();
    }
    else
    {
        carrier = "tcp";
    }

    if( !( prop.check("AxisName") && prop.find("AxisName").isList() ) )
    {
        yError("VirtualAnalogClient: Missing required AxisName list parameter");
        return false;
    }

    Bottle * AxisNameBot = prop.find("AxisName").asList();

    m_axisName.resize(AxisNameBot->size());
    for(int jnt=0; jnt < AxisNameBot->size(); jnt++)
    {
        m_axisName[jnt] = AxisNameBot->get(jnt).asString();
    }

    // Handle type
    m_axisType.resize(m_axisName.size());

    if( ( prop.check("AxisType") && prop.find("AxisType").isList() ) )
    {
        Bottle * AxisTypeBot = prop.find("AxisType").asList();
        for(int jnt=0; jnt < AxisTypeBot->size(); jnt++)
        {
            ConstString type = AxisTypeBot->get(jnt).asString();
            if (type == "revolute")
            {
                m_axisType[jnt] = VOCAB_JOINTTYPE_REVOLUTE;
            }
            else if (type == "prismatic")
            {
                m_axisType[jnt] = VOCAB_JOINTTYPE_UNKNOWN;
            }
            else
            {
                yError() << "VirtualAnalogClient: unknown joint type " << type;
                return false;
            }
        }
    }
    else
    {
        for(size_t jnt=0; jnt < m_axisType.size(); jnt++)
        {
            m_axisType[jnt] = VOCAB_JOINTTYPE_REVOLUTE;
        }
    }

    if( !( prop.check("virtualAnalogSensorInteger") && prop.find("virtualAnalogSensorInteger").isInt() ) )
    {
        yError("VirtualAnalogClient: Missing required virtualAnalogSensorInteger int parameter");
        return false;
    }

    m_virtualAnalogSensorInteger = prop.find("virtualAnalogSensorInteger").isInt();

    // Resize buffer
    measureBuffer.resize(m_axisName.size(),0.0);

    // Open the port
    bool ok = m_outputPort.open(m_local);

    if( !ok )
    {
        yError() << "VirtualAnalogClient: impossible to open port " << m_local;
    }

    // Connect the port
    bool autoconnect = true;

    if( prop.check("autoconnect") )
    {
        if( prop.find("autoconnect").isBool() )
        {
            autoconnect = prop.find("autoconnect").asBool();
        }
        else 
        {
            yError() << "VirtualAnalogClient: autoconnect option found, but is not a bool, exiting.";
        }
    }

    if( autoconnect )
    {
        ok = Network::connect(m_local,m_remote,carrier);

        if( !ok )
        {
            yError() << "VirtualAnalogClient: impossible to connect port " << m_local << " to " << m_remote << " with carrier " << carrier;
            return false;
        }
    }

    return true;
}

bool VirtualAnalogClient::close()
{
    bool ok = Network::disconnect(m_local,m_remote);
    m_outputPort.close();
    return ok;
}

bool VirtualAnalogClient::updateVirtualAnalogSensorMeasure(Vector& measure)
{
    if( (int) measure.size() != this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogClient: updateMeasure failed : input measure has size " << measure.size() << " while the client is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
        return false;
    }

    measureBuffer = measure;

    sendData();

    return true;
}

bool VirtualAnalogClient::updateVirtualAnalogSensorMeasure(int ch, double& measure)
{
    if( ch < 0 || ch >= this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogClient: updateMeasure failed : requested channel " << ch << " while the client is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
    }

    measureBuffer[ch] = measure;

    sendData();

    return true;
}

void VirtualAnalogClient::sendData()
{
    Bottle & a = m_outputPort.prepare();
    a.clear();
    a.addInt(m_virtualAnalogSensorInteger);
    for(size_t i=0;i<measureBuffer.length();i++)
    {
        a.addDouble(measureBuffer(i));
    }
    m_outputPort.write();
}

int VirtualAnalogClient::getVirtualAnalogSensorChannels()
{
    return this->m_axisName.size();
}

VAS_status VirtualAnalogClient::getVirtualAnalogSensorStatus(int /*ch*/)
{
    return yarp::dev::VAS_status::VAS_OK;
}

bool VirtualAnalogClient::getAxisName(int axis, ConstString& name)
{
    if( axis < 0 || axis >= this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogClient: getAxisName failed : requested axis " << axis << " while the client is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
        return false;
    }

    name = m_axisName[axis];

    return true;
}

bool VirtualAnalogClient::getJointType(int axis, JointTypeEnum& type)
{
    if( axis < 0 || axis >= this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogClient: getJointType failed : requested axis " << axis << " while the client is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
        return false;
    }

    type = m_axisType[axis];
    return true;
}
