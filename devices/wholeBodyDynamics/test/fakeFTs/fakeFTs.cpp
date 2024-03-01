// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
// SPD X-License-Identifier: BSD-3-Clause


#include "fakeFTs.h"

#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <string>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

namespace {
YARP_LOG_COMPONENT(fakeFTs, "yarp.device.fakeFTs")
constexpr double DEFAULT_PERIOD = 0.01; // seconds
constexpr double DEFAULT_DUMMY_VALUE = 0.0;
}

/**
 * This device implements a fake analog sensor
 * emulating an IMU
 */
fakeFTs::fakeFTs() :
        PeriodicThread(DEFAULT_PERIOD)
{
}

fakeFTs::~fakeFTs()
{
    close();
}

bool fakeFTs_getConfigParamsAsList(yarp::os::Searchable& config,std::string propertyName , std::vector<std::string> & list, bool isRequired)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());
    yarp::os::Bottle *propNames=prop.find(propertyName).asList();
    if(propNames==nullptr)
    {
        if(isRequired)
        {
            yError() <<"fakeFTs: Error parsing parameters: \" "<<propertyName<<" \" should be followed by a list\n";
        }
        return false;
    }

    list.resize(propNames->size());
    for(auto elem=0u; elem < propNames->size(); elem++)
    {
        list[elem] = propNames->get(elem).asString().c_str();
    }

    return true;
}

bool fakeFTs::open(yarp::os::Searchable &config)
{
    double period;
    if( config.check("period")) {
        period = config.find("period").asInt32() / 1000.0;
        setPeriod(period);
    } else  {
        yInfo() << "Using default period of " << DEFAULT_PERIOD << " s";
    }

    bool required = true;
    bool ok = fakeFTs_getConfigParamsAsList(config, "sensorNames", m_sensorNames, required);

    if (!ok) {
        return false;
    }

    start();
    return true;
}

bool fakeFTs::close()
{
    fakeFTs::stop();
    return true;
}

bool fakeFTs::threadInit()
{
    lastStamp.update();
    return true;
}

void fakeFTs::run()
{
    static double count=10;

    lastStamp.update();

    dummy_value = count;
    if (!constantValue) {
        count++;
    }

    if (count >= 360) {
        count = 0;
    }
}

size_t fakeFTs::genericGetNrOfSensors() const
{
    return m_sensorNames.size();
}

yarp::dev::MAS_status fakeFTs::genericGetStatus(size_t sens_index) const
{
    if (sens_index>m_sensorNames.size()) {
        return yarp::dev::MAS_status::MAS_ERROR;
    }

    return yarp::dev::MAS_status::MAS_OK;
}

bool fakeFTs::genericGetSensorName(size_t sens_index, std::string &name) const
{
    if (sens_index>m_sensorNames.size()) {
        return false;
    }

    name = m_sensorNames[sens_index];
    return true;
}

bool fakeFTs::genericGetFrameName(size_t sens_index, std::string &frameName) const
{
    if (sens_index>m_sensorNames.size()) {
        return false;
    }

    frameName = m_sensorNames[sens_index];
    return true;
}

// ---------------------- ITemperatureSensors --------------------------------------------------------
size_t fakeFTs::getNrOfTemperatureSensors() const
{
    return genericGetNrOfSensors();
}

yarp::dev::MAS_status fakeFTs::getTemperatureSensorStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool fakeFTs::getTemperatureSensorName(size_t sens_index, std::string &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool fakeFTs::getTemperatureSensorFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool fakeFTs::getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const
{
    if (sens_index>m_sensorNames.size()) {
        return false;
    }

    out = dummy_value;
    timestamp = lastStamp.getTime();

    return true;
}

bool fakeFTs::getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    out.resize(1);
    out[0] = dummy_value;
    timestamp = lastStamp.getTime();
    return true;
}

//------------------------- ISixAxisForceTorqueSensors -------------------------

size_t fakeFTs::getNrOfSixAxisForceTorqueSensors() const
{
    return genericGetNrOfSensors();
}

yarp::dev::MAS_status fakeFTs::getSixAxisForceTorqueSensorStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool fakeFTs::getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool fakeFTs::getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool fakeFTs::getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index>m_sensorNames.size()) {
        return false;
    }

    out.resize(6);
    for (size_t i=0; i < 6; i++) {
        out[i] = dummy_value;
    }
    timestamp = lastStamp.getTime();

    return true;
}
