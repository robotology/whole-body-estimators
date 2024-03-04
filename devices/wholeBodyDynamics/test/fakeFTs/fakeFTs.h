/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/math/Math.h>

/**
* @ingroup dev_impl_fake
* \brief `fakeFTs` : fake device implementing the device interface typically implemented by an Inertial Measurement Unit
*
* | YARP device name |
* |:-----------------:|
* | `fakeFTs` |
*
*
* The parameters accepted by this device are:
* | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
* |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
* | period         |       -        | int     | millisecond    |   10          | No          | Period over which the measurement is updated.  |       |
* | sensorNames    |       -        | int     | millisecond    |   10          | No          | Name of sensor names.  |       |
*
*/
class fakeFTs :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
        public yarp::dev::ITemperatureSensors,
        public yarp::dev::ISixAxisForceTorqueSensors
{
public:
    fakeFTs();
    fakeFTs(const fakeFTs&) = delete;
    fakeFTs(fakeFTs&&) = delete;
    fakeFTs& operator=(const fakeFTs&) = delete;
    fakeFTs& operator=(fakeFTs&&) = delete;

    ~fakeFTs() override;

    // Device Driver interface
    bool open(yarp::os::Searchable &config) override;
    bool close() override;

    // ITemperatureSensors
    virtual size_t getNrOfTemperatureSensors() const override;
    virtual yarp::dev::MAS_status getTemperatureSensorStatus(size_t sens_index) const override;
    virtual bool getTemperatureSensorName(size_t sens_index, std::string &name) const override;
    virtual bool getTemperatureSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const override;
    virtual bool getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    // ISixAxisForceTorqueSensors
    virtual size_t getNrOfSixAxisForceTorqueSensors() const override;
    virtual yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const override;
    virtual bool getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const override;
    virtual bool getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

private:
    size_t genericGetNrOfSensors() const;
    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const;
    bool genericGetSensorName(size_t sens_index, std::string &name) const;
    bool genericGetFrameName(size_t sens_index, std::string &frameName) const;

    bool threadInit() override;
    void run() override;

    double dummy_value;
    yarp::os::Stamp lastStamp;
    std::vector<std::string> m_sensorNames;

    bool constantValue;
};
