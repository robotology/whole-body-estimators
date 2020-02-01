/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "VirtualAnalogRemapper.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <map>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

VirtualAnalogRemapper::VirtualAnalogRemapper()
{

}

VirtualAnalogRemapper::~VirtualAnalogRemapper()
{

}

bool VirtualAnalogRemapper::open(Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString());

    if( !( prop.check("axesNames") && prop.find("axesNames").isList() ) )
    {
        yError("VirtualAnalogRemapper: Missing required axesNames string parameter");
        return false;
    }

    Bottle * axesNamesBot = prop.find("axesNames").asList();

    m_axesNames.resize(axesNamesBot->size());
    for(int jnt=0; jnt < axesNamesBot->size(); jnt++)
    {
        m_axesNames[jnt] = axesNamesBot->get(jnt).asString();
    }

    // Waiting for attach now
    return true;
}

bool VirtualAnalogRemapper::attachAll(const PolyDriverList& p)
{
    // For the moment we assume that the controlboard device is the
    // same that implements the IVirtualAnalogSensor interface (this is how things
    // are implemented in embObjMotionControl) In general we at least assume that
    // all the devices that implement IVirtualAnalogSensor also implement IAxisInfo
    // to provide a name for the virtual analog axis
    std::vector<IVirtualAnalogSensor *> virtualAnalogList;
    std::vector<IAxisInfo *>            axisInfoList;
    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
    {
        IVirtualAnalogSensor * pVirtualAnalogSens = 0;
        IAxisInfo            * pAxisInfo          = 0;
        if( p[devIdx]->poly->view(pVirtualAnalogSens) )
        {
            virtualAnalogList.push_back(pVirtualAnalogSens);
            if( !(p[devIdx]->poly->view(pAxisInfo)) )
            {
                yError() << "VirtualAnalogRemapper : attachAll error: device "
                         << p[devIdx]->key << " exposes a IVirtualAnalogSensor, but not a IAxisInfo interface,"
                         << " impossible to map the list of joint names to the IVirtualAnalogSensor interface";
                return false;
            }
            else
            {
                axisInfoList.push_back(pAxisInfo);
            }
        }
    }

    // Find the axisName ---> device , localAxis mapping
    std::map<std::string, IVirtualAnalogSensor *> axisName2virtualAnalogSensorPtr;
    std::map<std::string, IAxisInfo *> axisName2IAxisInfoPtr;
    std::map<std::string, int> axisName2localAxis;

    for(size_t devIdx = 0; devIdx < virtualAnalogList.size(); devIdx++)
    {
        int nrOfVirtualAxes = virtualAnalogList[devIdx]->getVirtualAnalogSensorChannels();
        for(int localAxis=0; localAxis < nrOfVirtualAxes; localAxis++)
        {
            yarp::os::ConstString axisName;
            axisInfoList[devIdx]->getAxisName(localAxis,axisName);

            std::string axisNameStd = axisName.c_str();

            axisName2virtualAnalogSensorPtr[axisNameStd] = virtualAnalogList[devIdx];
            axisName2IAxisInfoPtr[axisNameStd] = axisInfoList[devIdx];
            axisName2localAxis[axisNameStd] = localAxis;
        }
    }

    // Save the axis ---> device, localAxis mapping
    remappedAxes.resize(m_axesNames.size());

    for(size_t axis = 0; axis < remappedAxes.size(); axis++)
    {
        std::string jointName = m_axesNames[axis];

        // if the name is not exposed in the VirtualAnalogSensors, save it as a null pointer
        if( axisName2virtualAnalogSensorPtr.find(jointName) == axisName2virtualAnalogSensorPtr.end() )
        {
            remappedAxes[axis].dev = 0;
            remappedAxes[axis].devInfo = 0;
            remappedAxes[axis].localAxis = 0;


            // we support not publishing the information for some axis
            // (as most of the time we estimates torques for axis that don't have a virtual analog sensor)
        }
        else
        {
            remappedAxes[axis].dev = axisName2virtualAnalogSensorPtr[jointName];
            remappedAxes[axis].devInfo = axisName2IAxisInfoPtr[jointName];
            remappedAxes[axis].localAxis = axisName2localAxis[jointName];
        }
    }


    // Let's fill the remappedSubdevices structure
    remappedSubdevices.resize(virtualAnalogList.size());

    for(size_t subdev = 0; subdev < virtualAnalogList.size(); subdev++)
    {
        remappedSubdevices[subdev].dev = virtualAnalogList[subdev];
        remappedSubdevices[subdev].measureBuffer.resize(remappedSubdevices[subdev].dev->getVirtualAnalogSensorChannels(),0.0);
        remappedSubdevices[subdev].local2globalIdx.resize(remappedSubdevices[subdev].dev->getVirtualAnalogSensorChannels(),-1);

        // Let's fill the local2globalIdx vector by searching on all the vector
        // In the meanwhile, we also count the channels of the subdevices that are remapped:
        // if all the axes on subdevice are remapped, we can use the updateMeasure(Vector&out) method
        int axesOfSubDeviceThatAreRemapped=0;
        for(size_t localIndex = 0; localIndex < remappedSubdevices[subdev].local2globalIdx.size(); localIndex++)
        {
            for(size_t globalAxis = 0; globalAxis < remappedAxes.size(); globalAxis++)
            {
                if( (remappedAxes[globalAxis].dev == remappedSubdevices[subdev].dev) &&
                    (remappedAxes[globalAxis].localAxis == (int)localIndex) )
                {
                    remappedSubdevices[subdev].local2globalIdx[localIndex] = globalAxis;
                    remappedAxes[globalAxis].devIdx = (int)subdev;
                    axesOfSubDeviceThatAreRemapped++;
                }
            }
        }

        // If all the axes of the subdevice are remapped, we can use the vector updateMeasure
        if( axesOfSubDeviceThatAreRemapped == remappedSubdevices[subdev].dev->getVirtualAnalogSensorChannels() )
        {
            remappedSubdevices[subdev].useVectorUpdateMeasure = true;
            for(size_t localIndex = 0; localIndex < remappedSubdevices[subdev].local2globalIdx.size(); localIndex++)
            {
                int globalIdx = remappedSubdevices[subdev].local2globalIdx[localIndex];
                remappedAxes[globalIdx].useVectorUpdateMeasure = true;
            }
        }
        else
        {
            remappedSubdevices[subdev].useVectorUpdateMeasure = false;
            for(size_t localIndex = 0; localIndex < remappedSubdevices[subdev].local2globalIdx.size(); localIndex++)
            {
                int globalIdx = remappedSubdevices[subdev].local2globalIdx[localIndex];

                if( globalIdx > 0 )
                {
                    remappedAxes[globalIdx].useVectorUpdateMeasure = false;
                }
            }
        }
    }

    return true;
}


bool VirtualAnalogRemapper::detachAll()
{
    remappedAxes.resize(0);
    remappedSubdevices.resize(0);

    return true;
}


bool VirtualAnalogRemapper::close()
{
    detachAll();
    return true;
}

bool VirtualAnalogRemapper::updateVirtualAnalogSensorMeasure(Vector& measure)
{
    bool ret = true;

    if( (int) measure.size() != this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogClient: updateMeasure failed : input measure has size " << measure.size() << " while the client is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
        return false;
    }


    // Call the vector updateMeasure for subdevices in which all parts are remapped
    for(size_t subdevIdx=0; subdevIdx < remappedSubdevices.size(); subdevIdx++)
    {
        if( this->remappedSubdevices[subdevIdx].useVectorUpdateMeasure )
        {
            IVirtualAnalogSensor * dev = this->remappedSubdevices[subdevIdx].dev;

            // Update the measure buffer
            for(size_t localIndex = 0; localIndex < this->remappedSubdevices[subdevIdx].measureBuffer.size(); localIndex++)
            {
                int globalIndex = this->remappedSubdevices[subdevIdx].local2globalIdx[localIndex];
                this->remappedSubdevices[subdevIdx].measureBuffer[localIndex] = measure[globalIndex];
            }

            bool ok = dev->updateVirtualAnalogSensorMeasure(this->remappedSubdevices[subdevIdx].measureBuffer);
            ret = ok && ret;
        }
    }

    // use single axis method (for axis that are not already updated)
    for(int jnt=0; jnt < this->getVirtualAnalogSensorChannels(); jnt++)
    {
        if( !(this->remappedAxes[jnt].useVectorUpdateMeasure) )
        {
            IVirtualAnalogSensor * dev = this->remappedAxes[jnt].dev;
            int localAxis = this->remappedAxes[jnt].localAxis;

            if( dev )
            {
                bool ok = dev->updateVirtualAnalogSensorMeasure(localAxis,measure[jnt]);
                ret = ok && ret;
            }
        }
    }

    return ret;
}

bool VirtualAnalogRemapper::updateVirtualAnalogSensorMeasure(int ch, double& measure)
{
    if( ch < 0 || ch >= this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogRemapper: updateMeasure failed : requested channel " << ch << " while the client is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
    }

    // In this case we need to use the single axis method
    IVirtualAnalogSensor * dev = this->remappedAxes[ch].dev;
    int localAxis = this->remappedAxes[ch].localAxis;

    bool ret;
    if( dev )
    {
        ret = dev->updateVirtualAnalogSensorMeasure(localAxis,measure);
    }
    else
    {
        ret = false;
    }
    return ret;
}

int VirtualAnalogRemapper::getVirtualAnalogSensorChannels()
{
    return this->m_axesNames.size();
}

yarp::dev::VAS_status VirtualAnalogRemapper::getVirtualAnalogSensorStatus(int ch)
{
    if( ch < 0 || ch >= this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogRemapper: getState failed : requested channel " << ch << " while the client is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
    }

    // In this case we need to use the single axis method
    IVirtualAnalogSensor * dev = this->remappedAxes[ch].dev;
    int localAxis = this->remappedAxes[ch].localAxis;

    yarp::dev::VAS_status status;
    if( dev )
    {
        status = dev->getVirtualAnalogSensorStatus(localAxis);
    }
    else
    {
        status = yarp::dev::VAS_status::VAS_ERROR;
    }
    return status;
}

bool VirtualAnalogRemapper::getAxisName(int axis, ConstString& name)
{
    if( axis < 0 || axis >= this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogRemapper: getAxisName failed : requested axis " << axis << " while the remapper is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
        return false;
    }

    // In this case we need to use the single axis method
    IAxisInfo * dev = this->remappedAxes[axis].devInfo;
    int localAxis = this->remappedAxes[axis].localAxis;

    bool ret;
    if( dev )
    {
        ret = dev->getAxisName(localAxis,name);
    }
    else
    {
        ret = false;
    }
    return ret;
}

bool VirtualAnalogRemapper::getJointType(int axis, JointTypeEnum& type)
{
    if( axis < 0 || axis >= this->getVirtualAnalogSensorChannels() )
    {
        yError() << "VirtualAnalogRemapper: getJointType failed : requested axis " << axis << " while the remapper is configured with " << this->getVirtualAnalogSensorChannels() << " channels";
        return false;
    }

    // In this case we need to use the single axis method
    IAxisInfo * dev = this->remappedAxes[axis].devInfo;
    int localAxis = this->remappedAxes[axis].localAxis;

    bool ret;
    if( dev )
    {
        ret = dev->getJointType(localAxis,type);
    }
    else
    {
        ret = false;
    }
    return ret;
}
