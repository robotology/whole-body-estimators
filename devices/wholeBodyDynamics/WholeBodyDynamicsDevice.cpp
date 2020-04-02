#define SKIN_EVENTS_TIMEOUT 0.2
#include "WholeBodyDynamicsDevice.h"

#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/Utils.h>

#include <cassert>
#include <cmath>

namespace yarp
{
namespace dev
{

const size_t wholeBodyDynamics_nrOfChannelsOfYARPFTSensor = 6;
const size_t wholeBodyDynamics_nrOfChannelsOfAYARPIMUSensor = 12;
const double wholeBodyDynamics_sensorTimeoutInSeconds = 2.0;

WholeBodyDynamicsDevice::WholeBodyDynamicsDevice(): RateThread(10),
                                                    portPrefix("/wholeBodyDynamics"),
                                                    streamFilteredFT(false),
                                                    correctlyConfigured(false),
                                                    sensorReadCorrectly(false),
                                                    estimationWentWell(false),
                                                    validOffsetAvailable(false),
                                                    lastReadingSkinContactListStamp(0.0),
                                                    settingsEditor(settings)
{
    // Calibration quantities
    calibrationBuffers.ongoingCalibration = false;
    calibrationBuffers.calibratingFTsensor.resize(0);
    calibrationBuffers.offsetSumBuffer.resize(0);
    ftProcessors.resize(0);
    calibrationBuffers.estimationSumBuffer.resize(0);
    calibrationBuffers.measurementSumBuffer.resize(0);
    calibrationBuffers.nrOfSamplesToUseForCalibration = 0;
    calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration = 0;

}

WholeBodyDynamicsDevice::~WholeBodyDynamicsDevice()
{

}


bool WholeBodyDynamicsDevice::openSettingsPort()
{
    settingsPort.setReader(settingsEditor);

    bool ok = settingsPort.open(portPrefix+"/settings");

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice: Impossible to open port " << portPrefix+"/settings";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::openRPCPort()
{
    this->wholeBodyDynamics_IDLServer::yarp().attachAsServer(rpcPort);

    bool ok = rpcPort.open(portPrefix+"/rpc");

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice: Impossible to open port " << portPrefix+"/rpc";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::closeSettingsPort()
{
    settingsPort.close();
    return true;
}

bool WholeBodyDynamicsDevice::closeRPCPort()
{
    rpcPort.close();
    return true;
}

bool WholeBodyDynamicsDevice::closeSkinContactListsPorts()
{
    this->portContactsInput.close();
    this->portContactsOutput.close();

    return true;
}

bool WholeBodyDynamicsDevice::closeExternalWrenchesPorts()
{
    for(unsigned int i = 0; i < outputWrenchPorts.size(); i++ )
    {
        if( outputWrenchPorts[i].output_port )
        {
            outputWrenchPorts[i].output_port->close();
            delete outputWrenchPorts[i].output_port;
            outputWrenchPorts[i].output_port = 0;
        }
    }
    return true;
}

bool WholeBodyDynamicsDevice::closeFilteredFTPorts()
{
    for(unsigned int i = 0; i < outputFTPorts.size(); i++ )
    {
        if( outputFTPorts[i] )
        {
            outputFTPorts[i]->close();
            outputFTPorts[i].reset();
        }
    }
    return true;
}



void addVectorOfStringToProperty(yarp::os::Property& prop, std::string key, std::vector<std::string> & list)
{
    prop.addGroup(key);
    yarp::os::Bottle & bot = prop.findGroup(key).addList();
    for(size_t i=0; i < list.size(); i++)
    {
        bot.addString(list[i].c_str());
    }
    return;
}

bool getUsedDOFsList(os::Searchable& config, std::vector<std::string> & usedDOFs)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    yarp::os::Bottle *propAxesNames=prop.find("axesNames").asList();
    if(propAxesNames==0)
    {
       yError() <<"WholeBodyDynamicsDevice: Error parsing parameters: \"axesNames\" should be followed by a list\n";
       return false;
    }

    usedDOFs.resize(propAxesNames->size());
    for(int ax=0; ax < propAxesNames->size(); ax++)
    {
        usedDOFs[ax] = propAxesNames->get(ax).asString().c_str();
    }

    return true;
}

bool getGravityCompensationDOFsList(os::Searchable& config, std::vector<std::string> & gravityCompensationDOFs)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    yarp::os::Bottle *propAxesNames=prop.find("gravityCompensationAxesNames").asList();
    if(propAxesNames==0)
    {
       yError() <<"wholeBodyDynamics : Error parsing parameters: \"gravityCompensationAxesNames\" should be followed by a list\n";
       return false;
    }

    gravityCompensationDOFs.resize(propAxesNames->size());
    for(int ax=0; ax < propAxesNames->size(); ax++)
    {
        gravityCompensationDOFs[ax] = propAxesNames->get(ax).asString().c_str();
    }

    return true;
}


bool WholeBodyDynamicsDevice::openRemapperControlBoard(os::Searchable& config)
{
    // Pass to the remapper just the relevant parameters (axesList)
    yarp::os::Property propRemapper;
    propRemapper.put("device","controlboardremapper");
    bool ok = getUsedDOFsList(config,estimationJointNames);
    if(!ok) return false;

    addVectorOfStringToProperty(propRemapper,"axesNames",estimationJointNames);

    ok = remappedControlBoard.open(propRemapper);

    if( !ok )
    {
        return ok;
    }

    // View relevant interfaces for the remappedControlBoard
    ok = ok && remappedControlBoard.view(remappedControlBoardInterfaces.encs);
    ok = ok && remappedControlBoard.view(remappedControlBoardInterfaces.multwrap);
    ok = ok && remappedControlBoard.view(remappedControlBoardInterfaces.impctrl);
    ok = ok && remappedControlBoard.view(remappedControlBoardInterfaces.ctrlmode);
    ok = ok && remappedControlBoard.view(remappedControlBoardInterfaces.intmode);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : open impossible to use the necessary interfaces in remappedControlBoard";
        return ok;
    }

    // Check if the controlboard and the estimator have a consistent number of joints
    int axes = 0;
    remappedControlBoardInterfaces.encs->getAxes(&axes);
    if( axes != (int) estimator.model().getNrOfDOFs() )
    {
        yError() << "wholeBodyDynamics : open estimator model and the remappedControlBoard has an inconsistent number of joints";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::openRemapperVirtualSensors(os::Searchable& config)
{
    // Pass to the remapper just the relevant parameters (axesList)
    yarp::os::Property propRemapper;
    propRemapper.put("device","virtualAnalogRemapper");
    bool ok = getUsedDOFsList(config,estimationJointNames);
    if(!ok) return false;

    addVectorOfStringToProperty(propRemapper,"axesNames",estimationJointNames);

    ok = remappedVirtualAnalogSensors.open(propRemapper);

    if( !ok )
    {
        return ok;
    }

    // View relevant interfaces for the remappedVirtualAnalogSensors
    ok = ok && remappedVirtualAnalogSensors.view(remappedVirtualAnalogSensorsInterfaces.ivirtsens);
    ok = ok && remappedVirtualAnalogSensors.view(remappedVirtualAnalogSensorsInterfaces.multwrap);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : open impossible to use the necessary interfaces in remappedControlBoard";
        return ok;
    }

    // Check if the controlboard and the estimator have a consistent number of joints
    int axes = remappedVirtualAnalogSensorsInterfaces.ivirtsens->getVirtualAnalogSensorChannels();
    if( axes != (int) estimator.model().getNrOfDOFs() )
    {
        yError() << "wholeBodyDynamics : open estimator model and the remapperVirtualAnalogSensor has an inconsistent number of joints";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::openEstimator(os::Searchable& config)
{
    // get the list of considered dofs from config
    bool ok = getUsedDOFsList(config,estimationJointNames);
    if(!ok) return false;

    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    std::string modelFileName = "model.urdf";
    if( config.check("modelFile") && config.find("modelFile").isString() )
    {
        modelFileName = config.find("modelFile").asString();
    }

    std::string modelFileFullPath = rf.findFileByName(modelFileName);

    yInfo() << "wholeBodyDynamics : Loading model from " << modelFileFullPath;

    ok = estimator.loadModelAndSensorsFromFileWithSpecifiedDOFs(modelFileFullPath,estimationJointNames);
    if( !ok )
    {
        yInfo() << "wholeBodyDynamics : impossible to create ExtWrenchesAndJointTorquesEstimator from file "
                 << modelFileName << " ( full path: " << modelFileFullPath << " ) ";
        return false;
    }

    if( estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) == 0 )
    {
        yWarning() << "wholeBodyDynamics : the loaded model has 0 FT sensors, so the estimation will use just the model.";
        yWarning() << "wholeBodyDynamics : If you instead want to add the FT sensors to your model, please check iDynTree documentation on how to add sensors to models.";
    }

    this->resizeBuffers();

    return true;
}

bool WholeBodyDynamicsDevice::openDefaultContactFrames(os::Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    yarp::os::Bottle *propContactFrames=prop.find("defaultContactFrames").asList();
    if(propContactFrames==0)
    {
       yError() <<"wholeBodyDynamics : Error parsing parameters: \"defaultContactFrames\" should be followed by a list\n";
       return false;
    }

    defaultContactFrames.resize(propContactFrames->size());
    for(int ax=0; ax < propContactFrames->size(); ax++)
    {
        defaultContactFrames[ax] = propContactFrames->get(ax).asString().c_str();
    }

    // We build the defaultContactFramesIdx vector
    std::vector<iDynTree::FrameIndex> defaultContactFramesIdx;
    defaultContactFramesIdx.clear();
    for(size_t i=0; i < defaultContactFrames.size(); i++)
    {
        iDynTree::FrameIndex idx = estimator.model().getFrameIndex(defaultContactFrames[i]);

        if( idx == iDynTree::FRAME_INVALID_INDEX )
        {
            yWarning() << "Frame " << defaultContactFrames[i] << " not found in the model, discarding it";
        }
        else
        {
            defaultContactFramesIdx.push_back(idx);
        }
    }

    // For each submodel, we find the first suitable contact frame
    // This is a n^2 algorithm, but given that is just used in
    // configuration it should be ok
    size_t nrOfSubModels = estimator.submodels().getNrOfSubModels();

    // We indicate with FRAME_INVALID_INDEX the fact that we still don't have a default contact for the given submodel
    subModelIndex2DefaultContact.resize(nrOfSubModels,iDynTree::FRAME_INVALID_INDEX);

    for(size_t i=0; i < defaultContactFramesIdx.size(); i++)
    {
        size_t subModelIdx = estimator.submodels().getSubModelOfFrame(estimator.model(),defaultContactFramesIdx[i]);

        // If the subModel of the frame still does not have a default contact, we add it
        if( subModelIndex2DefaultContact[subModelIdx] == iDynTree::FRAME_INVALID_INDEX )
        {
            subModelIndex2DefaultContact[subModelIdx] = defaultContactFramesIdx[i];
        }
    }


    // Let's check that every submodel has a default contact position
    for(size_t subModelIdx = 0; subModelIdx < nrOfSubModels; subModelIdx++)
    {
        if( subModelIndex2DefaultContact[subModelIdx] == iDynTree::FRAME_INVALID_INDEX )
        {
            yError() << "wholeBodyDynamics : openDefaultContactFrames : missing default contact for submodel composed by the links: ";
            const iDynTree::Traversal & subModelTraversal = estimator.submodels().getTraversal(subModelIdx);
            for(iDynTree::TraversalIndex i = 0; i < (iDynTree::TraversalIndex) subModelTraversal.getNrOfVisitedLinks(); i++)
            {
                iDynTree::LinkIndex linkIdx = subModelTraversal.getLink(i)->getIndex();
                yError() << "wholeBodyDynamics : openDefaultContactFrames :" << estimator.model().getLinkName(linkIdx);
            }

            return false;
        }
    }

    return true;
}

bool WholeBodyDynamicsDevice::openSkinContactListPorts(os::Searchable& config)
{
    bool ok = this->portContactsInput.open(portPrefix+"/skin_contacts:i");
    if(!ok) return ok;

    ok = this->portContactsOutput.open(portPrefix+"/contacts:o");
    if(!ok) return ok;

    // Configure the conversion helper to read/publish data on this ports
    yarp::os::Bottle & bot = config.findGroup("IDYNTREE_SKINDYNLIB_LINKS");
    for(int i=1; i < bot.size(); i++ )
    {
        yarp::os::Bottle * map_bot = bot.get(i).asList();
        if( map_bot->size() != 2 || map_bot->get(1).asList() == NULL ||
            map_bot->get(1).asList()->size() != 3 )
        {
            yError() << "WholeBodyDynamicsDevice: IDYNTREE_SKINDYNLIB_LINKS group is malformed (" << map_bot->toString() << ")";
            return false;
        }

        std::string iDynTree_link_name = map_bot->get(0).asString();
        std::string iDynTree_skinFrame_name = map_bot->get(1).asList()->get(0).asString();
        int skinDynLib_body_part = map_bot->get(1).asList()->get(1).asInt();
        int skinDynLib_link_index = map_bot->get(1).asList()->get(2).asInt();

        bool ret_sdl = conversionHelper.addSkinDynLibAlias(estimator.model(),
                                                           iDynTree_link_name,iDynTree_skinFrame_name,
                                                           skinDynLib_body_part,skinDynLib_link_index);

        if( !ret_sdl )
        {
            yError() << "WholeBodyDynamicsDevice: IDYNTREE_SKINDYNLIB_LINKS link " << iDynTree_link_name
                      << " and frame " << iDynTree_skinFrame_name << " and not found in urdf model";
            return false;
        }
    }

    return ok;
}

bool WholeBodyDynamicsDevice::openExternalWrenchesPorts(os::Searchable& config)
{
    // Read ports info from config
    // Load output external wrenches ports informations
    yarp::os::Bottle & output_wrench_bot = config.findGroup("WBD_OUTPUT_EXTERNAL_WRENCH_PORTS");
    if( output_wrench_bot.isNull() )
    {
        // The WBD_OUTPUT_EXTERNAL_WRENCH_PORTS is optional
        return true;
    }

    for(int output_wrench_port = 1; output_wrench_port < output_wrench_bot.size(); output_wrench_port++)
    {
        outputWrenchPortInformation wrench_port_struct;
        yarp::os::Bottle *wrench_port = output_wrench_bot.get(output_wrench_port).asList();
        if( wrench_port == NULL || wrench_port->isNull() || wrench_port->size() != 2
            || wrench_port->get(1).asList() == NULL
            || !(wrench_port->get(1).asList()->size() == 2 || wrench_port->get(1).asList()->size() == 3 ) )
        {
            yError() << "wholeBodyDynamics : malformed WBD_OUTPUT_EXTERNAL_WRENCH_PORTS group found in configuration, exiting";
            if( wrench_port )
            {
                yError() << "wholeBodyDynamics : malformed line " << wrench_port->toString();
            }
            else
            {
                yError() << "wholeBodyDynamics : malformed line " << output_wrench_bot.get(output_wrench_port).toString();
                yError() << "wholeBodyDynamics : malformed group " << output_wrench_bot.toString();
            }
            return false;
        }

        wrench_port_struct.port_name = wrench_port->get(0).asString();
        wrench_port_struct.link = wrench_port->get(1).asList()->get(0).asString();

        if( wrench_port->get(1).asList()->size() == 2 )
        {
            // Simple configuration, both the origin and the orientation of the
            // force belong to the same frame
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.origin_frame = wrench_port_struct.orientation_frame;
        }
        else
        {
            assert( wrench_port->get(1).asList()->size() == 3 );
            // Complex configuration: the first parameter is the frame of the point of expression,
            // the second parameter is the frame of orientation
            wrench_port_struct.origin_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(2).asString();
        }

        outputWrenchPorts.push_back(wrench_port_struct);

    }

    // Load indeces for specified links and frame
    for(unsigned i=0; i < outputWrenchPorts.size(); i++ )
    {
        outputWrenchPorts[i].link_index =
            kinDynComp.getRobotModel().getLinkIndex(outputWrenchPorts[i].link);
        if( outputWrenchPorts[i].link_index < 0 )
        {
            yError() << "wholeBodyDynamics : Link " << outputWrenchPorts[i].link << " not found in the model.";
            return false;
        }

        outputWrenchPorts[i].origin_frame_index =
            kinDynComp.getRobotModel().getFrameIndex(outputWrenchPorts[i].origin_frame);

        if( outputWrenchPorts[i].origin_frame_index < 0 )
        {
            yError() << "wholeBodyDynamics : Frame " << outputWrenchPorts[i].origin_frame << " not found in the model.";
            return false;
        }

        outputWrenchPorts[i].orientation_frame_index =
            kinDynComp.getRobotModel().getFrameIndex(outputWrenchPorts[i].orientation_frame);

        if( outputWrenchPorts[i].orientation_frame_index < 0 )
        {
            yError() << "wholeBodyDynamics : Frame " << outputWrenchPorts[i].orientation_frame_index << " not found in the model.";
            return false;
        }
    }

    // Open ports
    bool ok = true;
    for(unsigned int i = 0; i < outputWrenchPorts.size(); i++ )
    {
        std::string port_name = outputWrenchPorts[i].port_name;
        outputWrenchPorts[i].output_port = new yarp::os::BufferedPort<yarp::sig::Vector>;
        ok = ok && outputWrenchPorts[i].output_port->open(port_name);
        outputWrenchPorts[i].output_vector.resize(wholeBodyDynamics_nrOfChannelsOfYARPFTSensor);
    }

    if( !ok )
    {
        yError() << "wholeBodyDynamics impossible to open port for publishing external wrenches";
        return false;
    }

    return ok;
}

bool WholeBodyDynamicsDevice::openFilteredFTPorts(os::Searchable& config)
{
    bool ok = true;
    // create port names

    std::string sensorName;
    std::string portName;
    outputFTPorts.resize(estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE));
    for(int ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        sensorName= estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName() ;

        yInfo() << "wholeBodyDynamics: creating port name and opening port for  " << sensorName;

        portName=(portPrefix+"/filteredFT/"+sensorName);
        outputFTPorts[ft]=std::make_unique<yarp::os::BufferedPort<yarp::sig::Vector> >();
        ok = ok && outputFTPorts[ft]->open(portName);
    }

    if( !ok )
    {
        yError() << "wholeBodyDynamics impossible to open port for publishing filtered ft wrenches";
        return false;
    }
    return ok;
}


void WholeBodyDynamicsDevice::resizeBuffers()
{
    this->jointPos.resize(estimator.model());
    this->jointVel.resize(estimator.model());
    this->jointAcc.resize(estimator.model());
    this->measuredContactLocations.resize(estimator.model());
    this->ftMeasurement.resize(wholeBodyDynamics_nrOfChannelsOfYARPFTSensor);
    this->imuMeasurement.resize(wholeBodyDynamics_nrOfChannelsOfAYARPIMUSensor);
    this->rawSensorsMeasurements.resize(estimator.sensors());
    this->filteredSensorMeasurements.resize(estimator.sensors());
    this->estimatedJointTorques.resize(estimator.model());
    this->estimatedJointTorquesYARP.resize(this->estimatedJointTorques.size(),0.0);
    this->estimateExternalContactWrenches.resize(estimator.model());

    // Resize F/T stuff
    size_t nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    calibrationBuffers.calibratingFTsensor.resize(nrOfFTSensors,false);
    iDynTree::Wrench zeroWrench;
    zeroWrench.zero();
    calibrationBuffers.offsetSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
    calibrationBuffers.measurementSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
    calibrationBuffers.estimationSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
    calibrationBuffers.assumedContactLocationsForCalibration.resize(estimator.model());
    calibrationBuffers.predictedExternalContactWrenchesForCalibration.resize(estimator.model());
    calibrationBuffers.predictedJointTorquesForCalibration.resize(estimator.model());
    calibrationBuffers.predictedSensorMeasurementsForCalibration.resize(estimator.sensors());

    ftProcessors.resize(nrOfFTSensors);

    // Resize filters
    filters.init(nrOfFTSensors,
                 settings.forceTorqueFilterCutoffInHz,
                 settings.imuFilterCutoffInHz,
                 estimator.model().getNrOfDOFs(),
                 settings.jointVelFilterCutoffInHz,
                 settings.jointAccFilterCutoffInHz,
                 getRate()/1000.0);

    // Resize external wrenches publishing software
    this->netExternalWrenchesExertedByTheEnviroment.resize(estimator.model());
    bool ok = this->kinDynComp.loadRobotModel(estimator.model());

    if( !ok )
    {
        yError() << "wholeBodyDynamics : error in opening KinDynComputation class";
    }
}


bool WholeBodyDynamicsDevice::loadSettingsFromConfig(os::Searchable& config)
{
    // Fill setting with their default values
    settings.kinematicSource             = IMU;
    settings.imuFilterCutoffInHz         = 3.0;
    settings.forceTorqueFilterCutoffInHz = 3.0;
    settings.jointVelFilterCutoffInHz    = 3.0;
    settings.jointAccFilterCutoffInHz    = 3.0;

    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    // Check the assumeFixed parameter
    if( prop.check("assume_fixed") )
    {
        if( ! prop.find("assume_fixed").isString() )
        {
            yError() << "wholeBodyDynamics : assume_fixed is present, but it is not a string";
            return false;
        }

        std::string fixedFrameName = prop.find("assume_fixed").asString();

        iDynTree::FrameIndex fixedFrameIndex = estimator.model().getFrameIndex(fixedFrameName);

        if( fixedFrameIndex == iDynTree::FRAME_INVALID_INDEX )
        {
            yError() << "wholeBodyDynamics : assume_fixed is present, but " << fixedFrameName << " is not a frame in the model";
            return false;
        }

        // Add a hardcoded warning, ugly but I think that in the short term is useful
        if( fixedFrameName != "root_link" &&
            fixedFrameName != "l_sole" &&
            fixedFrameName != "r_sole" )
        {
            yWarning() << "wholeBodyDynamics : assume_fixed is set to " << fixedFrameName << " that is not root_link, l_sole or r_sole, so pay attention to correctly set the gravity vector";
        }

        settings.kinematicSource = FIXED_FRAME;
        settings.fixedFrameName = fixedFrameName;
    }

    // Check for the imu frame
    if( prop.check("imuFrameName") &&
        prop.find("imuFrameName").isString() )
    {
        settings.imuFrameName = prop.find("imuFrameName").asString();
    }
    else
    {
        yError() << "wholeBodyDynamics : missing required string parameter imuFrameName";
        return false;
    }

    // fixedFrameGravity is always required even if you
    // use the IMU because the estimation could switch in use a fixed frame
    // via RPC, so we should have a valid gravity to use
    if( prop.check("fixedFrameGravity") &&
        prop.find("fixedFrameGravity").isList() &&
        prop.find("fixedFrameGravity").asList()->size() == 3 )
    {
        settings.fixedFrameGravity.x = prop.find("fixedFrameGravity").asList()->get(0).asDouble();
        settings.fixedFrameGravity.y = prop.find("fixedFrameGravity").asList()->get(1).asDouble();
        settings.fixedFrameGravity.z = prop.find("fixedFrameGravity").asList()->get(2).asDouble();
    }
    else
    {
        yError() << "wholeBodyDynamics : missing required parameter fixedFrameGravity";
        return false;
    }

    // Set the port prefix. The default value "/wholeBodyDynamics"
    // is set in the device constructor
    if( prop.check("portPrefix") &&
        prop.find("portPrefix").isString())
    {
        portPrefix = prop.find("portPrefix").asString();
    }

    // Enable or disable ft streaming. The default value is false;
    // is set in the device constructor
    if( prop.check("streamFilteredFT") &&
        prop.find("streamFilteredFT").isBool())
    {
        streamFilteredFT = prop.find("streamFilteredFT").asBool();
    }

    std::string useJointVelocityOptionName = "useJointVelocity";
    if( !(prop.check(useJointVelocityOptionName.c_str()) && prop.find(useJointVelocityOptionName.c_str()).isBool()) )
    {
        yWarning() << "wholeBodyDynamics: useJointVelocity bool parameter missing, please specify it.";
        yWarning() << "wholeBodyDynamics: setting useJointVelocity to the default value of true, but this is a deprecated behaviour that will be removed in the future.";
        settings.useJointVelocity = true;
    }
    else
    {
        settings.useJointVelocity = prop.find(useJointVelocityOptionName.c_str()).asBool();
    }

    std::string useJointAccelerationOptionName = "useJointAcceleration";
    if( !(prop.check(useJointAccelerationOptionName.c_str()) && prop.find(useJointAccelerationOptionName.c_str()).isBool()) )
    {
        yWarning() << "wholeBodyDynamics: useJointAcceleration bool parameter missing, please specify it.";
        yWarning() << "wholeBodyDynamics: setting useJointAcceleration to the default value of true, but this is a deprecated behaviour that will be removed in the future.";
        settings.useJointAcceleration = true;
    }
    else
    {
        settings.useJointAcceleration = prop.find(useJointAccelerationOptionName.c_str()).asBool();
    }

    return true;
}

bool WholeBodyDynamicsDevice::loadSecondaryCalibrationSettingsFromConfig(os::Searchable& config)
{
   bool ret;
   yarp::os::Property propAll;
   propAll.fromString(config.toString().c_str());

    if( !propAll.check("FT_SECONDARY_CALIBRATION") )
    {
        ret = true;
    }
    else
    {
        yarp::os::Bottle & propSecondCalib = propAll.findGroup("FT_SECONDARY_CALIBRATION");
        for(int i=1; i < propSecondCalib.size(); i++ )
        {
            yarp::os::Bottle * map_bot = propSecondCalib.get(i).asList();
            if( map_bot->size() != 2 || map_bot->get(1).asList() == NULL ||
                map_bot->get(1).asList()->size() != 36 )
            {
                yError() << "wholeBodyDynamics: FT_SECONDARY_CALIBRATION group is malformed (" << map_bot->toString() << "). ";
                return false;
            }

            std::string iDynTree_sensorName = map_bot->get(0).asString();
            iDynTree::Matrix6x6 secondaryCalibMat;

            for(int r=0; r < 6; r++)
            {
                for(int c=0; c < 6; c++)
                {
                    int rowMajorIndex = 6*r+c;
                    secondaryCalibMat(r,c) = map_bot->get(1).asList()->get(rowMajorIndex).asDouble();
                }
            }

            // Linearly search for the specified sensor
            bool sensorFound = false;
            for(int ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
            {
                if( estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName() == iDynTree_sensorName )
                {
                    yDebug() << "wholeBodyDynamics: using secondary calibration matrix for sensor " << iDynTree_sensorName;

                    ftProcessors[ft].secondaryCalibrationMatrix() = secondaryCalibMat;
                    sensorFound = true;
                }
            }

            // If a specified sensor was not found, give an error
            if( !sensorFound )
            {
                yError() << "wholeBodyDynamics: secondary calibration matrix specified for FT sensor " << iDynTree_sensorName
                          << " but no sensor with that name found in the model";
                return false;
            }
        }
        ret = true;
    }

    return ret;

}


bool WholeBodyDynamicsDevice::loadGravityCompensationSettingsFromConfig(os::Searchable& config)
{
    bool ret = true;
    yarp::os::Property propAll;
    propAll.fromString(config.toString().c_str());

    if( !propAll.check("GRAVITY_COMPENSATION") )
    {
        yWarning() << "wholeBodyDynamics: GRAVITY_COMPENSATION group not found,  disabling gravity compensation support";
        m_gravityCompensationEnabled = false;
        m_gravityCompesationJoints.resize(0);
        ret = true;
    }
    else
    {
        yarp::os::Searchable & propGravComp = propAll.findGroup("GRAVITY_COMPENSATION");

        if( !(propGravComp.check("enableGravityCompensation") && propGravComp.find("enableGravityCompensation").isBool()) )
        {
            yError() << "wholeBodyDynamics: GRAVITY_COMPENSATION group found, but enableGravityCompensation bool parameter missing";
            return false;
        }

        if( !(propGravComp.check("gravityCompensationBaseLink") && propGravComp.find("gravityCompensationBaseLink").isString()) )
        {
            yError() << "wholeBodyDynamics: GRAVITY_COMPENSATION group found, but gravityCompensationBaseLink string parameter missing";
            return false;
        }

        if( !(propGravComp.check("gravityCompensationAxesNames") && propGravComp.find("gravityCompensationAxesNames").isList()) )
        {
            yError() << "wholeBodyDynamics: GRAVITY_COMPENSATION group found, but gravityCompensationAxesNames list parameter missing";
            return false;
        }

        m_gravityCompensationEnabled = propGravComp.find("enableGravityCompensation").asBool();

        std::vector<std::string> gravityCompesationAxes;

        ret = getGravityCompensationDOFsList(propGravComp,gravityCompesationAxes);

        if( !ret) return false;

        m_gravityCompesationJoints.resize(0);
        for(size_t i=0; i < gravityCompesationAxes.size(); i++)
        {
            iDynTree::JointIndex dofGravityJointIndex = this->kinDynComp.getRobotModel().getJointIndex(gravityCompesationAxes[i]);

            if( !(this->kinDynComp.getRobotModel().isValidJointIndex(dofGravityJointIndex)) )
            {
                yError() << "wholeBodyDynamics: joint " << gravityCompesationAxes[i] << " passed as a gravityCompensationAxesNames not found in the model.";
                return false;
            }

            if( this->kinDynComp.getRobotModel().getJoint(dofGravityJointIndex)->getNrOfDOFs() != 1 )
            {
                yError() << "wholeBodyDynamics: joint " << gravityCompesationAxes[i] << " passed as a gravityCompensationAxesNames is not a 1 dof joint.";
                return false;
            }

            size_t dofOffset = this->kinDynComp.getRobotModel().getJoint(dofGravityJointIndex)->getDOFsOffset();

            m_gravityCompesationJoints.push_back(dofOffset);
        }

        // We use the kinDynComp class that was opened together with the estimator
        std::string gravityCompensationBaseLink = propGravComp.find("gravityCompensationBaseLink").asString().c_str();

        ret = m_gravCompHelper.loadModel(this->estimator.model(),gravityCompensationBaseLink);
        m_gravityCompensationTorques.resize(this->estimator.model());

        if( !ret )
        {
            yError() << "wholeBodyDynamics: link " << gravityCompensationBaseLink << " passed as gravityCompensationBaseLink not found in the model.";
            return false;
        }
    }

    return ret;
}


bool WholeBodyDynamicsDevice::open(os::Searchable& config)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    bool ok;

    // Load settings in the class
    ok = this->loadSettingsFromConfig(config);
   if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in loading settings from config.";
        return false;
    }

    // Create the estimator
    ok = this->openEstimator(config);
     if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening estimator object.";
        return false;
    } 

    // Open settings related to gravity compensation (we need the estimator to be open)
    ok = this->loadGravityCompensationSettingsFromConfig(config);
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening gravity compensator settings.";
        return false;
    } 

    // Open settings related to gravity compensation (we need the estimator to be open)
    ok = this->loadSecondaryCalibrationSettingsFromConfig(config);
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in loading secondary calibration matrix settings.";
        return false;
    } 

    // Open rpc port
    ok = this->openRPCPort();
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening rpc port.";
        return false;
    } 

    // Open settings port
    ok = this->openSettingsPort();
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening settings port.";
        return false;
    } 

    // Open the controlboard remapper
    ok = this->openRemapperControlBoard(config);
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening controlboard remapper.";
        return false;
    } 

     // Open the virtualsensor remapper
    ok = this->openRemapperVirtualSensors(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening virtual analog sensors remapper.";
        return false;
    } 

    ok = this->openDefaultContactFrames(config);
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening default contact frame settings.";
        return false;
    } 

    // Open the skin-related ports
    ok = this->openSkinContactListPorts(config);
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening skin-related port.";
        return false;
    } 

    // Open the ports related to publishing external wrenches
    ok = this->openExternalWrenchesPorts(config);
    if( !ok ) 
    {
        yError() << "wholeBodyDynamics: Problem in opening external wrenches port.";
        return false;
    } 

    // Open the ports related to publishing filtered ft wrenches
    if (streamFilteredFT){
        ok = this->openFilteredFTPorts(config);
        if( !ok )
        {
            yError() << "wholeBodyDynamics: Problem in opening filtered ft ports.";
            return false;
        }
    }

    return true;
}

bool WholeBodyDynamicsDevice::attachAllControlBoard(const PolyDriverList& p)
{
    PolyDriverList controlBoardList;
    for(size_t devIdx = 0; devIdx < (size_t) p.size(); devIdx++)
    {
        IEncoders * pEncs = 0;
        if( p[devIdx]->poly->view(pEncs) )
        {
            controlBoardList.push(const_cast<PolyDriverDescriptor&>(*p[devIdx]));
        }
    }

    // Attach the controlBoardList to the controlBoardRemapper
    bool ok = remappedControlBoardInterfaces.multwrap->attachAll(controlBoardList);

    if( !ok )
    {
        yError() << " WholeBodyDynamicsDevice::attachAll in attachAll of the remappedControlBoard";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::attachAllVirtualAnalogSensor(const PolyDriverList& p)
{
    bool ok = remappedVirtualAnalogSensorsInterfaces.multwrap->attachAll(p);

    if( !ok )
    {
        yError() << " WholeBodyDynamicsDevice::attachAll: error in attachAll of the remapperVirtualAnalogSensor";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::attachAllFTs(const PolyDriverList& p)
{
    std::vector<IAnalogSensor *> ftList;
    std::vector<std::string>     ftDeviceNames;
    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
    {
        // A device is considered an ft if it implements IAnalogSensor and has 6 channels
        IAnalogSensor * pAnalogSens = 0;
        if( p[devIdx]->poly->view(pAnalogSens) )
        {
            if( pAnalogSens->getChannels() == (int)wholeBodyDynamics_nrOfChannelsOfYARPFTSensor )
            {
                ftList.push_back(pAnalogSens);
                ftDeviceNames.push_back(p[devIdx]->key);
            }
        }
    }

    if( ftList.size() != estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) )
    {
        yError() << "wholeBodyDynamicsDevice : was expecting "
                 << estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)
                 << " from the model, but got " << ftList.size() << " FT sensor in the attach list.";
        return false;
    }

    // For now we assume that the name of the F/T sensor device match the sensor name in the URDF
    // In the future we could use a new fancy sensor interface
    ftSensors.resize(ftList.size());
    for(size_t IDTsensIdx=0; IDTsensIdx < ftSensors.size(); IDTsensIdx++)
    {
        std::string sensorName = estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,IDTsensIdx)->getName();

        // Search for a suitable device
        int deviceThatHasTheSameNameOfTheSensor = -1;
        for(size_t deviceIdx = 0; deviceIdx < ftList.size(); deviceIdx++)
        {
            if( ftDeviceNames[deviceIdx] == sensorName )
            {
                deviceThatHasTheSameNameOfTheSensor = deviceIdx;
            }
        }

        if( deviceThatHasTheSameNameOfTheSensor == -1 )
        {
            yError() << "WholeBodyDynamicsDevice was expecting a sensor named " << sensorName << " but it did not find one in the attached devices";
            return false;
        }

        ftSensors[IDTsensIdx] = ftList[deviceThatHasTheSameNameOfTheSensor];
    }

    // We try to read for a brief moment the sensors for two reasons:
    // so we can make sure that they actually work, and to make sure that the buffers are correctly initialized
    bool verbose = false;
    double tic = yarp::os::Time::now();
    bool timeSpentTryngToReadSensors = 0.0;
    bool readSuccessfull = false;
    while( (timeSpentTryngToReadSensors < wholeBodyDynamics_sensorTimeoutInSeconds) && !readSuccessfull )
    {
        readSuccessfull = readFTSensors(verbose);
        timeSpentTryngToReadSensors = (yarp::os::Time::now() - tic);
    }

    if( !readSuccessfull )
    {
       yError() << "WholeBodyDynamicsDevice was unable to correctly read from the FT sensors";
    }

    return readSuccessfull;
}


bool WholeBodyDynamicsDevice::attachAllIMUs(const PolyDriverList& p)
{
    std::vector<IGenericSensor*> imuList;

    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
    {
        IGenericSensor * pGenericSensor = 0;
        if( p[devIdx]->poly->view(pGenericSensor) )
        {
            imuList.push_back(pGenericSensor);
        }
    }

    size_t nrOfIMUDetected = imuList.size();

    if( nrOfIMUDetected != 1 )
    {
        yError() << "WholeBodyDynamicsDevice was expecting only one IMU, but it did not find " << nrOfIMUDetected << " in the attached devices";
        return false;
    }

    if( imuList.size() == 1 )
    {
        this->imuInterface = imuList[0];
    }

    // We try to read for a brief moment the sensors for two reasons:
    // so we can make sure that they actually work, and to make sure that the buffers are correctly initialized
    bool verbose = false;
    double tic = yarp::os::Time::now();
    bool timeSpentTryngToReadSensors = 0.0;
    bool readSuccessfull = false;
    while( (timeSpentTryngToReadSensors < wholeBodyDynamics_sensorTimeoutInSeconds) && !readSuccessfull )
    {
        readSuccessfull = readIMUSensors(verbose);
        timeSpentTryngToReadSensors = (yarp::os::Time::now() - tic);
    }

    if( !readSuccessfull )
    {
       yError() << "WholeBodyDynamicsDevice was unable to correctly read from the IMU for " << wholeBodyDynamics_sensorTimeoutInSeconds << " seconds, exiting.";
    }

    return readSuccessfull;
}

bool WholeBodyDynamicsDevice::attachAll(const PolyDriverList& p)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    bool ok = true;
    ok = ok && this->attachAllControlBoard(p);
    ok = ok && this->attachAllVirtualAnalogSensor(p);
    ok = ok && this->attachAllFTs(p);
    ok = ok && this->attachAllIMUs(p);

    ok = ok && this->setupCalibrationWithExternalWrenchOnOneFrame("base_link",100);

    if( ok )
    {
        correctlyConfigured = true;
        this->start();
    }

    return ok;
}

double deg2rad(const double angleInDeg)
{
    return angleInDeg*M_PI/180.0;
}

void convertVectorFromDegreesToRadians(iDynTree::VectorDynSize & vector)
{
    for(size_t i=0; i < vector.size(); i++)
    {
        vector(i) = deg2rad(vector(i));
    }

    return;
}

bool WholeBodyDynamicsDevice::readFTSensors(bool verbose)
{
    bool FTSensorsReadCorrectly = true;
    for(size_t ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::Wrench bufWrench;
        int ftRetVal = ftSensors[ft]->read(ftMeasurement);

        bool ok = (ftRetVal == IAnalogSensor::AS_OK);

        FTSensorsReadCorrectly = FTSensorsReadCorrectly && ok;

        if( !ok && verbose )
        {
            std::string sensorName = estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName();
            yWarning() << "wholeBodyDynamics warning : FT sensor " << sensorName << " was not readed correctly, using old measurement";
        }

        bool isNaN = false;
        for (size_t i = 0; i < ftMeasurement.size(); i++)
        {
            if( std::isnan(ftMeasurement[i]) )
            {
                isNaN = true;
                break;
            }
        }
        if( isNaN )
        {
            std::string sensorName = estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName();
            yError() << "wholeBodyDynamics : FT sensor " << sensorName << " contains nan: " << ftMeasurement.toString() << ", returning error.";
            return false;
        }

        if( ok )
        {
            // Format of F/T measurement in YARP/iDynTree is consistent: linear/angular
            iDynTree::toiDynTree(ftMeasurement,bufWrench);

            rawSensorsMeasurements.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,bufWrench);
        }
    }

    return FTSensorsReadCorrectly;
}

bool WholeBodyDynamicsDevice::readIMUSensors(bool verbose)
{
    rawIMUMeasurements.angularAcc.zero();
    rawIMUMeasurements.linProperAcc.zero();
    rawIMUMeasurements.angularVel.zero();

    bool ok = imuInterface->read(imuMeasurement);

    if( !ok && verbose )
    {
        yWarning() << "wholeBodyDynamics warning : imu sensor was not readed correctly, using old measurement";
    }

    if( ok )
    {
        // Check format of IMU in YARP http://wiki.icub.org/wiki/Inertial_Sensor
        rawIMUMeasurements.angularVel(0) = deg2rad(imuMeasurement[6]);
        rawIMUMeasurements.angularVel(1) = deg2rad(imuMeasurement[7]);
        rawIMUMeasurements.angularVel(2) = deg2rad(imuMeasurement[8]);

        rawIMUMeasurements.linProperAcc(0) = imuMeasurement[3];
        rawIMUMeasurements.linProperAcc(1) = imuMeasurement[4];
        rawIMUMeasurements.linProperAcc(2) = imuMeasurement[5];
    }

    return ok;
}


void WholeBodyDynamicsDevice::readSensors()
{
    // Read encoders
    sensorReadCorrectly = remappedControlBoardInterfaces.encs->getEncoders(jointPos.data());

    // Convert from degrees (used on wire by YARP) to radians (used by iDynTree)
    convertVectorFromDegreesToRadians(jointPos);

    bool ok;

    if( !sensorReadCorrectly )
    {
        yWarning() << "wholeBodyDynamics warning : joint positions was not readed correctly";
    }

    // At the moment we are assuming that all joints are revolute

    if( settings.useJointVelocity )
    {
        ok = remappedControlBoardInterfaces.encs->getEncoderSpeeds(jointVel.data());
        sensorReadCorrectly = sensorReadCorrectly && ok;
        if( !ok )
        {
            yWarning() << "wholeBodyDynamics warning : joint velocities was not readed correctly";
        }

        // Convert from degrees (used on wire by YARP) to radians (used by iDynTree)
        convertVectorFromDegreesToRadians(jointVel);
    }
    else
    {
        jointVel.zero();
    }

    if( settings.useJointAcceleration )
    {
        ok = remappedControlBoardInterfaces.encs->getEncoderAccelerations(jointAcc.data());
        sensorReadCorrectly = sensorReadCorrectly && ok;
        if( !ok )
        {
            yWarning() << "wholeBodyDynamics warning : joint accelerations was not readed correctly";
        }

        // Convert from degrees (used on wire by YARP) to radians (used by iDynTree)
        convertVectorFromDegreesToRadians(jointAcc);

    }
    else
    {
        jointAcc.zero();
    }

    // Read F/T sensors
    ok = readFTSensors();
    sensorReadCorrectly = ok && sensorReadCorrectly;

    // Read IMU Sensor
    if( settings.kinematicSource == IMU )
    {
        ok = readIMUSensors();
        sensorReadCorrectly = ok && sensorReadCorrectly;
    }


}

void WholeBodyDynamicsDevice::filterSensorsAndRemoveSensorOffsets()
{
    filters.updateCutOffFrequency(settings.forceTorqueFilterCutoffInHz,
                                  settings.imuFilterCutoffInHz,
                                  settings.jointVelFilterCutoffInHz,
                                  settings.jointAccFilterCutoffInHz);

    // Filter and remove offset fromn F/T sensors
    for(size_t ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::Wrench rawFTMeasure;
        rawSensorsMeasurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,rawFTMeasure);

        iDynTree::Wrench rawFTMeasureWithOffsetRemoved  = ftProcessors[ft].filt(rawFTMeasure);

        // Filter the data
        iDynTree::toYarp(rawFTMeasureWithOffsetRemoved,filters.bufferYarp6);

        // Run the filter
        const yarp::sig::Vector & outputFt = filters.forcetorqueFilters[ft]->filt(filters.bufferYarp6);

        iDynTree::Wrench filteredFTMeasure;

        iDynTree::toiDynTree(outputFt,filteredFTMeasure);

        filteredSensorMeasurements.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,filteredFTMeasure);
    }

    // Filter joint vel
    if( settings.useJointVelocity )
    {
        iDynTree::toYarp(jointVel,filters.bufferYarpDofs);

        const yarp::sig::Vector & outputJointVel = filters.jntVelFilter->filt(filters.bufferYarpDofs);

        iDynTree::toiDynTree(outputJointVel,jointVel);
    }

    // Filter joint acc
    if( settings.useJointAcceleration )
    {
        iDynTree::toYarp(jointAcc,filters.bufferYarpDofs);

        const yarp::sig::Vector & outputJointAcc = filters.jntAccFilter->filt(filters.bufferYarpDofs);

        iDynTree::toiDynTree(outputJointAcc,jointVel);
    }

    // Filter IMU Sensor
    if( settings.kinematicSource == IMU )
    {
        iDynTree::toYarp(rawIMUMeasurements.linProperAcc,filters.bufferYarp3);

        const yarp::sig::Vector & outputLinAcc = filters.imuLinearAccelerationFilter->filt(filters.bufferYarp3);

        iDynTree::toiDynTree(outputLinAcc,filteredIMUMeasurements.linProperAcc);

        iDynTree::toYarp(rawIMUMeasurements.angularVel,filters.bufferYarp3);

        const yarp::sig::Vector & outputAngVel = filters.imuAngularVelocityFilter->filt(filters.bufferYarp3);

        iDynTree::toiDynTree(outputAngVel,filteredIMUMeasurements.angularVel);

        // For now we just assume that the angular acceleration is zero
        filteredIMUMeasurements.angularAcc.zero();
    }
}

void WholeBodyDynamicsDevice::updateKinematics()
{
    // Read IMU Sensor and update the kinematics in the model
    if( settings.kinematicSource == IMU )
    {
        // Hardcode for the meanwhile
        iDynTree::FrameIndex imuFrameIndex = estimator.model().getFrameIndex(settings.imuFrameName);

        estimator.updateKinematicsFromFloatingBase(jointPos,jointVel,jointAcc,imuFrameIndex,
                                                   filteredIMUMeasurements.linProperAcc,filteredIMUMeasurements.angularVel,filteredIMUMeasurements.angularAcc);

        if( m_gravityCompensationEnabled )
        {
            m_gravCompHelper.updateKinematicsFromProperAcceleration(jointPos,
                                                                    imuFrameIndex,
                                                                    filteredIMUMeasurements.linProperAcc);
        }
    }
    else
    {
        iDynTree::Vector3 gravity;

        // this should be valid because it was validated when set
        iDynTree::FrameIndex fixedFrameIndex = estimator.model().getFrameIndex(settings.fixedFrameName);

        gravity(0) = settings.fixedFrameGravity.x;
        gravity(1) = settings.fixedFrameGravity.y;
        gravity(2) = settings.fixedFrameGravity.z;

        estimator.updateKinematicsFromFixedBase(jointPos,jointVel,jointAcc,fixedFrameIndex,gravity);

        if( m_gravityCompensationEnabled )
        {
            m_gravCompHelper.updateKinematicsFromGravity(jointPos,
                                                         fixedFrameIndex,
                                                         gravity);
        }
    }
}


void WholeBodyDynamicsDevice::readContactPoints()
{
    // In this function the location of the external forces acting on the robot
    // are computed. The basic strategy is to assume a contact for each subtree in which the
    // robot is divided by the F/T sensors.

    measuredContactLocations.clear();
    int numberOfContacts=0;
    size_t nrOfSubModels = estimator.submodels().getNrOfSubModels();

    // read skin
    iCub::skinDynLib::skinContactList *scl =this->portContactsInput.read(false); //scl=null could also mean no new message
    if(scl)
    {
        //< \todo TODO check for envelope?
        lastReadingSkinContactListStamp = yarp::os::Time::now();
        if(scl->empty())   // if no skin contacts => leave the old contacts but reset pressure and contact list
        {
            if (contactsReadFromSkin.empty())
            {

                for(size_t subModel = 0; subModel < nrOfSubModels; subModel++)
                {
                    bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                            subModelIndex2DefaultContact[subModel], //frameIndex in iDynTree
                                                                            iDynTree::UnknownWrenchContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero()));
                    if( !ok )
                    {
                        yWarning() << "wholeBodyDynamics: Failing in adding default contact for submodel " << subModel;
                    }
                }
                return;
            }

            //< \todo TODO this (using the last contacts if no contacts are detected) should be at subtree level, not at global level??
            // ask what is intended with this TODO
            else
            {
                for(iCub::skinDynLib::skinContactList::iterator it=contactsReadFromSkin.begin(); it!=contactsReadFromSkin.end(); it++)
                {
                    it->setPressure(0.0);
                    it->setActiveTaxels(0);
                    numberOfContacts++;
                }
            }
        }
        else
        {
            contactsReadFromSkin.clear();
            for(iCub::skinDynLib::skinContactList::iterator it=scl->begin(); it!=scl->end(); it++)
            {
                //  less than 10 taxels are active then suppose zero moment
                if( it->getActiveTaxels()<10)
                {
                    it->fixMoment();                    
                }
                contactsReadFromSkin.insert(contactsReadFromSkin.end(),*it);
                numberOfContacts++;
            }
        }
    }
    else
    {
        if(yarp::os::Time::now()-lastReadingSkinContactListStamp>SKIN_EVENTS_TIMEOUT && lastReadingSkinContactListStamp!=0.0)
        {
            contactsReadFromSkin.clear();
        }
        // For now just put the default contact points
        //This logic only gives the location of the contacts but it does not store any value of pressure or wrench in the contact,
        if (contactsReadFromSkin.empty())
        {
            for(size_t subModel = 0; subModel < nrOfSubModels; subModel++)
            {
                bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                        subModelIndex2DefaultContact[subModel], //frameIndex in iDynTree
                                                                        iDynTree::UnknownWrenchContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero()));
                if( !ok )
                {
                    yWarning() << "wholeBodyDynamics: Failing in adding default contact for submodel " << subModel;
                }
            }
            return;
        }

        //< \todo TODO this (using the last contacts if no contacts are detected) should be at subtree level, not at global level??
        // ask what is intended with this TODO
        else
        {
            for(iCub::skinDynLib::skinContactList::iterator it=contactsReadFromSkin.begin(); it!=contactsReadFromSkin.end(); it++)
            {
                it->setPressure(0.0);
                it->setActiveTaxels(0);
                //yDebug() << "wholeBodyDynamics: skincontactlist empty, setting pressure and active taxels to 0";
                numberOfContacts++;
            }
        }

        //return;
    }

    // convert skinContactList into LinkUnknownWrenchContacts TODO: change function to keep and store wrench information only contact location and force directionis kept
    conversionHelper.fromSkinDynLibToiDynTree(estimator.model(),contactsReadFromSkin,measuredContactLocations);

    //declare and initialize contact count to 0
    std::vector<int> contacts_for_given_subModel(nrOfSubModels,0);

    numberOfContacts=0;
    int subModelIndex=0;
    // check each link to see if they have and assigned contact in which case check the subModelIndex
    for(size_t linkIndex = 0; linkIndex < estimator.model().getNrOfLinks(); linkIndex++)
    {
        numberOfContacts= measuredContactLocations.getNrOfContactsForLink(linkIndex);
        if( numberOfContacts >0)
        {
            subModelIndex = estimator.submodels().getSubModelOfLink(linkIndex);
            contacts_for_given_subModel[subModelIndex]++;
        }
    }

    for(size_t subModel = 0; subModel < nrOfSubModels; subModel++)
    {
        if( contacts_for_given_subModel[subModel] == 0 )
        {
            bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                    subModelIndex2DefaultContact[subModel], //frameIndex in iDynTree
                                                                    iDynTree::UnknownWrenchContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero()));
            if( !ok )
            {
                yWarning() << "wholeBodyDynamics: Failing in adding default contact for submodel " << subModel;
            }
        }
        /*else
        {
             yDebug() << "wholeBodyDynamics: number of contacts in submodel "<<subModel<<" = "<<contacts_for_given_subModel[subModel];
        }*/
    }

    return;
}

void addToSummer(iDynTree::Vector6 & buffer, const iDynTree::Wrench & addedWrench)
{
    for(size_t i=0; i < wholeBodyDynamics_nrOfChannelsOfYARPFTSensor; i++)
    {
        buffer(i) = buffer(i) + addedWrench(i);
    }
}

void computeMean(const iDynTree::Vector6 & buffer, const size_t nrOfSamples, iDynTree::Wrench & mean)
{
    for(size_t i=0; i < wholeBodyDynamics_nrOfChannelsOfYARPFTSensor; i++)
    {
        mean(i) = buffer(i)/nrOfSamples;
    }
}


void WholeBodyDynamicsDevice::computeCalibration()
{
    if( calibrationBuffers.ongoingCalibration )
    {
        // Todo: Check that the model is actually still during calibration

        // Run the calibration
        estimator.computeExpectedFTSensorsMeasurements(calibrationBuffers.assumedContactLocationsForCalibration,
                                                       calibrationBuffers.predictedSensorMeasurementsForCalibration,
                                                       calibrationBuffers.predictedExternalContactWrenchesForCalibration,
                                                       calibrationBuffers.predictedJointTorquesForCalibration);

        // The kinematics information was already set by the readSensorsAndUpdateKinematics method, just compute the offset and add to the buffer
        for(size_t ft = 0; ft < ftSensors.size(); ft++)
        {
            if( calibrationBuffers.calibratingFTsensor[ft] )
            {
                iDynTree::Wrench estimatedFT;
                iDynTree::Wrench measuredRawFT;
                calibrationBuffers.predictedSensorMeasurementsForCalibration.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,estimatedFT);
                rawSensorsMeasurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,measuredRawFT);

                // We apply only the secondary calibration matrix because we are actually computing the offset right now
                measuredRawFT = ftProcessors[ft].applySecondaryCalibrationMatrix(measuredRawFT);

                addToSummer(calibrationBuffers.offsetSumBuffer[ft],measuredRawFT-estimatedFT);
                addToSummer(calibrationBuffers.measurementSumBuffer[ft],measuredRawFT);
                addToSummer(calibrationBuffers.estimationSumBuffer[ft],estimatedFT);
            }
        }

        // Increase the number of collected samples
        calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration++;

        if( calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration >= calibrationBuffers.nrOfSamplesToUseForCalibration )
        {
            // Compute the offset by averaging the results
            for(size_t ft = 0; ft < ftSensors.size(); ft++)
            {
                if( calibrationBuffers.calibratingFTsensor[ft] )
                {
                    iDynTree::Wrench measurementMean, estimationMean;
                    computeMean(calibrationBuffers.offsetSumBuffer[ft],calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration,ftProcessors[ft].offset());
                    computeMean(calibrationBuffers.measurementSumBuffer[ft],calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration,measurementMean);
                    computeMean(calibrationBuffers.estimationSumBuffer[ft],calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration,estimationMean);

                    yInfo() << "wholeBodyDynamics: Offset for sensor " << estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName() << " " << ftProcessors[ft].offset().toString();
                    yInfo() << "wholeBodyDynamics: obtained assuming a measurement of " << measurementMean.asVector().toString() << " and an estimated ft of " << estimationMean.asVector().toString();
                }
            }

            // We finalize the calibration
            this->endCalibration();
        }
    }

}


void WholeBodyDynamicsDevice::computeExternalForcesAndJointTorques()
{
    // The kinematics information was already set by the readSensorsAndUpdateKinematics method
    //This is receiving contact location but no wrench information from the contacts TODO: integrate dynContact info
    estimationWentWell = estimator.estimateExtWrenchesAndJointTorques(measuredContactLocations,filteredSensorMeasurements,
                                                                       estimateExternalContactWrenches,estimatedJointTorques);
}

void WholeBodyDynamicsDevice::publishEstimatedQuantities()
{
    if( !estimationWentWell )
    {
        yError() << "WholeBodyDynamicsDevice: Error in estimation, no estimates will be published.";
    }
    else
    {
        // Only send estimation if a valid offset is available
        if( validOffsetAvailable )
        {
            //Send torques
            publishTorques();

            //Send external contacts
            publishContacts();

            //Send external wrench estimates
            publishExternalWrenches();

            // Send gravity compensation torques
            publishGravityCompensation();

            //Send filtered inertia for gravity compensation
            //publishFilteredInertialForGravityCompensator();

            //Send filtered force torque sensor measurment, if requested
            if( streamFilteredFT){
                publishFilteredFTWithoutOffset();
            }
        }
    }
}

void WholeBodyDynamicsDevice::publishGravityCompensation()
{
    if( m_gravityCompensationEnabled )
    {
        this->m_gravCompHelper.getGravityCompensationTorques(this->m_gravityCompensationTorques);
        // Publish torques only in joints that are in compliant mode that they need it

        for(size_t ii=0; ii < m_gravityCompesationJoints.size(); ii++)
        {
            size_t dof = m_gravityCompesationJoints[ii];

            int ctrl_mode=0;
            yarp::dev::InteractionModeEnum int_mode;
            remappedControlBoardInterfaces.ctrlmode->getControlMode(dof,&ctrl_mode);
            remappedControlBoardInterfaces.intmode->getInteractionMode(dof,&int_mode);

            switch(ctrl_mode)
            {
                case VOCAB_CM_POSITION:
                case VOCAB_CM_POSITION_DIRECT:
                case VOCAB_CM_MIXED:
                case VOCAB_CM_VELOCITY:
                     if (int_mode == VOCAB_IM_COMPLIANT)
                     {
                         remappedControlBoardInterfaces.impctrl->setImpedanceOffset((int)dof,this->m_gravityCompensationTorques(dof));
                     }
                     else
                     {
                         //stiff or unknown mode, nothing to do
                     }
                     break;
                default:
                    // We don't do anything in VOCAB_CM_TORQUE, differently from the
                    // old gravity compensation : because otherwise we interfere
                    // with any torque control loop
                    //
                    // for all this other control modes do nothing
                    // VOCAB_CM_PWM
                    // VOCAB_CM_CURRENT
                    // VOCAB_CM_OPENLOOP:
                    // VOCAB_CM_IDLE:
                    // VOCAB_CM_UNKNOWN:
                    // VOCAB_CM_HW_FAULT:
                    break;
            }
        }
    }
}

void WholeBodyDynamicsDevice::resetGravityCompensation()
{
    if( m_gravityCompensationEnabled )
    {
        for(size_t ii=0; ii < m_gravityCompesationJoints.size(); ii++)
        {
            size_t dof = m_gravityCompesationJoints[ii];

            // Regardless of the controlmode, we reset the setImpedanceOffset
            remappedControlBoardInterfaces.impctrl->setImpedanceOffset((int)dof,0.0);
        }
    }
}

template <class T> void broadcastData(T& _values, yarp::os::BufferedPort<T>& _port)
{
    if (_port.getOutputCount()>0 )
    {
        _port.prepare()  = _values ;
        _port.write();
    }
}

void WholeBodyDynamicsDevice::publishTorques()
{
    iDynTree::toYarp(this->estimatedJointTorques,this->estimatedJointTorquesYARP);
    this->remappedVirtualAnalogSensorsInterfaces.ivirtsens->updateVirtualAnalogSensorMeasure(this->estimatedJointTorquesYARP);
}

void WholeBodyDynamicsDevice::publishContacts()
{
    // Clear the buffer of published forces
    contactsEstimated.clear();

    // Convert the result of estimation
    bool ok = conversionHelper.updateSkinContactListFromLinkContactWrenches(estimator.model(),estimateExternalContactWrenches,contactsEstimated);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : publishContacts() error in converting estimated external wrenches from iDynTree to skinDynLib";
    }

    if( ok )
    {
        broadcastData(contactsEstimated,portContactsOutput);
    }
}

void WholeBodyDynamicsDevice::publishExternalWrenches()
{
    if( this->outputWrenchPorts.size() > 0 )
    {
        // Update kinDynComp model
        iDynTree::Vector3 dummyGravity;
        dummyGravity.zero();
        this->kinDynComp.setRobotState(this->jointPos,this->jointVel,dummyGravity);

        // Compute net wrenches for each link
        estimateExternalContactWrenches.computeNetWrenches(netExternalWrenchesExertedByTheEnviroment);
    }


    // Get wrenches from the estimator and publish it on the port
    for(size_t i=0; i < this->outputWrenchPorts.size(); i++ )
    {
        // Get the wrench in the link frame
        iDynTree::LinkIndex link = this->outputWrenchPorts[i].link_index;
        iDynTree::Wrench & link_f = netExternalWrenchesExertedByTheEnviroment(link);

        // Transform the wrench in the desired frame
        iDynTree::FrameIndex orientation = this->outputWrenchPorts[i].orientation_frame_index;
        iDynTree::FrameIndex origin      = this->outputWrenchPorts[i].origin_frame_index;
        iDynTree::Wrench pub_f = this->kinDynComp.getRelativeTransformExplicit(origin,orientation,link,link)*link_f;

        iDynTree::toYarp(pub_f,outputWrenchPorts[i].output_vector);

        broadcastData<yarp::sig::Vector>(outputWrenchPorts[i].output_vector,
                                         *(outputWrenchPorts[i].output_port));
    }
}

void WholeBodyDynamicsDevice::publishFilteredFTWithoutOffset()
{

        iDynTree::Wrench filteredFTMeasure;
        yarp::sig::Vector filteredFT;
        // Get filtered wrenches and publish it on the port
        for(int ft=0; ft <outputFTPorts.size(); ft++ )
        {
            filteredSensorMeasurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,filteredFTMeasure);
            iDynTree::toYarp(filteredFTMeasure,filteredFT);
            broadcastData<yarp::sig::Vector>(filteredFT,
                                             *(outputFTPorts[ft]));

        }

}

void WholeBodyDynamicsDevice::run()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    if( correctlyConfigured )
    {
        // Load settings if modified
        //this->reconfigureClassFromSettings();

        // Read sensor readings
        this->readSensors();

        // Filter sensor and remove offset
        this->filterSensorsAndRemoveSensorOffsets();

        // Update kinematics
        this->updateKinematics();

        // Read contacts info from the skin or from assume contact location
        this->readContactPoints();

        // Compute calibration if we are in calibration mode
        this->computeCalibration();

        // Compute estimated external forces and internal joint torques
        this->computeExternalForcesAndJointTorques();

        // Publish estimated quantities
        this->publishEstimatedQuantities();
    }
}

bool WholeBodyDynamicsDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    correctlyConfigured = false;

    if (isRunning())
    {
        stop();
    }

    // If gravity compensation was enabled, reset the offsets
    this->resetGravityCompensation();


    this->remappedControlBoardInterfaces.multwrap->detachAll();
    this->remappedVirtualAnalogSensorsInterfaces.multwrap->detachAll();

    closeFilteredFTPorts();
    closeExternalWrenchesPorts();
    closeRPCPort();
    closeSettingsPort();
    closeSkinContactListsPorts();


    return true;
}

bool WholeBodyDynamicsDevice::close()
{
    this->remappedControlBoard.close();
    this->remappedVirtualAnalogSensors.close();

    correctlyConfigured = false;

    return true;
}

bool WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchOnOneFrame(const std::string & frameName, const int32_t nrOfSamples)
{
    // Let's configure the external forces that then are assume to be active on the robot while calibration

    // Clear the class
    calibrationBuffers.assumedContactLocationsForCalibration.clear();

    // Check if the frame exist
    iDynTree::FrameIndex frameIndex = estimator.model().getFrameIndex(frameName);
    if( frameIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchOnOneFrame impossible to find frame " << frameName;
        return false;
    }

    // We assume that the contact is a 6-D the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameIndex,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchOnOneFrame error for frame " << frameName;
        return false;
    }

    setupCalibrationCommonPart(nrOfSamples);

    return true;
}

void WholeBodyDynamicsDevice::setupCalibrationCommonPart(const int32_t nrOfSamples)
{
    calibrationBuffers.nrOfSamplesToUseForCalibration = (size_t)nrOfSamples;
    calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration = 0;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.calibratingFTsensor[ft] = true;
    }
    calibrationBuffers.ongoingCalibration = true;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.offsetSumBuffer[ft].zero();
        calibrationBuffers.measurementSumBuffer[ft].zero();
        calibrationBuffers.estimationSumBuffer[ft].zero();
    }
}

bool WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames(const std::string & frame1Name, const std::string & frame2Name, const int32_t nrOfSamples)
{
    // Let's configure the external forces that then are assume to be active on the robot while calibration on two links (assumed to be simmetric)

    // Clear the class
    calibrationBuffers.assumedContactLocationsForCalibration.clear();

    // Check if the frame exist
    iDynTree::FrameIndex frame1Index = estimator.model().getFrameIndex(frame1Name);
    if( frame1Index == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame1Name;
        return false;
    }

    iDynTree::FrameIndex frame2Index = estimator.model().getFrameIndex(frame2Name);
    if( frame2Index == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame2Name;
        return false;
    }

    // We assume that both  contacts are a 6-D Wrench the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame1Index,calibrationAssumedContact);
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame2Index,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchesOnTwoFrames error";
        return false;
    }

    setupCalibrationCommonPart(nrOfSamples);

    return true;
}

bool WholeBodyDynamicsDevice::calib(const std::string& calib_code, const int32_t nr_of_samples)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : calib ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("base_link",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;

}

bool WholeBodyDynamicsDevice::calibStanding(const std::string& calib_code, const int32_t nr_of_samples)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : calibStanding ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchesOnTwoFrames("r_sole","l_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;

}

bool WholeBodyDynamicsDevice::calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yWarning() << " wholeBodyDynamics : calibStandingLeftFoot ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("l_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yWarning() << " wholeBodyDynamics : calibStandingRightFoot ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("r_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;

}

bool WholeBodyDynamicsDevice::calibStandingOnOneLink(const std::string &standing_frame, const int32_t nr_of_samples)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame(standing_frame,nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::calibStandingOnTwoLinks(const std::string &first_standing_frame,
                                                      const std::string &second_standing_frame,
                                                      const int32_t nr_of_samples)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    bool ok = this->setupCalibrationWithExternalWrenchesOnTwoFrames(first_standing_frame,second_standing_frame,nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::resetOffset(const std::string& calib_code)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : calib ignoring calib_code " << calib_code;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        ftProcessors[ft].offset().zero();
    }

    return true;
}


bool WholeBodyDynamicsDevice::changeFixedLinkSimpleLeggedOdometry(const std::string& /*new_fixed_link*/)
{
    yError() << "wholeBodyDynamics : changeFixedLinkSimpleLeggedOdometry method not implemented";
    return false;
}

double WholeBodyDynamicsDevice::get_forceTorqueFilterCutoffInHz()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    return this->settings.forceTorqueFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_forceTorqueFilterCutoffInHz(const double newCutoff)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    this->settings.forceTorqueFilterCutoffInHz = newCutoff;

    return true;
}

double WholeBodyDynamicsDevice::get_jointVelFilterCutoffInHz()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    return this->settings.jointVelFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_jointVelFilterCutoffInHz(const double newCutoff)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    this->settings.jointVelFilterCutoffInHz = newCutoff;

    return true;
}

double WholeBodyDynamicsDevice::get_jointAccFilterCutoffInHz()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    return this->settings.jointAccFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_jointAccFilterCutoffInHz(const double newCutoff)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    this->settings.jointAccFilterCutoffInHz = newCutoff;

    return true;
}


double WholeBodyDynamicsDevice::get_imuFilterCutoffInHz()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    return this->settings.imuFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_imuFilterCutoffInHz(const double newCutoff)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    this->settings.imuFilterCutoffInHz = newCutoff;

    return true;
}

bool WholeBodyDynamicsDevice::useFixedFrameAsKinematicSource(const std::string& fixedFrame)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    iDynTree::FrameIndex fixedFrameIndex = estimator.model().getFrameIndex(fixedFrame);

    if( fixedFrameIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : useFixedFrameAsKinematicSource : requested not exiting frame " << fixedFrame << ", method failed";
        return false;
    }

    // Set the kinematic source to a fixed frame
    settings.kinematicSource = FIXED_FRAME;
    settings.fixedFrameName = fixedFrame;

    yInfo() << "wholeBodyDynamics : successfully set the kinematic source to be the fixed frame " << fixedFrame;
    yInfo() << "wholeBodyDynamics : with gravity " << settings.fixedFrameGravity.toString();

    return true;
}

bool WholeBodyDynamicsDevice::useIMUAsKinematicSource()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yInfo() << "wholeBodyDynamics : successfully set the kinematic source to be the IMU ";

    settings.kinematicSource = IMU;

    return true;
}

bool WholeBodyDynamicsDevice::setUseOfJointVelocities(const bool enable)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    this->settings.useJointVelocity = enable;

    return true;
}

bool WholeBodyDynamicsDevice::setUseOfJointAccelerations(const bool enable)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    this->settings.useJointAcceleration = enable;

    return true;
}

std::string WholeBodyDynamicsDevice::getCurrentSettingsString()
{
   std::lock_guard<std::mutex> guard(this->deviceMutex);

   return settings.toString();
}

bool WholeBodyDynamicsDevice::resetSimpleLeggedOdometry(const std::string& /*initial_world_frame*/, const std::string& /*initial_fixed_link*/)
{
    yError() << " wholeBodyDynamics : resetSimpleLeggedOdometry method not implemented";
    return false;
}

bool WholeBodyDynamicsDevice::quit()
{
    yError() << " wholeBodyDynamics : quit method not implemented";
    return false;
}

size_t WholeBodyDynamicsDevice::getNrOfFTSensors()
{
    return this->ftProcessors.size();
}

void WholeBodyDynamicsDevice::endCalibration()
{
    validOffsetAvailable = true;
    calibrationBuffers.ongoingCalibration = false;
    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.calibratingFTsensor[ft] = false;
    }

    yInfo() << " : calibration ended.";

    return;
}

wholeBodyDynamicsDeviceFilters::wholeBodyDynamicsDeviceFilters(): imuLinearAccelerationFilter(0),
                                                                  imuAngularVelocityFilter(0),
                                                                  forcetorqueFilters(0),
                                                                  jntVelFilter(0),
                                                                  jntAccFilter(0),
                                                                  bufferYarp3(0),
                                                                  bufferYarp6(0),
                                                                  bufferYarpDofs(0)
{

}

void wholeBodyDynamicsDeviceFilters::init(int nrOfFTSensors,
                                          double initialCutOffForFTInHz,
                                          double initialCutOffForIMUInHz,
                                          int nrOfDOFsProcessed,
                                          double initialCutOffForJointVelInHz,
                                          double initialCutOffForJointAccInHz,
                                          double periodInSeconds)
{
    // Allocate buffers
    bufferYarp3.resize(3,0.0);
    bufferYarp6.resize(6,0.0);
    bufferYarpDofs.resize(nrOfDOFsProcessed,0.0);

    imuLinearAccelerationFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForIMUInHz,periodInSeconds,bufferYarp3);
    imuAngularVelocityFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForIMUInHz,periodInSeconds,bufferYarp3);

    forcetorqueFilters.resize(nrOfFTSensors);
    for(int ft_numeric = 0; ft_numeric < nrOfFTSensors; ft_numeric++ )
    {
        forcetorqueFilters[ft_numeric] =
                new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForFTInHz,periodInSeconds,bufferYarp6);
    }

    jntVelFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForJointVelInHz,periodInSeconds,bufferYarpDofs);
    jntAccFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForJointAccInHz,periodInSeconds,bufferYarpDofs);
}


void wholeBodyDynamicsDeviceFilters::updateCutOffFrequency(double cutoffForFTInHz,
                                                           double cutOffForIMUInHz,
                                                           double cutOffForJointVelInHz,
                                                           double cutOffForJointAccInHz)
{
    imuLinearAccelerationFilter->setCutFrequency(cutOffForIMUInHz);
    imuAngularVelocityFilter->setCutFrequency(cutOffForIMUInHz);

    for(size_t ft_numeric = 0; ft_numeric < forcetorqueFilters.size(); ft_numeric++ )
    {
        forcetorqueFilters[ft_numeric]->setCutFrequency(cutoffForFTInHz);
    }

    jntVelFilter->setCutFrequency(cutOffForJointVelInHz);
    jntAccFilter->setCutFrequency(cutOffForJointAccInHz);
}

void wholeBodyDynamicsDeviceFilters::fini()
{
    if( imuLinearAccelerationFilter )
    {
        delete imuLinearAccelerationFilter;
        imuLinearAccelerationFilter = 0;
    }

    if( imuAngularVelocityFilter )
    {
        delete imuAngularVelocityFilter;
        imuAngularVelocityFilter = 0;
    }

    for(size_t ft_numeric = 0; ft_numeric < forcetorqueFilters.size(); ft_numeric++ )
    {
        delete forcetorqueFilters[ft_numeric];
        forcetorqueFilters[ft_numeric] = 0;
    }

    forcetorqueFilters.resize(0);

    if( jntVelFilter )
    {
        delete jntVelFilter;
        jntVelFilter = 0;
    }

    if( jntAccFilter )
    {
        delete jntAccFilter;
        jntAccFilter = 0;
    }
}

wholeBodyDynamicsDeviceFilters::~wholeBodyDynamicsDeviceFilters()
{
    fini();
}




}
}


