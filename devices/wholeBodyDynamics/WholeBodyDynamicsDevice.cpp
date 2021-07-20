#define SKIN_EVENTS_TIMEOUT 0.2
#include "WholeBodyDynamicsDevice.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <cassert>
#include <cmath>

namespace yarp
{
namespace dev
{

const size_t wholeBodyDynamics_nrOfChannelsOfYARPFTSensor = 6;
const size_t wholeBodyDynamics_nrOfChannelsOfAYARPIMUSensor = 12;
const double wholeBodyDynamics_sensorTimeoutInSeconds = 2.0;
constexpr double defaultWholeBodyDynamicsPeriod = 0.01;

WholeBodyDynamicsDevice::WholeBodyDynamicsDevice(): yarp::os::PeriodicThread(defaultWholeBodyDynamicsPeriod),
                                                    portPrefix("/wholeBodyDynamics"),
                                                    correctlyConfigured(false),
                                                    sensorReadCorrectly(false),
                                                    estimationWentWell(false),
                                                    validOffsetAvailable(false),
                                                    lastReadingSkinContactListStamp(0.0),
                                                    streamFilteredFT(false),
                                                    checkTemperatureEvery_seconds(0.55),
                                                    useSkinForContacts{true},
                                                    isIMUAttached{false},
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
    // temperature measuremnts vector
    tempMeasurements.resize(0);
    ftTempMapping.resize(0);
    prevFTTempTimeStamp=0;
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

    netExternalWrenchesPort.close();

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

bool getConfigParamsAsList(os::Searchable& config,std::string propertyName , std::vector<std::string> & list, bool isRequired)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());
    yarp::os::Bottle *propNames=prop.find(propertyName).asList();
    if(propNames==nullptr)
    {
        if(isRequired)
        {
            yError() <<"WholeBodyDynamicsDevice: Error parsing parameters: \" "<<propertyName<<" \" should be followed by a list\n";
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

bool getUsedDOFsList(os::Searchable& config, std::vector<std::string> & usedDOFs)
{
    bool required{true};
    return getConfigParamsAsList(config,"axesNames",usedDOFs, required);
}

bool getAdditionalConsideredFixedJointsList(os::Searchable& config, std::vector<std::string> & additionalFixedJoints)
{
    bool notRequired{false};
    return getConfigParamsAsList(config,"additionalConsideredFixedJoints",additionalFixedJoints, notRequired);
}

bool getGravityCompensationDOFsList(os::Searchable& config, std::vector<std::string> & gravityCompensationDOFs)
{
    bool required{true};
    return getConfigParamsAsList(config,"gravityCompensationAxesNames",gravityCompensationDOFs, required);
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

    // Add additional fixed joints listed with the parameter `additionalConsideredFixedJoints`
    std::vector<std::string> additionalConsideredJoints;
    bool additionalFixedJointsExist = getAdditionalConsideredFixedJointsList(config,additionalConsideredJoints);
    if(additionalFixedJointsExist)
    {
        yInfo() << "wholeBodyDynamics: Loading additional fixed joints from the config file: " << additionalConsideredJoints;
        // Append additionalConsideredJoints to estimationJointNames
        estimationJointNames.insert(estimationJointNames.end(), additionalConsideredJoints.begin(), additionalConsideredJoints.end());
    }

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

    return true;
}

bool WholeBodyDynamicsDevice::openContactFrames(os::Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    yarp::os::Bottle *propDefaultContactFrames=prop.find("defaultContactFrames").asList();
    yarp::os::Bottle *propOverrideContactFrames=prop.find("overrideContactFrames").asList();
    yarp::os::Bottle *propContactWrenchType=prop.find("contactWrenchType").asList();
    yarp::os::Bottle *propContactWrenchDirection=prop.find("contactWrenchDirection").asList();
    yarp::os::Bottle *propContactWrenchPosition=prop.find("contactWrenchPosition").asList();

    if(propOverrideContactFrames==0)
    {
        //overrideContactFrames don't exist -> considering defaultContactFrames
        if(propDefaultContactFrames==0)
        {
            yError() << "wholeBodyDynamics : openContactFrames : Error parsing parameters: \"defaultContactFrames\" should be followed by a list\n";
            return false;
        }
        else
        {
            //fill the variable `contactFramesNames` and build objects "iDynTree::UnknownWrenchContact" for each default contact frame
            contactFramesNames.resize(propDefaultContactFrames->size());
            unknownExtWrench.resize(contactFramesNames.size());
            nrUnknownsInExtWrench.resize(contactFramesNames.size());

            for(int ax=0; ax < propDefaultContactFrames->size(); ax++)
            {
                contactFramesNames[ax] = propDefaultContactFrames->get(ax).asString().c_str();

                unknownExtWrench[ax].contactPoint = iDynTree::Position::Zero();
                unknownExtWrench[ax].unknownType = iDynTree::FULL_WRENCH;
                unknownExtWrench[ax].forceDirection = iDynTree::Direction::Default();
                nrUnknownsInExtWrench[ax] = 6;
            }
        }
    }
    else
    {
        //overrideContactFrames exist -> they are parsed instead of defaultContactFrames
        overrideContactFramesSelected = true;

        bool OverrideFrameParsedOK = parseOverrideContactFramesData(propOverrideContactFrames, propContactWrenchType, propContactWrenchDirection, propContactWrenchPosition);

        if(!OverrideFrameParsedOK)
        {
            yError() << "wholeBodyDynamics : openContactFrames :  parameters 'overrideContactFrames', 'contactWrenchType', 'contactWrenchDirection' and 'contactWrenchPosition' contain some problems.";
            return false;
        }
    }

    // We build the contactFramesIdx vector
    std::vector<iDynTree::FrameIndex> contactFramesIdx;
    contactFramesIdx.clear();
    std::vector<int> indexesToDiscard;
    indexesToDiscard.clear();
    for(size_t i=0; i < contactFramesNames.size(); i++)
    {
        iDynTree::FrameIndex idx = estimator.model().getFrameIndex(contactFramesNames[i]);

        if( idx == iDynTree::FRAME_INVALID_INDEX )
        {
            yWarning() << "Frame " << contactFramesNames[i] << " not found in the model, discarding it";
            indexesToDiscard.push_back(i);
        }
        else
        {
            contactFramesIdx.push_back(idx);
        }
    }

    sort(indexesToDiscard.begin(), indexesToDiscard.end());
    //remove the variables that correspond to invalid frames
    for(int i=indexesToDiscard.size()-1; i>=0; i-- )
    {
        contactFramesNames.erase(contactFramesNames.begin() + indexesToDiscard[i]);
        unknownExtWrench.erase(unknownExtWrench.begin() + indexesToDiscard[i]);
        nrUnknownsInExtWrench.erase(nrUnknownsInExtWrench.begin() + indexesToDiscard[i]);
    }

    // The frames from in 1 submodel should be no more than 1 if the type is FULL_WRENCH, no more than 2 if the type is PURE_FORCE, and no more than 6 if the type is PURE_FORCE_WITH_KNOWN_DIRECTION.
    // Refer to section 3 of the theory paper https://www.researchgate.net/publication/236152161_Contact_Force_Estimations_Using_Tactile_Sensors_and_Force_Torque_Sensors

    // For each submodel, we find the first suitable contact frame
    // This is a n^2 algorithm, but given that is just used in
    // configuration it should be ok
    size_t nrOfSubModels = estimator.submodels().getNrOfSubModels();

    // We indicate with FRAME_INVALID_INDEX the fact that we still don't have a default/override contact for the given submodel
    contactFramesIdxValidForSubModel.resize(contactFramesIdx.size(),iDynTree::FRAME_INVALID_INDEX);
    //create a vector to store the submodel indexes that will have contact points
    std::vector<size_t> validSubModelsIndexes;
    validSubModelsIndexes.resize(contactFramesIdx.size(),iDynTree::FRAME_INVALID_INDEX);

    nrUnknownsInSubModel.resize(nrOfSubModels,0);

    for(size_t i=0; i < contactFramesIdx.size(); i++)
    {
        size_t subModelIdx = estimator.submodels().getSubModelOfFrame(estimator.model(),contactFramesIdx[i]);

        //check if adding the current contact to this subModel will increase the no of variable to more than 6
        if(nrUnknownsInSubModel[subModelIdx]+nrUnknownsInExtWrench[i] > 6)
        {
            yWarning() << "subModel no " << subModelIdx << " has the maximum number of variables (6). The frame " << contactFramesNames[i] << " will not be added to it.";
        }
        else
        {
            //add the contact frame
            contactFramesIdxValidForSubModel[i] = contactFramesIdx[i];
            //add the submodel index to the valid submodel indexes group
            validSubModelsIndexes[i] = subModelIdx;
            nrUnknownsInSubModel[subModelIdx] += nrUnknownsInExtWrench[i];
        }
    }

    //after filling the valid submodels vector, we sort it to be able to use "std::binary_search"
    std::sort(validSubModelsIndexes.begin(), validSubModelsIndexes.end());

    // Let's check that every submodel has a default/override contact position
    std::vector<iDynTree::FrameIndex> invalidFrameIndexVector(nrOfSubModels,iDynTree::FRAME_INVALID_INDEX);
    for(size_t subModelIdx = 0; subModelIdx < nrOfSubModels; subModelIdx++)
    {
        //check if the submodel is not in the list
        if(!std::binary_search(validSubModelsIndexes.begin(), validSubModelsIndexes.end(), subModelIdx))
        {
            yError() << "wholeBodyDynamics : openContactFrames : missing default/override contact for submodel composed by the links: ";
            const iDynTree::Traversal & subModelTraversal = estimator.submodels().getTraversal(subModelIdx);
            for(iDynTree::TraversalIndex i = 0; i < (iDynTree::TraversalIndex) subModelTraversal.getNrOfVisitedLinks(); i++)
            {
                iDynTree::LinkIndex linkIdx = subModelTraversal.getLink(i)->getIndex();
                yError() << "wholeBodyDynamics : openContactFrames :" << estimator.model().getLinkName(linkIdx);
            }
            return false;
        }
    }
    return true;
}

bool WholeBodyDynamicsDevice::parseOverrideContactFramesData(yarp::os::Bottle *_propOverrideContactFrames, yarp::os::Bottle *_propContactWrenchType, yarp::os::Bottle *_propContactWrenchDirection, yarp::os::Bottle *_propContactWrenchPosition)
{
    //fill the variable `contactFramesNames`
    contactFramesNames.resize(_propOverrideContactFrames->size());
    for(int ax=0; ax < _propOverrideContactFrames->size(); ax++)
    {
        contactFramesNames[ax] = _propOverrideContactFrames->get(ax).asString().c_str();
    }

    //Check if the parameters `propcontactWrenchType`, `contactWrenchDirection` and `contactWrenchPosition` exist in the configuration file
    if(_propContactWrenchType==0 || _propContactWrenchDirection==0 || _propContactWrenchPosition==0)
    {
        yError() << "wholeBodyDynamics : openContactFrames :  missing necessary parameters for the overrideContactFrames.";
        return false;
    }
    else
    {
        //fill the other variables from the prop objects
        contactWrenchType.resize(_propContactWrenchType->size());
        for(int ax=0; ax < _propContactWrenchType->size(); ax++)
        {
            contactWrenchType[ax] = _propContactWrenchType->get(ax).asString().c_str();
        }

        //Check if the size of propContactWrenchDirection is a multiple of 3
        if(_propContactWrenchDirection->size()%3 == 0)
        {
            contactWrenchDirection.resize(_propContactWrenchDirection->size()/3);
            for(int ax=0; ax < contactWrenchDirection.size(); ax++)
            {
                contactWrenchDirection[ax].resize(3);
                //put every 3 elements of `propContactWrenchDirection` in one raw of `contactWrenchDirection`
                for(int ay=0; ay < 3; ay++)
                {
                    int index = 3*ax+ay;
                    contactWrenchDirection[ax][ay] = _propContactWrenchDirection->get(index).asDouble();
                }
            }
        }
        else
        {
            yError() << "wholeBodyDynamics : openContactFrames :  parameter 'contactWrenchDirection' size is not consistent.";
            return false;
        }

        //Check if the size of propContactWrenchPosition is a mulitple of 3
        if(_propContactWrenchPosition->size()%3 == 0)
        {
            contactWrenchPosition.resize(_propContactWrenchPosition->size()/3);
            for(int ax=0; ax < contactWrenchPosition.size(); ax++)
            {
                contactWrenchPosition[ax].resize(3);
                //put every 3 elements of `propContactWrenchPosition` in one raw of `contactWrenchPosition`
                for(int ay=0; ay < 3; ay++)
                {
                    int index = 3*ax+ay;
                    contactWrenchPosition[ax][ay] = _propContactWrenchPosition->get(index).asDouble();
                }
            }
        }
        else
        {
            yError() << "wholeBodyDynamics : openContactFrames :  parameter 'contactWrenchPosition' size is not consistent.";
            return false;
        }

        //check full size of the lists `contactWrenchType`, `contactWrenchDirection` and `contactWrenchPosition` to verify it's consistent with  number of frames in `overrideContactFrames`
        if(contactWrenchType.size() == contactFramesNames.size() && contactWrenchDirection.size() == contactFramesNames.size() && contactWrenchPosition.size() == contactFramesNames.size())
        {
            yInfo() << "wholeBodyDynamics : openContactFrames : parsing the values from the parameters 'overrideContactFrames', 'contactWrenchType', 'contactWrenchDirection' and 'contactWrenchPosition'.";
        }
        else
        {
            yError() << "wholeBodyDynamics : openContactFrames :  parameters 'overrideContactFrames', 'contactWrenchType', 'contactWrenchDirection' and 'contactWrenchPosition' sizes are not consistent.";
            return false;
        }
    }

    //build objects "iDynTree::UnknownWrenchContact" for each override contact frame
    unknownExtWrench.resize(contactFramesNames.size());
    nrUnknownsInExtWrench.resize(contactFramesNames.size());
    // check if the parameter `contactWrenchType` contains only the values "full", "pure" or "pureKnown"
    for(int i=0; i<contactFramesNames.size(); i++)
    {
        unknownExtWrench[i].contactPoint = iDynTree::Position(contactWrenchPosition[i][0], contactWrenchPosition[i][1], contactWrenchPosition[i][2]);

        if(contactWrenchType[i]=="full")
        {
            nrUnknownsInExtWrench[i] = 6;
            unknownExtWrench[i].unknownType = iDynTree::FULL_WRENCH;
            unknownExtWrench[i].forceDirection = iDynTree::Direction::Default();
        }
        else if(contactWrenchType[i]=="pure")
        {
            nrUnknownsInExtWrench[i] = 3;
            unknownExtWrench[i].unknownType = iDynTree::PURE_FORCE;
            unknownExtWrench[i].forceDirection = iDynTree::Direction::Default();
        }
        else if(contactWrenchType[i]=="pureKnown")
        {
            nrUnknownsInExtWrench[i] = 1;
            unknownExtWrench[i].unknownType = iDynTree::PURE_FORCE_WITH_KNOWN_DIRECTION;
            unknownExtWrench[i].forceDirection = iDynTree::Direction(contactWrenchDirection[i][0], contactWrenchDirection[i][1], contactWrenchDirection[i][2]);
        }
        else
        {
            yError() << "wholeBodyDynamics : openContactFrames :  parameter 'contactWrenchType' contains some errors/inconsistencies.";
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

    std::string outputWrenchPortInfoType = config.find("outputWrenchPortInfoType").asString();
    if(outputWrenchPortInfoType == "contactWrenches")
    {
        m_outputWrenchPortInfoType = contactWrenches;
    }
    else
    {
        m_outputWrenchPortInfoType = netWrench;
    }

    enablePublishNetExternalWrenches = config.check("publishNetExternalWrenches", yarp::os::Value(false)).asBool();
    if (enablePublishNetExternalWrenches)
    {
        if (!netExternalWrenchesPort.open(portPrefix +"/netExternalWrenches:o"))
        {
            yError() << "wholeBodyDynamics impossible to open port for publishing net external wrenches";
            return false;
        }
    }

    return ok;
}

bool WholeBodyDynamicsDevice::openMultipleAnalogSensorRemapper(os::Searchable &config)
{
    // Pass to the remapper just the relevant parameters (sensorList)
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());
    yarp::os::Property propMASRemapper;
    yarp::os::Bottle & propMasNames=prop.findGroup("multipleAnalogSensorsNames");
    propMASRemapper.put("device","multipleanalogsensorsremapper");
    bool ok=false;
    for (auto types=1u;types<propMasNames.size();types++){
        yarp::os::Bottle * mas_type = propMasNames.get(types).asList();
        if( mas_type->size() != 2 || mas_type->get(1).asList() == nullptr )
        {
            yError() << "wholeBodyDynamics: multipleAnalogSensorsNames group is malformed (" << mas_type->toString() << "). ";
            return false;
        }
        else {
            ok=true;
        }

        propMASRemapper.addGroup(mas_type->get(0).asString());
        yarp::os::Bottle  MASnames;
        yarp::os::Bottle & MASnamesList= MASnames.addList();
        for(auto i=0u; i < mas_type->get(1).asList()->size(); i++)
        {
            MASnamesList.addString(mas_type->get(1).asList()->get(i).asString());
        }
        propMASRemapper.put(mas_type->get(0).asString(),MASnames.get(0));

    }
    ok = multipleAnalogRemappedDevice.open(propMASRemapper);
    if( !ok )
    {
        return ok;
    }
    // View relevant interfaces for the multipleAnalogRemappedDevice
    ok = ok && multipleAnalogRemappedDevice.view(remappedMASInterfaces.temperatureSensors);
    ok = ok && multipleAnalogRemappedDevice.view(remappedMASInterfaces.ftMultiSensors);
    ok = ok && multipleAnalogRemappedDevice.view(remappedMASInterfaces.multwrap);
    if( !ok )
    {
        yError() << "wholeBodyDynamics : openMultipleAnalogSensorRemapper : error while opening the necessary interfaces in multipleAnalogRemappedDevice";
        return ok;
    }

   return true;
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

    // Resize temperature sensor inside F/T sensors
    this->tempMeasurements.resize(nrOfFTSensors,0.0);
    this->ftTempMapping.resize(nrOfFTSensors,-1);

    // Resize filters
    filters.init(nrOfFTSensors,
                 settings.forceTorqueFilterCutoffInHz,
                 settings.imuFilterCutoffInHz,
                 estimator.model().getNrOfDOFs(),
                 settings.jointVelFilterCutoffInHz,
                 settings.jointAccFilterCutoffInHz,
                 getPeriod());

    // Resize external wrenches publishing software
    this->netExternalWrenchesExertedByTheEnviroment.resize(estimator.model());
    bool ok = this->kinDynComp.loadRobotModel(estimator.model());

    if( !ok )
    {
        yError() << "wholeBodyDynamics : error in opening KinDynComputation class";
    }

    for (iDynTree::LinkIndex link = 0;
         link < static_cast<iDynTree::LinkIndex>(netExternalWrenchesExertedByTheEnviroment.getNrOfLinks());
         ++link)
    {
        yarp::os::Bottle& wrenchBottle = netExternalWrenchesBottle.addList();
        wrenchBottle.addString(estimator.model().getLinkName(link));
        yarp::os::Bottle& wrenchValues = wrenchBottle.addList();

        for (size_t i = 0; i < 6; ++i)
        {
            wrenchValues.addDouble(0.0);
        }
    }
}


bool WholeBodyDynamicsDevice::loadSettingsFromConfig(os::Searchable& config)
{
    // Fill setting with their default values
    settings.kinematicSource             = IMU;

    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    if (prop.check("devicePeriodInSeconds"))
    {
        if(!prop.find("devicePeriodInSeconds").isDouble())
        {
            yError() << "wholeBodyDynamics : The devicePeriodInSeconds must be a double";
            return false;
        }
        this->setPeriod(prop.find("devicePeriodInSeconds").asDouble());
    }
    else
    {
        yWarning() << "wholeBodyDynamics : The devicePeriodInSeconds parameter is not found. The default one is used. Period:" << this->getPeriod() << "seconds.";
    }


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

    if (settings.kinematicSource != FIXED_FRAME)
    {
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

    if ( prop.check("useSkinForContacts") &&
         prop.find("useSkinForContacts").isBool())
    {
        useSkinForContacts = prop.find("useSkinForContacts").asBool();
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

    // Check for measurements low pass filter cutoff frequencies
    if (!applyLPFSettingsFromConfig(prop, "imuFilterCutoffInHz"))
    {
        yError() << "wholeBodyDynamics : missing required string parameter imuFilterCutoffInHz";
        return false;
    }

    if (!applyLPFSettingsFromConfig(prop, "forceTorqueFilterCutoffInHz"))
    {
        yError() << "wholeBodyDynamics : missing required string parameter forceTorqueFilterCutoffInHz";
        return false;
    }

    if (settings.useJointVelocity && !applyLPFSettingsFromConfig(prop, "jointVelFilterCutoffInHz"))
    {
        yError() << "wholeBodyDynamics : missing required string parameter jointVelFilterCutoffInHz";
        return false;
    }


    if (settings.useJointAcceleration && !applyLPFSettingsFromConfig(prop, "jointAccFilterCutoffInHz"))
    {
        yError() << "wholeBodyDynamics : missing required string parameter jointAccFilterCutoffInHz";
        return false;
    }

    yInfo() << "wholeBodyDynamics : Using the following filter cutoff frequencies,";
    yInfo() << "wholeBodyDynamics: imuFilterCutoffInHz: " <<   settings.imuFilterCutoffInHz << " Hz.";
    yInfo() << "wholeBodyDynamics: forceTorqueFilterCutoffInHz: " <<   settings.forceTorqueFilterCutoffInHz << " Hz.";
    if (settings.useJointVelocity) { yInfo() << "wholeBodyDynamics: jointVelFilterCutoffInHz: " <<   settings.jointVelFilterCutoffInHz << " Hz."; }
    if (settings.useJointAcceleration) { yInfo() << "wholeBodyDynamics: jointAccFilterCutoffInHz: " <<   settings.jointAccFilterCutoffInHz << " Hz."; }

    // Set time to check for a new temperature measurement. The default value is 0.55;
    // is set in the device constructor
    if( prop.check("checkTemperatureEvery_seconds") &&
        prop.find("checkTemperatureEvery_seconds").isDouble())
    {
        checkTemperatureEvery_seconds = prop.find("checkTemperatureEvery_seconds").asDouble();
    }

    if ( !(prop.check("startWithZeroFTSensorOffsets") && prop.find("startWithZeroFTSensorOffsets").isBool()) )
    {
        settings.startWithZeroFTSensorOffsets = false;
    }
    else
    {
        settings.startWithZeroFTSensorOffsets = prop.find("startWithZeroFTSensorOffsets").asBool();
        if (settings.startWithZeroFTSensorOffsets)
        {
            yInfo() << "wholeBodyDynamics: setting startWithZeroFTSensorOffsets was set to true , FT sensor offsets will be automatically reset to zero.";
        }
    }

    std::string disableSensorReadCheckAtStartupOptionName = "disableSensorReadCheckAtStartup";
    if( !(prop.check(disableSensorReadCheckAtStartupOptionName) && prop.find(disableSensorReadCheckAtStartupOptionName).isBool()) )
    {
        settings.disableSensorReadCheckAtStartup = false;
    }
    else
    {
        settings.disableSensorReadCheckAtStartup = prop.find(disableSensorReadCheckAtStartupOptionName).asBool();
    }

    if( !prop.check("HW_USE_MAS_IMU") )
    {
        useMasIMU = false;
    }
    else
    {
        yarp::os::Searchable & propMASIMU = prop.findGroup("HW_USE_MAS_IMU");
        useMasIMU = true;

        if( !(propMASIMU.check("accelerometer") && propMASIMU.find("accelerometer").isString()) )
        {
            yError() << "wholeBodyDynamics: HW_USE_MAS_IMU group found, but accelerometer string parameter missing";
            return false;
        }
        masAccName = propMASIMU.find("accelerometer").asString();

        if( !(propMASIMU.check("gyroscope") && propMASIMU.find("gyroscope").isString()) )
        {
            yError() << "wholeBodyDynamics: HW_USE_MAS_IMU group found, but gyroscope string parameter missing";
            return false;
        }
        masGyroName = propMASIMU.find("gyroscope").asString();
    }

    return true;
}

bool WholeBodyDynamicsDevice::applyLPFSettingsFromConfig(const yarp::os::Property& prop, const std::string& setting_name)
{
    if( prop.check(setting_name) &&
        prop.find(setting_name).isDouble() )
    {
        double cut_off_freq = prop.find(setting_name).asDouble();
        if (setting_name == "imuFilterCutoffInHz") { settings.imuFilterCutoffInHz = cut_off_freq; }
        if (setting_name == "forceTorqueFilterCutoffInHz") { settings.forceTorqueFilterCutoffInHz = cut_off_freq; }
        if (setting_name == "jointVelFilterCutoffInHz") { settings.jointVelFilterCutoffInHz = cut_off_freq; }
        if (setting_name == "jointAccFilterCutoffInHz") { settings.jointAccFilterCutoffInHz = cut_off_freq; }
    }
    else
    {
        return false;
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

bool WholeBodyDynamicsDevice::loadTemperatureCoefficientsSettingsFromConfig(os::Searchable& config)
{
    bool ret;
    yarp::os::Property propAll;
    propAll.fromString(config.toString().c_str());

    if( !propAll.check("FT_TEMPERATURE_COEFFICIENTS") )
    {
        ret = true;
    }
    else
    {
        yarp::os::Bottle & propTempCoeff = propAll.findGroup("FT_TEMPERATURE_COEFFICIENTS");
        for(auto i=1u; i < propTempCoeff.size(); i++ )
        {
            yarp::os::Bottle * map_bot = propTempCoeff.get(i).asList();
            if( map_bot->size() != 2 || map_bot->get(1).asList() == nullptr ||
                    map_bot->get(1).asList()->size() != 7 )
            {
                yError() << "wholeBodyDynamics: FT_TEMPERATURE_COEFFICIENTS group is malformed (" << map_bot->toString() << "). ";
                return false;
            }

            std::string iDynTree_sensorName = map_bot->get(0).asString();
            iDynTree::Vector6 temperatureCoeffs;
            double tempOffset=0;

            for(auto r=0u; r < 6; r++)
            {
                temperatureCoeffs(r) = map_bot->get(1).asList()->get(r).asDouble();
            }
            tempOffset= map_bot->get(1).asList()->get(6).asDouble();

            // Linearly search for the specified sensor
            bool sensorFound = false;
            for(auto ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
            {

                if( estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName() == iDynTree_sensorName )
                {
                    yDebug() << "wholeBodyDynamics: using temperature coefficients for sensor " << iDynTree_sensorName << " temperatureCoeffs values " << temperatureCoeffs.toString();

                    ftProcessors[ft].temperatureCoefficients() = temperatureCoeffs;
                    ftProcessors[ft].tempOffset() =tempOffset;
                    sensorFound = true;
                    break;
                }
            }

            // If a specified sensor was not found, give an error
            if( !sensorFound )
            {
                yError() << "wholeBodyDynamics: temperature coefficients matrix specified for FT sensor " << iDynTree_sensorName
                         << " but no sensor with that name found in the model";
                return false;
            }
        }
        ret = true;
    }

    return ret;

}

bool WholeBodyDynamicsDevice::loadFTSensorOffsetFromConfig(os::Searchable& config)
{
    bool ret;
    yarp::os::Property propAll;
    propAll.fromString(config.toString().c_str());

    if( !propAll.check("FT_OFFSET") )
    {
        ret = true;
    }
    else
    {
        yarp::os::Bottle & propSecondCalib = propAll.findGroup("FT_OFFSET");
        for(auto i=1u; i < propSecondCalib.size(); i++ )
        {
            yarp::os::Bottle * map_bot = propSecondCalib.get(i).asList();
            if( map_bot->size() != 2 || map_bot->get(1).asList() == nullptr ||
                    map_bot->get(1).asList()->size() != 6 )
            {
                yError() << "wholeBodyDynamics: FT_OFFSET group is malformed (" << map_bot->toString() << "). ";
                return false;
            }

            std::string iDynTree_sensorName = map_bot->get(0).asString();
            iDynTree::Wrench  ftOffset;

            for(int r=0; r < 6; r++)
            {
                ftOffset(r) = map_bot->get(1).asList()->get(r).asDouble();
            }


            // Linearly search for the specified sensor
            bool sensorFound = false;
            for(int ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
            {
                if( estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName() == iDynTree_sensorName )
                {
                    yDebug() << "wholeBodyDynamics: using FT offset"<< ftOffset.toString() <<" for sensor " << iDynTree_sensorName;

                    ftProcessors[ft].estimatedOffset() = ftOffset;
                    sensorFound = true;
                }
            }

            // If a specified sensor was not found, give an error
            if( !sensorFound )
            {
                yError() << "wholeBodyDynamics: FT offset specified for FT sensor " << iDynTree_sensorName
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

    yDebug() << "wholeBodyDynamics Statistics: Opening estimator.";
    double tick = yarp::os::Time::now();

    bool ok;
    // Create the estimator
    ok = this->openEstimator(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening estimator object.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: Estimator opened in " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Loading settings from configuration files.";
    tick = yarp::os::Time::now();

    // Load settings in the class
    ok = this->loadSettingsFromConfig(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in loading settings from config.";
        return false;
    }

    // resize internal buffers
    this->resizeBuffers();

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

    // Open settings related to gravity compensation (we need the estimator to be open)
    ok = this->loadTemperatureCoefficientsSettingsFromConfig(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in loading temperature coefficients matrix settings.";
        return false;
    }

    // Open settings related to gravity compensation (we need the estimator to be open)
    ok = this->loadFTSensorOffsetFromConfig(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in loading offline estimated ft offsets settings.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: Settings loaded in " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Opening RPC port.";
    tick = yarp::os::Time::now();

    // Open rpc port
    ok = this->openRPCPort();
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening rpc port.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: RPC port opened in " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Opening settings port.";
    tick = yarp::os::Time::now();

    // Open settings port
    ok = this->openSettingsPort();
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening settings port.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: Settings port opened in " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Opening remapper control board.";
    tick = yarp::os::Time::now();

    // Open the controlboard remapper
    ok = this->openRemapperControlBoard(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening controlboard remapper.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: Remapper control board opened in " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Opening remapper virtual sensors.";
    tick = yarp::os::Time::now();

     // Open the virtualsensor remapper
    ok = this->openRemapperVirtualSensors(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening virtual analog sensors remapper.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: Remapper virtual sensors opened in " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Opening contact frames.";
    tick = yarp::os::Time::now();

    ok = this->openContactFrames(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening default contact frame settings.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: Contact frames opened in " << yarp::os::Time::now() - tick << "s.";

    // Open the skin-related ports
    if (useSkinForContacts)
    {
        yDebug() << "wholeBodyDynamics Statistics: Opening skin contact list ports.";
        tick = yarp::os::Time::now();

        ok = this->openSkinContactListPorts(config);
        if( !ok )
        {
            yError() << "wholeBodyDynamics: Problem in opening skin-related port.";
            return false;
        }

        yDebug() << "wholeBodyDynamics Statistics: Skin contact list opened in " << yarp::os::Time::now() - tick << "s.";
    }

    yDebug() << "wholeBodyDynamics Statistics: Opening external wrenches ports.";
    tick = yarp::os::Time::now();

    // Open the ports related to publishing external wrenches
    ok = this->openExternalWrenchesPorts(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening external wrenches port.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: External port wrenches opened in " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Opening multiple analog sensors remapper.";
    tick = yarp::os::Time::now();

    // Open the multiple analog sensor remapper
    ok = this->openMultipleAnalogSensorRemapper(config);
    if( !ok )
    {
        yError() << "wholeBodyDynamics: Problem in opening multiple analog sensor remapper.";
        return false;
    }

    yDebug() << "wholeBodyDynamics Statistics: Multiple analog sensors remapper opened in " << yarp::os::Time::now() - tick << "s.";

    // Open the ports related to publishing filtered ft wrenches
    if (streamFilteredFT){
        yDebug() << "wholeBodyDynamics Statistics: Opening filtered FT ports.";
        tick = yarp::os::Time::now();

        ok = this->openFilteredFTPorts(config);
        if( !ok )
        {
            yError() << "wholeBodyDynamics: Problem in opening filtered ft ports.";
            return false;
        }

        yDebug() << "wholeBodyDynamics Statistics: Filtered FT ports opened in " << yarp::os::Time::now() - tick << "s.";
    }

    yDebug() << "wholeBodyDynamics Statistics: Configuration finished. Waiting attachAll to be called.";

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
    yInfo()<<"Starting attach MAS and analog ft";
    PolyDriverList ftSensorList;
    PolyDriverList tempSensorList;
    //std::vector<std::string>     tempDeviceNames;
    std::vector<IAnalogSensor *> ftList;
    std::vector<std::string>     ftDeviceNames;
    std::vector<std::string>     ftAnalogSensorNames;
    for(auto devIdx = 0; devIdx <p.size(); devIdx++)
    {
        ISixAxisForceTorqueSensors * fts = nullptr;
        ITemperatureSensors *tempS =nullptr;
        if( p[devIdx]->poly->view(fts) )
        {
            ftSensorList.push(const_cast<PolyDriverDescriptor&>(*p[devIdx]));
            ftDeviceNames.push_back(p[devIdx]->key);
        }
        // A device is considered an ft if it implements IAnalogSensor and has 6 channels
        IAnalogSensor * pAnalogSens = nullptr;
        if( p[devIdx]->poly->view(pAnalogSens) )
        {
            if( pAnalogSens->getChannels() ==static_cast<int>(wholeBodyDynamics_nrOfChannelsOfYARPFTSensor) )
            {
                ftList.push_back(pAnalogSens);
                ftDeviceNames.push_back(p[devIdx]->key);
                ftAnalogSensorNames.push_back(p[devIdx]->key);
            }
        }

        if( p[devIdx]->poly->view(tempS) )
        {
            tempSensorList.push(const_cast<PolyDriverDescriptor&>(*p[devIdx]));
           // tempDeviceNames.push_back(p[devIdx]->key);
        }
    }
    yDebug()<<"wholeBodyDynamicsDevice :: number of ft sensors found in both ft + mas"<<ftDeviceNames.size()<< "where analog are "<<ftList.size()<<" and mas are "<<ftSensorList.size();

    if( ftList.size() != estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) )
    {
        yError() << "wholeBodyDynamicsDevice : was expecting "
                 << estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)
                 << " from the model, but got " << ftList.size() << " FT sensor in the attach list.";
        return false;
    }

    // Attach the controlBoardList to the controlBoardRemapper
    bool ok = remappedMASInterfaces.multwrap->attachAll(ftSensorList);
    ok = ok & remappedMASInterfaces.multwrap->attachAll(tempSensorList);

    if( !ok  )
    {
        yError() << " WholeBodyDynamicsDevice::attachAll in attachAll of the remappedMASInterfaces";
        return false;
    }

    // Check if the MASremapper and the estimator have a consistent number of ft sensors
    int tempSensors = 0;
    tempSensors=static_cast<int>( remappedMASInterfaces.temperatureSensors->getNrOfTemperatureSensors());
    if( tempSensors > static_cast<int>( estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)) )
    {
        yError() << "wholeBodyDynamics : The multipleAnalogRemappedDevice has more sensors than those in the open estimator ft sensors list";
        return false;
    }

    int checkCounter=0;
    std::string ftName;
    std::string tempName;
    int ftMap=-1;
    for (int tSensor=0;tSensor<tempSensors;tSensor++){
        remappedMASInterfaces.temperatureSensors->getTemperatureSensorName(tSensor,tempName);
        int individualCheck=0;
        for (int ft=0;ft<static_cast<int>( estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)); ft++){
            ftName=estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName();
            if (tempName==ftName){
                individualCheck++;
                ftMap=ft;
            }
        }
        if (individualCheck!=1){
            yWarning()<< "wholeBodyDynamics : A temperature sensor name in multipleAnalogRemappedDevice do not match the name of the ft sensors";
            yDebug()<<"wholeBodyDynamics : Could not find or found multiple times sensor "<<tempName<<" among ft sensor names";
        }
        else {
            checkCounter++;
            // assuming the name is the same the order withing the mas interfaces might have a different id so we need a mapping
            ftTempMapping[static_cast<size_t>(ftMap)]=tSensor;
            yInfo()<< "wholeBodyDynamics: ftTempMapping "<< ftMap << " is ft position in model "<< tSensor <<"="<<ftTempMapping[ftMap] << " sensor name "<< tempName;
        }
    }
    if (checkCounter!=tempSensors){
        yError("wholeBodyDynamics : Not all temperature sensors have the same name as the ft sensors, expected %d , found %d",tempSensors,checkCounter);
        return false;
    }
    //End of temporary temperature check


    // Old check performed on ft sensors
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
            if( ftAnalogSensorNames[deviceIdx] == sensorName )
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

    if (!settings.disableSensorReadCheckAtStartup) {
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
            return false;
        }
    }

    return true;
}


bool WholeBodyDynamicsDevice::attachAllIMUs(const PolyDriverList& p)
{
    if (!useMasIMU)
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
    }
    else
    {
        for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
        {
            IThreeAxisLinearAccelerometers * pAcc{nullptr};
            if( p[devIdx]->poly->view(pAcc) )
            {
                if (pAcc->getNrOfThreeAxisLinearAccelerometers() != 1)
                {
                     yError() << "WholeBodyDynamicsDevice MAS IMU ERROR- Nr acc should be 1";
                    return false;
                }

                std::string accName;
                pAcc->getThreeAxisLinearAccelerometerName(0, accName);
                if (accName != masAccName)
                {
                    yError() << "WholeBodyDynamicsDevice MAS IMU ERROR- acc name mismatch";
                    return false;
                }

                masAccInterface = pAcc;
            }

            IThreeAxisGyroscopes * pGyro{nullptr};
            if( p[devIdx]->poly->view(pGyro) )
            {
                if (pGyro->getNrOfThreeAxisGyroscopes() != 1)
                {
                     yError() << "WholeBodyDynamicsDevice MAS IMU ERROR- Nr gyro should be 1";
                    return false;
                }

                std::string gyroName;
                pGyro->getThreeAxisGyroscopeName(0, gyroName);
                if (gyroName != masGyroName)
                {
                    yError() << "WholeBodyDynamicsDevice MAS IMU ERROR - gyro name mismatch";
                    return false;
                }

                masGyroInterface = pGyro;
            }
        }
    }
    if (!settings.disableSensorReadCheckAtStartup) {
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
            return false;
        }
    }

    return true;
}

bool WholeBodyDynamicsDevice::attachAll(const PolyDriverList& p)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    bool ok = true;
    yDebug() << "wholeBodyDynamics Statistics: attachAll started. Attaching all control board.";
    double tick = yarp::os::Time::now();

    ok = ok && this->attachAllControlBoard(p);
    yDebug() << "wholeBodyDynamics Statistics: Attaching all control board took " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Attaching all virtual analog sensors.";
    tick = yarp::os::Time::now();
    ok = ok && this->attachAllVirtualAnalogSensor(p);
    yDebug() << "wholeBodyDynamics Statistics: Attaching all virtual analog sensors took " << yarp::os::Time::now() - tick << "s.";

    yDebug() << "wholeBodyDynamics Statistics: Attaching all FTs.";
    tick = yarp::os::Time::now();
    ok = ok && this->attachAllFTs(p);
    yDebug() << "wholeBodyDynamics Statistics: Attaching all Fts took " << yarp::os::Time::now() - tick << "s.";


    if (settings.kinematicSource == IMU)
    {
        yDebug() << "wholeBodyDynamics Statistics: Attaching all IMUs.";
        tick = yarp::os::Time::now();
        ok = ok && this->attachAllIMUs(p);
        isIMUAttached = true;
        yDebug() << "wholeBodyDynamics Statistics: Attaching all IMUs took " << yarp::os::Time::now() - tick << "s.";

    }

    yDebug() << "wholeBodyDynamics Statistics: Calibrating offsets.";
    tick = yarp::os::Time::now();
    if (settings.startWithZeroFTSensorOffsets)
    {
        this->setFTSensorOffsetsToZero();
        validOffsetAvailable = true;
    }
    else
    {
        ok = ok && this->setupCalibrationWithExternalWrenchOnOneFrame("base_link",100);
    }
    yDebug() << "wholeBodyDynamics Statistics: Calibrating took " << yarp::os::Time::now() - tick << "s.";

    if( ok )
    {
        yDebug() << "wholeBodyDynamics Statistics: Starting";
        correctlyConfigured = true;
        this->start();
    }

    return ok;
}

void WholeBodyDynamicsDevice::setFTSensorOffsetsToZero()
{
    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        ftProcessors[ft].offset().zero();
    }
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
    bool TempSensorReadCorrectly = true;
    double timeFTStamp=yarp::os::Time::now();
    bool readTemperatureSensorThisTime=false;
    yarp::dev::MAS_status    sensorStatus;
    for(size_t ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::Wrench bufWrench;
        int ftRetVal = ftSensors[ft]->read(ftMeasurement);
        if (timeFTStamp-prevFTTempTimeStamp>checkTemperatureEvery_seconds){
            if (ftTempMapping[ft]!=-1){
                sensorStatus=remappedMASInterfaces.temperatureSensors->getTemperatureSensorStatus(ftTempMapping[ft]);
                std::string nameOfSensor;
                remappedMASInterfaces.temperatureSensors->getTemperatureSensorName(ftTempMapping[ft],nameOfSensor);


                if (sensorStatus==MAS_OK ){
                    TempSensorReadCorrectly=remappedMASInterfaces.temperatureSensors->getTemperatureSensorMeasure(ftTempMapping[ft],tempMeasurements[ft],timeFTStamp);
                     remappedMASInterfaces.temperatureSensors->getTemperatureSensorStatus(ftTempMapping[ft]);
                }
                else
                { //should be yError() but lets leave it like this for now
                    yInfo()<<"wholeBodyDynamics : Temp sensor "<< nameOfSensor << " return status "<< sensorStatus;
                }
                readTemperatureSensorThisTime=true;
            }
            else {
                tempMeasurements[ft]=0;
            }
        }
        bool ok = (ftRetVal == IAnalogSensor::AS_OK);

        FTSensorsReadCorrectly = FTSensorsReadCorrectly && ok && TempSensorReadCorrectly;

        if( !ok && verbose )
        {
            std::string sensorName = estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName();
            yWarning() << "wholeBodyDynamics warning : FT sensor " << sensorName << " was not readed correctly, using old measurement";
        }

        bool isNaN = false;
        for (size_t i = 0; i < ftMeasurement.size(); i++)
        {
            if( std::isnan(ftMeasurement[i]) ||  std::isnan(tempMeasurements[ft]))
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
    if (readTemperatureSensorThisTime) {
        prevFTTempTimeStamp=timeFTStamp;
    }
    return FTSensorsReadCorrectly;
}

bool WholeBodyDynamicsDevice::readIMUSensors(bool verbose)
{
    rawIMUMeasurements.angularAcc.zero();
    rawIMUMeasurements.linProperAcc.zero();
    rawIMUMeasurements.angularVel.zero();

    bool ok{false};
    if (!useMasIMU)
    {
        ok = imuInterface->read(imuMeasurement);

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
    }
    else
    {
        double time_stamp;
        yarp::sig::Vector acc(3), gyro(3);
        ok = masAccInterface->getThreeAxisLinearAccelerometerMeasure(0, acc, time_stamp);
        ok = masGyroInterface->getThreeAxisGyroscopeMeasure(0, gyro, time_stamp) && ok;
        rawIMUMeasurements.angularVel(0) = deg2rad(gyro[0]);
        rawIMUMeasurements.angularVel(1) = deg2rad(gyro[1]);
        rawIMUMeasurements.angularVel(2) = deg2rad(gyro[2]);

        rawIMUMeasurements.linProperAcc(0) = acc[0];
        rawIMUMeasurements.linProperAcc(1) = acc[1];
        rawIMUMeasurements.linProperAcc(2) = acc[2];
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

        iDynTree::Wrench rawFTMeasureWithOffsetRemoved  = ftProcessors[ft].filt(rawFTMeasure,tempMeasurements[ft]);

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

        iDynTree::toiDynTree(outputJointAcc,jointAcc);
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
    if (useSkinForContacts)
    {
        iCub::skinDynLib::skinContactList *scl =this->portContactsInput.read(false); //scl=null could also mean no new message
        if(scl)
        {
            //< \todo TODO check for envelope?
            lastReadingSkinContactListStamp = yarp::os::Time::now();
            if(scl->empty())   // if no skin contacts => leave the old contacts but reset pressure and contact list
            {
                if (contactsReadFromSkin.empty())
                {
                    //iterate for each added contact frame
                    for(size_t ovFrame = 0; ovFrame < contactFramesNames.size(); ovFrame++)
                    {
                        if(contactFramesIdxValidForSubModel[ovFrame] != iDynTree::FRAME_INVALID_INDEX)
                        {
                            bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                                    contactFramesIdxValidForSubModel[ovFrame], //frameIndex in iDynTree
                                                                                    unknownExtWrench[ovFrame]);
                            if( !ok )
                            {
                                yWarning() << "wholeBodyDynamics: Failing in adding override contact for submodel " << estimator.submodels().getSubModelOfFrame(estimator.model(),contactFramesIdxValidForSubModel[ovFrame]);
                            }
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
                //iterate for each added contact frame
                for(size_t ovFrame = 0; ovFrame < contactFramesNames.size(); ovFrame++)
                {
                    if(contactFramesIdxValidForSubModel[ovFrame] != iDynTree::FRAME_INVALID_INDEX)
                    {
                        bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                                contactFramesIdxValidForSubModel[ovFrame], //frameIndex in iDynTree
                                                                                unknownExtWrench[ovFrame]);
                        if( !ok )
                        {
                            yWarning() << "wholeBodyDynamics: Failing in adding override contact for submodel " << estimator.submodels().getSubModelOfFrame(estimator.model(),contactFramesIdxValidForSubModel[ovFrame]);
                        }
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
    }

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
            //iterate for each added contact frame
            for(size_t ovFrame = 0; ovFrame < contactFramesNames.size(); ovFrame++)
            {
                if(contactFramesIdxValidForSubModel[ovFrame] != iDynTree::FRAME_INVALID_INDEX)
                {
                    bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                            contactFramesIdxValidForSubModel[ovFrame], //frameIndex in iDynTree
                                                                            unknownExtWrench[ovFrame]);
                    if( !ok )
                    {
                        yWarning() << "wholeBodyDynamics: Failing in adding override contact for submodel " << subModel;
                    }
                }
            }
        }
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
                measuredRawFT = ftProcessors[ft].applySecondaryCalibrationMatrix(measuredRawFT,tempMeasurements[ft]);


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
            if (useSkinForContacts)
            {
                publishContacts();
            }

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
    _port.prepare()  = _values ;
    _port.write();
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
    }

    if (enablePublishNetExternalWrenches || this->outputWrenchPorts.size() > 0)
    {
        // Compute net wrenches for each link
        estimateExternalContactWrenches.computeNetWrenches(netExternalWrenchesExertedByTheEnviroment);
    }

    // Get wrenches from the estimator and publish it on the port
    for(size_t i=0; i < this->outputWrenchPorts.size(); i++ )
    {
        // Get the wrench in the link frame
        iDynTree::LinkIndex link = this->outputWrenchPorts[i].link_index;
        iDynTree::FrameIndex orientation = this->outputWrenchPorts[i].orientation_frame_index;
        iDynTree::FrameIndex origin      = this->outputWrenchPorts[i].origin_frame_index;

        //If contact wrenches are selected to be published from the configuration file
        if(m_outputWrenchPortInfoType == contactWrenches)
        {
            outputWrenchPorts[i].output_vector.clear();
            yarp::sig::Vector wrench_vector;

            for(int contId=0; contId<estimateExternalContactWrenches.getNrOfContactsForLink(link);contId++)
            {
                iDynTree::Wrench & link_f = estimateExternalContactWrenches.contactWrench(link,contId).contactWrench();

                // Transform the wrench in the desired frame
                iDynTree::Wrench pub_f = this->kinDynComp.getRelativeTransformExplicit(origin,orientation,link,link)*link_f;

                iDynTree::toYarp(pub_f,wrench_vector);
                for(int ele=0; ele<wrench_vector.size(); ele++)
                {
                    outputWrenchPorts[i].output_vector.push_back(wrench_vector[ele]);
                }
            }
        }
        //If net-wrench (default) are selected to be published from the configuration file
        else if(m_outputWrenchPortInfoType == netWrench)
        {
            iDynTree::Wrench & link_f = netExternalWrenchesExertedByTheEnviroment(link);

            // Transform the wrench in the desired frame
            iDynTree::Wrench pub_f = this->kinDynComp.getRelativeTransformExplicit(origin,orientation,link,link)*link_f;

            iDynTree::toYarp(pub_f,outputWrenchPorts[i].output_vector);
        }

        broadcastData<yarp::sig::Vector>(outputWrenchPorts[i].output_vector,
                                        *(outputWrenchPorts[i].output_port));
    }

    if (enablePublishNetExternalWrenches)
    {
        //Publish net wrenches
        for (iDynTree::LinkIndex link = 0;
             link < static_cast<iDynTree::LinkIndex>(netExternalWrenchesExertedByTheEnviroment.getNrOfLinks());
             ++link)
        {
            yarp::os::Bottle* wrenchPair = netExternalWrenchesBottle.get(link).asList();
            yarp::os::Bottle* linkWrenchBottle = wrenchPair->get(1).asList(); //The first value is the name, the second the wrench
            for (size_t i = 0; i < 6; ++i)
            {
                linkWrenchBottle->get(i) = yarp::os::Value(netExternalWrenchesExertedByTheEnviroment(link)(i));
            }
        }
        broadcastData<yarp::os::Bottle>(netExternalWrenchesBottle, netExternalWrenchesPort);
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
    this->remappedMASInterfaces.multwrap->detachAll();

    closeFilteredFTPorts();
    closeExternalWrenchesPorts();
    closeRPCPort();
    closeSettingsPort();
    if (useSkinForContacts)
    {
        closeSkinContactListsPorts();
    }


    return true;
}

bool WholeBodyDynamicsDevice::close()
{
    this->remappedControlBoard.close();
    this->remappedVirtualAnalogSensors.close();
    this->multipleAnalogRemappedDevice.close();

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
    // Let's configure the external forces that then are assume to be active on the robot while calibration on two links (assumed to be symmetric)

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

bool WholeBodyDynamicsDevice::setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1(const int32_t nrOfSamples)
{
    // Let's configure the external forces that then are assume to be active on the robot while calibration on two links (assumed to be symmetric)

    // Clear the class
    calibrationBuffers.assumedContactLocationsForCalibration.clear();

    // Check if the iRonCub-Mk1 jets frames exist
    std::string leftArmJetFrame = {"l_arm_jet_turbine"};
    std::string RightArmJetFrame = {"r_arm_jet_turbine"};
    std::string leftBackJetFrame = {"chest_l_jet_turbine"};
    std::string RightBackJetFrame = {"chest_r_jet_turbine"};
    iDynTree::FrameIndex frameLAIndex = estimator.model().getFrameIndex(leftArmJetFrame);
    if( frameLAIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1 impossible to find frame " << leftArmJetFrame << ", Are you using iRonCub-Mk1?";
        return false;
    }

    iDynTree::FrameIndex frameRAIndex = estimator.model().getFrameIndex(RightArmJetFrame);
    if( frameRAIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1 impossible to find frame " << RightArmJetFrame << ", Are you using iRonCub-Mk1?";
        return false;
    }

    iDynTree::FrameIndex frameLBIndex = estimator.model().getFrameIndex(leftBackJetFrame);
    if( frameLBIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1 impossible to find frame " << leftBackJetFrame << ", Are you using iRonCub-Mk1?";
        return false;
    }

    iDynTree::FrameIndex frameRBIndex = estimator.model().getFrameIndex(RightBackJetFrame);
    if( frameRBIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1 impossible to find frame " << RightBackJetFrame << ", Are you using iRonCub-Mk1?";
        return false;
    }

    // Check if the soles frames exist
    std::string LeftSoleFrame = {"l_sole"};
    std::string RightSoleFrame = {"r_sole"};
    iDynTree::FrameIndex frameLSIndex = estimator.model().getFrameIndex(LeftSoleFrame);
    if( frameLSIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1 impossible to find frame " << LeftSoleFrame;
        return false;
    }

    iDynTree::FrameIndex frameRSIndex = estimator.model().getFrameIndex(RightSoleFrame);
    if( frameRSIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1 impossible to find frame " << RightSoleFrame;
        return false;
    }

    // We assume that all the 6 contacts are Pure Forces with Known Directions (1D) at the origin of each frame
    iDynTree::Direction zPosAxis = iDynTree::Direction(0,0,1);
    iDynTree::Direction zNegAxis = iDynTree::Direction(0,0,-1);
    // Nominal Idle thrust values for JetCat P100 jet models = 2N
    // The negative sign is for the force direction represented in the Arm Jet frame
    double p100IdleThrustN = -7;
    // Nominal Idle thrust values for JetCat P220 jet models = 9N
    // The negative sign is for the force direction represented in the Back Jet frame
    double p220IdleThrustN = -9;
    iDynTree::Force jetArmForce = iDynTree::Force(0,0,p100IdleThrustN);
    iDynTree::Torque jetArmTorque = iDynTree::Torque(0,0,0);
    iDynTree::Wrench jetArmWrench = iDynTree::Wrench(jetArmForce, jetArmTorque);

    iDynTree::Force jetBackForce = iDynTree::Force(0,0,p220IdleThrustN);
    iDynTree::Torque jetBackTorque = iDynTree::Torque(0,0,0);
    iDynTree::Wrench jetBackWrench = iDynTree::Wrench(jetBackForce, jetBackTorque);

    iDynTree::UnknownWrenchContact calibrationAssumedArmJetContact(iDynTree::NO_UNKNOWNS,iDynTree::Position::Zero(),zPosAxis, jetArmWrench);
    iDynTree::UnknownWrenchContact calibrationAssumedBackJetContact(iDynTree::NO_UNKNOWNS,iDynTree::Position::Zero(),zPosAxis, jetBackWrench);

    // Add jets contacts
    bool ok = calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameLAIndex,calibrationAssumedArmJetContact);
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameRAIndex,calibrationAssumedArmJetContact);
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameLBIndex,calibrationAssumedBackJetContact);
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameRBIndex,calibrationAssumedBackJetContact);

    // Add feet contacts
    iDynTree::UnknownWrenchContact calibrationAssumedFeetContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameLSIndex,calibrationAssumedFeetContact);
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameRSIndex,calibrationAssumedFeetContact);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1 error";
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

bool WholeBodyDynamicsDevice::calibStandingWithJetsiRonCubMk1(const std::string& calib_code, const int32_t nr_of_samples)
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : calibStandingWithJetsiRonCubMk1 ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1(nr_of_samples);

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

    this->setFTSensorOffsetsToZero();

    return true;
}

bool WholeBodyDynamicsDevice::usePreEstimatedOffset()
{
    std::lock_guard<std::mutex> guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : using offset estimated offline.";

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        double norm = iDynTree::toEigen(ftProcessors[ft].estimatedOffset()).norm();
        if (norm!= 0.0){ // if norm is exactly 0.0 it means there was no offset  in the configuration file so it should be skipped
//TODO: think of a more robust way instead of norm=0.0
            //The offset estimated offline is meant to be added, while it is substracted in the six axis force torque  measure helpers, so we changed the sign here. There might be a better solution though.
            yInfo()<< " Value of pre estimated offset is (actual value used is '-' this value): "<< ftProcessors[ft].estimatedOffset().toString() << "for sensor in position "<<ft;
            ftProcessors[ft].offset()=-ftProcessors[ft].estimatedOffset();
        }
        else {
            yInfo()<< " Norm was exactly 0.0 for ft sensor in position "<<ft<< " so we are using offset calculated using calib all";
        }

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
    if (!isIMUAttached)
    {
        yError() << "wholeBodyDynamics : IMU was not attached during startup, cannot use IMU as kinematics source. ";
        return false;
    }

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
