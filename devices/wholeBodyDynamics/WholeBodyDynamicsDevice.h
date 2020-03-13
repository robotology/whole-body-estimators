#ifndef CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H
#define CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H



// YARP includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

// iCub includes
#include <iCub/skinDynLib/skinContactList.h>

// iDynTree includes
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>
#include <iDynTree/iCub/skinDynLibConversions.h>
#include <iDynTree/KinDynComputations.h>

// Filters
#include "ctrlLibRT/filters.h"

#include <wholeBodyDynamicsSettings.h>
#include <wholeBodyDynamics_IDLServer.h>
#include "SixAxisForceTorqueMeasureHelpers.h"
#include "GravityCompensationHelpers.h"
#include "KnownExternalWrench.h"

#include <vector>
#include <array>


namespace yarp {
namespace dev {


/**
 * Scructure of information relative to an external force published
 * on a port.
 */
struct outputWrenchPortInformation
{
    std::string port_name;
    std::string link;
    std::string origin_frame;
    std::string orientation_frame;
    iDynTree::LinkIndex link_index;
    iDynTree::FrameIndex origin_frame_index;
    iDynTree::FrameIndex orientation_frame_index;
    yarp::sig::Vector output_vector;
    yarp::os::BufferedPort<yarp::sig::Vector> * output_port;
};


class wholeBodyDynamicsDeviceFilters
{
    public:

    wholeBodyDynamicsDeviceFilters();

    /**
     * Allocate the filters.
     */
    void init(int nrOfFTSensors,
              double initialCutOffForFTInHz,
              double initialCutOffForIMUInHz,
              int nrOfJointsProcessed,
              double initialCutOffForJointVelInHz,
              double initialCutOffForJointAccInHz,
              double periodInSeconds);

    void updateCutOffFrequency(double cutOffForFTInHz,
                               double cutOffForIMUInHz,
                               double cutOffForJointVelInHz,
                               double cutOffForJointAccInHz);
    /**
     * Deallocate the filters
     */
    void fini();


    ~wholeBodyDynamicsDeviceFilters();

    ///<  low pass filters for IMU linear accelerations
    iCub::ctrl::realTime::FirstOrderLowPassFilter * imuLinearAccelerationFilter;

    ///< low pass filters for IMU angular velocity
    iCub::ctrl::realTime::FirstOrderLowPassFilter * imuAngularVelocityFilter;

    ///< low pass filters for ForceTorque sensors
    std::vector<iCub::ctrl::realTime::FirstOrderLowPassFilter *> forcetorqueFilters;

    ///< low pass filter for Joint velocities
    iCub::ctrl::realTime::FirstOrderLowPassFilter * jntVelFilter;

    ///< low pass filter for Joint accelerations
    iCub::ctrl::realTime::FirstOrderLowPassFilter * jntAccFilter;

    ///< Yarp vector buffer of dimension 3
    yarp::sig::Vector bufferYarp3;

    ///< Yarp vector buffer of dimension 6
    yarp::sig::Vector bufferYarp6;

    ///< Yarp vector buffer of dimension dofs
    yarp::sig::Vector bufferYarpDofs;
};

/**
 * \section WholeBodyDynamicsDevice
 * A device that takes a list of axes and estimates the joint torques for each one of this axes.
 *
 *  The parameters taken in input by this device are:
 * | Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
 * |:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
 * | axesNames      |      -         | vector of strings |   -   |   -           | Yes      | Ordered list of the axes that are part of the remapped device.    |       |
 * | modelFile      |      -         | path to file      |   -   | model.urdf    | No       | Path to the URDF file used for the kinematic and dynamic model.   |       |
 * | assume_fixed    |                | frame name        |   -   |     -         | No       | If it is present, the initial kinematic source used for estimation will be that specified frame is fixed, and its gravity is specified by fixedFrameGravity. Otherwise, the default IMU will be used. | |
 * | fixedFrameGravity  |      -     | vector of doubles | m/s^2 | -             | Yes      | Gravity of the frame that is assumed to be fixed, if the kinematic source used is the fixed frame. | |
 * | imuFrameName   |       -        | string            |   -   |      -        | Yes      | Name of the frame (in the robot model) with respect to which the IMU broadcast its sensor measurements. |  |
 * | imuFilterCutoffInHz |     -     | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter IMU measures. | The used filter is a simple first order filter. |
 * | forceTorqueFilterCutoffInHz | - | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter FT measures.  |  The used filter is a simple first order filter. |
 * | jointVelFilterCutoffInHz    | - | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter joint velocities measures. | The used filter is a simple first order filter. |
 * | jointAccFilterCutoffInHz    | - | double            | Hz    |      -        | Yes      | Cutoff frequency of the filter used to filter joint accelerations measures. | The used filter is a simple first order filter. |
 * | defaultContactFrames      | -   | vector of strings (name of frames ) |-| - |  Yes     | Vector of default contact frames. If no external force read from the skin is found on a given submodel, the defaultContactFrames list is scanned and the first frame found on the submodel is the one at which origin the unknown contact force is assumed to be. | - |
 * | alwaysUpdateAllVirtualTorqueSensors | -     |  bool |  -    |      -        |  Yes     | Enforce that a virtual sensor for each estimated axes is available. | Tipically this is set to false when the device is running in the robot, while to true if it is running outside the robot. |
 * | defaultContactFrames |      -   | vector of strings |  -    |    -          | Yes      | If not data is read from the skin, specify the location of the default contacts | For each submodel induced by the FT sensor, the first not used frame that belongs to that submodel is selected from the list. An error is raised if not suitable frame is found for a submodel. |
 * | useJointVelocity     |        - | bool              |  -    |      true     |  No      | Select if the measured joint velocities (read from the getEncoderSpeeds method) are used for estimation, or if they should be forced to 0.0 . | The default value of true is deprecated, and in the future the parameter will be required. |
 * | useJointAcceleration |        - | bool              |  -    |      true     |  No      | Select if the measured joint accelerations (read from the getEncoderAccelerations method) are used for estimation, or if they should be forced to 0.0 . | The default value of true is deprecated, and in the future the parameter will be required. |
 * | streamFilteredFT     |        - | bool              |  -    |      false    |  No      | Select if the filtered and offset removed forces will be streamed or not. The name of the ports have the following syntax:  portname=(portPrefix+"/filteredFT/"+sensorName). Example: "myPrefix/filteredFT/l_leg_ft_sensor" | The value streamed by this ports is affected by the secondary calibration matrix, the estimated offset and temperature coefficients ( if any ). |
 * | IDYNTREE_SKINDYNLIB_LINKS |  -  | group             | -     | -             | Yes      |  Group describing the mapping between link names and skinDynLib identifiers. | |
 * |                |   linkName_1   | string (name of a link in the model) | - | - | Yes   | Bottle of three elements describing how the link with linkName is described in skinDynLib: the first element is the name of the frame in which the contact info is expressed in skinDynLib (tipically DH frames), the second a integer describing the skinDynLib BodyPart , and the third a integer describing the skinDynLib LinkIndex  | |
 * |                |   ...   | string (name of a link in the model) | - | -     | Yes      | Bottle of three elements describing how the link with linkName is described in skinDynLib: the first element is the name of the frame in which the contact info is expressed in skinDynLib (tipically DH frames), the second a integer describing the skinDynLib BodyPart , and the third a integer describing the skinDynLib LinkIndex  | |
 * |                |   linkName_n   | string (name of a link in the model) | - | - | Yes   | Bottle of three elements describing how the link with linkName is described in skinDynLib: the first element is the name of the frame in which the contact info is expressed in skinDynLib (tipically DH frames), the second a integer describing the skinDynLib BodyPart , and the third a integer describing the skinDynLib LinkIndex  | |
 * | WBD_OUTPUT_EXTERNAL_WRENCH_PORTS |  -  | group             | -     | -     | Yes       |  Group describing the external forces published on a YARP port by wholeBodyDynamics. | |
 * |                |   portName_1   | string (name of the port opened to stream the external wrench | - | - | Yes    | Bottle of three elements describing the wrench published on the port: the first element is the link of which the published external wrench is applied. This wrench is expressed around the origin of the frame named as second paramter, and with the orientation of the third parameter.  |  |
 * |                |   ...   | | - | ..                                        | Yes       | ..  |  |
 * |                |   portName_n   | .. | - | -                               | Yes       | ..  | |
 * | GRAVITY_COMPENSATION |  -       | group             | -     | -            | No        |  Group for providing estimates of the torque necessary to compensate gravity. | Gravity calls setImpedanceOffset when the considered joints is in COMPLIANT_INTERACTION_MODE   |
 * |                      | enableGravityCompensation | bool | -  | -           | No        |  |  |
 * |                      | gravityCompensationBaseLink| string | - | -         | No        | ..  | |
 * |                      | gravityCompensationAxesNames | vector of strings | - | - | No   | Axes for which the gravity compensation is published. | |
 *
 * The axes contained in the axesNames parameter are then mapped to the wrapped controlboard in the attachAll method, using controlBoardRemapper class.
 * Furthermore are also used to match the yarp axes to the joint names found in the passed URDF file.
 *
 * \subsection GravityCompensation
 * This device also provides gravity compensation torques (using the IImpedanceControl::setImpedanceOffset method)
 * for axis that are in compliant interaction mode and in position/position direct/velocity control mode.
 * This estimates are obtained just with the model, assuming that there is a link (the gravityCompensationRootLink)
 * at which all external forces are exerted.
 * Tipically this estimates are provided only for the upper joints (arms and torso) of the robots, as the gravity
 * compensation terms for the legs depends on the support state of the robot.
 *
 * \subsection SecondaryCalibrationMatrix
 * This device support to specify a secondary calibration matrix to apply on the top of the (already calibrated) measure coming from the F/T sensors.
 * This feature is meant to be experimental, and will be removed at any time.
 *
 * The group of options that regolated this group is the following:
 * | Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
 * |:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
 * | FT_SECONDARY_CALIBRATION |  -       | group         | -     | -             | No       |  Group for providing secondary calibration matrix for FT sensors. |   |
 * |                          | ftSensorName_1 | vector of 36 doubles| Unitless, 1/m or m depending on the element. | Identity matrix   | No  | Elements of the  6x6 secondary calibration matrix, specified in row-major order. | The messages coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |
 * |                          | ...            | vector of 36 doubles| Unitless, 1/m or m depending on the element. | Identity matrix   | No  | Elements of the  6x6 secondary calibration matrix, specified in row-major order. | The messages coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |
 * |                          | ftSensorName_n | vector of 36 doubles| Unitless, 1/m or m depending on the element. | Identity matrix   | No  | Elements of the  6x6 secondary calibration matrix, specified in row-major order. | The messages coming from the sensor will be *multiplied* by the specified matrix to get the actual measurement used.  |
 *
 * All sensors not specified will use a 6x6 identity as a secondary calibration matrix.
 *
 * Example of part of a configuration file using .xml yarprobotinterface format (remember to put the fractional dot!).
 * \code{.xml}
 *      <group name="FT_SECONDARY_CALIBRATION">
 *             <param name="l_arm_ft_sensor">(1.0,0.0,0.0,0.0,0.0,0.0,
 *                                            0.0,1.0,0.0,0.0,0.0,0.0,
 *                                            0.0,0.0,1.0,0.0,0.0,0.0,
 *                                            0.0,0.0,0.0,1.0,0.0,0.0,
 *                                            0.0,0.0,0.0,0.0,1.0,0.0,
 *                                            0.0,0.0,0.0,0.0,0.0,1.0)</param>
 *             <param name="r_arm_ft_sensor">(1.0,0.0,0.0,0.0,0.0,0.0,
 *                                            0.0,1.0,0.0,0.0,0.0,0.0,
 *                                            0.0,0.0,1.0,0.0,0.0,0.0,
 *                                            0.0,0.0,0.0,0.001,0.0,0.0,
 *                                            0.0,0.0,0.0,0.0,0.001,0.0,
 *                                            0.0,0.0,0.0,0.0,0.0,0.001)</param>
 *      </group>
 * \endcode
 *
 * \subsection Filters
 * All the filters used for the input measurements are using the iCub::ctrl::realTime::FirstOrderLowPassFilter class.
 *
 * \subsection ConfigurationExamples
 *
 * Example onfiguration file using .ini format.
 *
 * \code{.unparsed}
 *  device wholebodydynamics
 *  axesNames (joint1 joint2 joint3)
 *
 * ...
 * \endcode
 *
 * Example configuration file using .xml yarprobotinterface format.
 * \code{.xml}
 * <devices robot="iCubGenova02" build="1">
 *     <device name="wholebodydynamics" type="wholebodydynamics">
 *         <param name="axesNames">(torso_pitch,torso_roll,torso_yaw,neck_pitch, neck_roll,neck_yaw,l_shoulder_pitch,l_shoulder_roll,l_shoulder_yaw,l_elbow,r_shoulder_pitch,r_shoulder_roll,r_shoulder_yaw,r_elbow,l_hip_pitch,l_hip_roll,l_hip_yaw,l_knee,l_ankle_pitch,l_ankle_roll,r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)</param>
 *         <param name="modelFile">model.urdf</param>
 *         <param name="fixedFrameGravity">(0,0,-9.81)</param>
 *         <param name="defaultContactFrames">(l_hand,r_hand,root_link,l_sole,r_sole,l_lower_leg,r_lower_leg,l_elbow_1,r_elbow_1)</param>
 *         <param name="imuFrameName">imu_frame</param>
 *         <param name="useJointVelocity">true</param>
 *         <param name="useJointAcceleration">true</param>
 *         <!-- map between iDynTree links (identified by a name)
 *              and skinDynLib links (identified by their frame name, a BodyPart enum
 *              and a local (to the body part) index -->
 *         <group name="IDYNTREE_SKINDYNLIB_LINKS">
 *             <param name="root_link">(root_link,1,0)</param>
 *             <param name="chest"> (chest,1,2)</param>
 *             <param name="l_upper_arm">(l_upper_arm,3,2)</param>
 *             <param name="l_elbow_1">(l_elbow_1, 3, 4)</param>
 *             <param name="r_upper_arm">(r_upper_arm,4,2)</param>
 *             <param name="r_elbow_1">(r_elbow_1, 4, 4)</param>
 *             <param name="l_lower_leg">(l_lower_leg,5,3)</param>
 *             <param name="l_ankle_1">(l_ankle_1,5,4)</param>
 *             <param name="l_foot">(l_foot_dh_frame,5,5)</param>
 *             <param name="r_lower_leg">(r_lower_leg,6,3)</param>
 *             <param name="r_ankle_1">(r_ankle_1,6,4)</param>
 *             <param name="r_foot">(r_foot_dh_frame,6,5)</param>
 *         </group>
 *
 *         <group name="WBD_OUTPUT_EXTERNAL_WRENCH_PORTS">
 *             <param name="/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o">(l_foot,l_sole,root_link)</param>
 *             <param name="/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o">(r_foot,r_sole,root_link)</param>
 *         </group>
 *
 *         <action phase="startup" level="15" type="attach">
 *             <paramlist name="networks">
 *                 <!-- motorcontrol and virtual torque sensors -->
 *                 <elem name="left_lower_leg">left_lower_leg_mc</elem>
 *                 <elem name="right_lower_leg">right_lower_leg_mc</elem>
 *                 <elem name="left_upper_leg">left_upper_leg_mc</elem>
 *                 <elem name="right_upper_leg">right_upper_leg_mc</elem>
 *                 <elem name="torso">torso_mc</elem>
 *                 <elem name="right_lower_arm">right_lower_arm_mc</elem>
 *                 <elem name="left_lower_arm">left_lower_arm_mc</elem>
 *                 <elem name="right_upper_arm">right_upper_arm_mc</elem>
 *                 <elem name="left_upper_arm">left_upper_arm_mc</elem>
 *                 <elem name="head">head_mc</elem>
 *                 <!-- imu -->
 *                 <elem name="imu">inertial</elem>
 *                 <!-- ft -->
 *                 <elem name="l_arm_ft_sensor">left_upper_arm_strain</elem>
 *                 <elem name="r_arm_ft_sensor">right_upper_arm_strain</elem>
 *                 <elem name="l_leg_ft_sensor">left_upper_leg_strain</elem>
 *                 <elem name="r_leg_ft_sensor">right_upper_leg_strain</elem>
 *                 <elem name="l_foot_ft_sensor">left_lower_leg_strain</elem>
 *                 <elem name="r_foot_ft_sensor">right_lower_leg_strain</elem>
 *             </paramlist>
 *         </action>
 *
 *         <action phase="shutdown" level="2" type="detach" />
 *
 *     </device>
 * </devices>
 * \endcode 
 *
 */
class WholeBodyDynamicsDevice :  public yarp::dev::DeviceDriver,
                                 public yarp::dev::IMultipleWrapper,
                                 public yarp::os::RateThread,
                                 public wholeBodyDynamics_IDLServer
{
    struct imuMeasurements
    {
        iDynTree::Vector3 linProperAcc;
        iDynTree::Vector3 angularVel;
        iDynTree::Vector3 angularAcc;
    };

private:
    /**
     * Port prefix used for all the ports opened by wholeBodyDynamics.
     */
    std::string portPrefix;

    /**
     * Flag set to false at the beginning, and to true only if attachAll has been correctly called.
     */
    bool correctlyConfigured;

    /**
     * True if sensors have been read correctly, false
     * if sensors have never been read or if one of the sensor devices returned an error (such as a timeout).
     */
    bool sensorReadCorrectly;

    /**
     * Flag set to false at the beginning, and to true only if the estimation
     * have been performed correctly.
     */
    bool estimationWentWell;

    /**
     * Flag set to false initially, then to true as soon as a valid offset is available
     * (so torque estimation can start to be broadcasted).
     */
    bool validOffsetAvailable;

    /**
     * Double to keep track of last time skin was read
     * (to verify timeouts).
     */
    double  lastReadingSkinContactListStamp;

    /**
      * Flag set to false at the beginning, and true depending on configuration flag
      * If true it will stream the filtered ft sensor values
      */
    bool streamFilteredFT;

    /**
     * Names of the axis (joint with at least a degree of freedom) used in estimation.
     */
    std::vector<std::string> estimationJointNames;

    /** Remapped controlboard containg the axes for which the joint torques are estimated */
    yarp::dev::PolyDriver remappedControlBoard;
    struct
    {
        yarp::dev::IEncoders        * encs;
        yarp::dev::IMultipleWrapper * multwrap;
        yarp::dev::IImpedanceControl * impctrl;
        yarp::dev::IControlMode2    * ctrlmode;
        yarp::dev::IInteractionMode * intmode;
    } remappedControlBoardInterfaces;

    /** Remapped virtual analog sensor containg the axes for which the joint torques estimates are published */
    yarp::dev::PolyDriver remappedVirtualAnalogSensors;
    struct
    {
        yarp::dev::IVirtualAnalogSensor * ivirtsens;
        yarp::dev::IMultipleWrapper     * multwrap;
    } remappedVirtualAnalogSensorsInterfaces;

    /** F/T sensors interfaces */
    std::vector<yarp::dev::IAnalogSensor * > ftSensors;

    /** IMU interface */
    yarp::dev::IGenericSensor * imuInterface;

    /**
     * Setting for the whole body external wrenches and joint torques estimation.
     * Contained in a Thrift-generated structure to enable easy editing through
     * a YARP RPC port.
     */
    wholeBodyDynamicsSettings settings;
    wholeBodyDynamicsSettings::Editor settingsEditor;

    /**
     * Mutex to protect the settings data structure, and all the data in
     * the class that is accessed by the run method, the attachAll methods
     * (managed by the yarprobotinterface thread) and by the RPC call
     * invoked by the RPC thread.
     */
    yarp::os::Mutex deviceMutex;

    /**
     * A port for editing remotly the setting of wholeBodyDynamics
     */
    yarp::os::Port settingsPort;

    /**
     * Open-related methods
     */

    bool openSettingsPort();
    bool openRPCPort();
    bool openRemapperControlBoard(os::Searchable& config);
    bool openRemapperVirtualSensors(os::Searchable& config);
    bool openEstimator(os::Searchable& config);
    bool openContactFrames(os::Searchable& config);
    bool openSkinContactListPorts(os::Searchable& config);
    bool openExternalWrenchesPorts(os::Searchable& config);    
    bool openFilteredFTPorts(os::Searchable& config);

    /**
     * Close-related methods
     */
    bool closeSettingsPort();
    bool closeRPCPort();
    bool closeSkinContactListsPorts();
    bool closeExternalWrenchesPorts();
    bool closeFilteredFTPorts();

    /**
     * Attach-related methods
     */

    /**
     * Attach all controlboard devices.
     * A device is identified as a controlboard if it
     * implements the IEncoders interface.
     */
    bool attachAllControlBoard(const PolyDriverList& p);

    /**
     * Attach all virtual analog sensor device.
     * A device is identified as a virtual analog sensor if it
     * implements the IVirtualAnalogSensor interface.
     *
     * Furthermore, the IAxisInfo interface is used to get the
     * name of the axes for which virtual torque measurements are provided.
     */
    bool attachAllVirtualAnalogSensor(const PolyDriverList& p);

    /**
     * Attach all Six Axis Force/Torque devices.
     * A device is identified as a Six Axis F/T if it
     * implements the IAnalogSensor interface.
     *
     */
    bool attachAllFTs(const PolyDriverList& p);

    /**
     * Attach all IMU devices.
     * A device is identified as an IMU if it
     * implements the IGenericSensor interface
     *  OR (it implements
     * the IAnalogSensor interface AND it has 12 channels).
     * (The first is the case of FT sensors in the yarprobotinterface
     *  and the second is in the case of AnalogSensorClient).
     *
     */
    bool attachAllIMUs(const PolyDriverList& p);

    /**
     * Run-related methods.
     */

    /**
     * Return true if we were able to read the sensors and update
     * the internal buffers, false otherwise.
     */
    bool readFTSensors(bool verbose=true);

    /**
     * Return true if we were able to read the sensors and update
     * the internal buffers, false otherwise.
     */
    bool readIMUSensors(bool verbose=true);
    void readSensors();
    void filterSensorsAndRemoveSensorOffsets();
    void updateKinematics();
    void readContactPoints();
    void computeCalibration();
    void computeExternalForcesAndJointTorques();



    // Publish related methods
    void publishTorques();
    void publishContacts();
    void publishExternalWrenches();
    void publishEstimatedQuantities();
    void publishGravityCompensation();
    void publishFilteredFTWithoutOffset();

    /**
     * Load settings from config.
     */
    bool loadSettingsFromConfig(yarp::os::Searchable& config);
    bool loadSecondaryCalibrationSettingsFromConfig(yarp::os::Searchable& config);
    bool loadGravityCompensationSettingsFromConfig(yarp::os::Searchable & config);

    /**
     * Class actually doing computations.
     */
    iDynTree::ExtWrenchesAndJointTorquesEstimator estimator;

    /**
     * Buffers related methods
     */
    void resizeBuffers();

    /*
     * Buffers
     *
     */
    iDynTree::JointPosDoubleArray  jointPos;
    iDynTree::JointDOFsDoubleArray jointVel;
    iDynTree::JointDOFsDoubleArray jointAcc;
    yarp::sig::Vector              ftMeasurement;
    yarp::sig::Vector              imuMeasurement;
    yarp::sig::Vector              estimatedJointTorquesYARP;

    /***
     * Buffer for raw sensors measurements.
     */
    iDynTree::SensorsMeasurements  rawSensorsMeasurements;
    imuMeasurements                rawIMUMeasurements;

    /**
     * Filters
     */
    wholeBodyDynamicsDeviceFilters filters;

    /**
     * Buffer for filtered (both to reduce noise and remove offset) sensors.
     */
    iDynTree::SensorsMeasurements  filteredSensorMeasurements;
    imuMeasurements                filteredIMUMeasurements;

    iDynTree::LinkUnknownWrenchContacts measuredContactLocations;
    iDynTree::JointDOFsDoubleArray estimatedJointTorques;
    iDynTree::LinkContactWrenches  estimateExternalContactWrenches;

    /**
     * FT processing data structures
     */
    struct
    {
        bool ongoingCalibration;
        std::vector<bool> calibratingFTsensor;
        std::vector<iDynTree::Vector6> offsetSumBuffer;
        std::vector<iDynTree::Vector6> measurementSumBuffer;
        std::vector<iDynTree::Vector6> estimationSumBuffer;
        iDynTree::LinkUnknownWrenchContacts assumedContactLocationsForCalibration;
        iDynTree::SensorsMeasurements  predictedSensorMeasurementsForCalibration;
        iDynTree::JointDOFsDoubleArray predictedJointTorquesForCalibration;
        iDynTree::LinkContactWrenches  predictedExternalContactWrenchesForCalibration;
        size_t nrOfSamplesUsedUntilNowForCalibration;
        size_t nrOfSamplesToUseForCalibration;
    } calibrationBuffers;

    /**
     * Vector of classes used to process the raw FT measurements,
     * removing offset and using a secondary calibration matrix.
     */
    std::vector<wholeBodyDynamics::SixAxisForceTorqueMeasureProcessor> ftProcessors;

    /***
     * RPC Calibration related methods
     */
    /**
      * Calibrate the force/torque sensors
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calib(const std::string& calib_code, const int32_t nr_of_samples = 100);

     /**
      * Calibrate the force/torque sensors when on double support
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the sole).
      * For this calibration the strong assumption of simmetry of the robot and its pose is done.
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calibStanding(const std::string& calib_code, const int32_t nr_of_samples = 100);

     /**
      * Calibrate the force/torque sensors when on single support on left foot
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the left sole).
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples = 100);

     /**
      * Calibrate the force/torque sensors when on single support on right foot
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the right sole).
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples = 100);

     virtual bool calibStandingOnOneLink(const std::string& standing_frame, const int32_t nr_of_samples = 100);
     virtual bool calibStandingOnTwoLinks(const std::string& first_standing_frame, const std::string& second_standing_frame, const int32_t nr_of_samples = 100);

     /**
      * Reset the sensor offset to 0 0 0 0 0 0 (six zeros).
      * @param calib_code argument to specify the sensors to reset (all,arms,legs,feet)
      * @return true/false on success/failure
      */
     virtual bool resetOffset(const std::string& calib_code);

     /**
      * Quit the module.
      * @return true/false on success/failure
      */
     virtual bool quit();

     /**
      * Reset the odometry world to be (initially) a frame specified in the robot model,
      * and specify a link that is assumed to be fixed in the odometry.
      * @param initial_world_frame the frame of the robot model that is assume to be initially
      *        coincident with the world/inertial frame.
      * @param new_fixed_link the name of the link that should be initially fixed
      * @return true/false on success/failure (typically if the frame/link names are wrong)
      */
     virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link);

     /**
      * Change the link that is considered fixed by the odometry.
      * @param new_fixed_link the name of the new link that should be considered fixed
      * @return true/false on success/failure (typically if the frame/link names are wrong)
      */
     virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link);

     /**
      * Set the cutoff frequency (in Hz) for IMU measurements
      * @return true/false on success/failure
      */
     virtual bool set_imuFilterCutoffInHz(const double newCutoff);
      /**
      * Get the cutoff frequency (in Hz) for IMU measurements
      * @return the cutoff frequency (in Hz)
      */
      virtual double get_imuFilterCutoffInHz();
      /**
      * Set the cutoff frequency (in Hz) for FT measurements
      * @return true/false on success/failure
      */
      virtual bool set_forceTorqueFilterCutoffInHz(const double newCutoff);
     /**
      * Get the cutoff frequency (in Hz) for FT measurements
      * @return the cutoff frequency (in Hz)
      */
      virtual double get_forceTorqueFilterCutoffInHz();
      /**
      * Set the cutoff frequency (in Hz) for joint velocities measurements
      * @return true/false on success/failure
      */
      virtual bool set_jointVelFilterCutoffInHz(const double newCutoff);
      /**
      * Get the cutoff frequency (in Hz) for joint velocities measurements
      * @return the cutoff frequency (in Hz)
      */
      virtual double get_jointVelFilterCutoffInHz();
      /**
      * Set the cutoff frequency (in Hz) for joint acceleration measurements
      * @return true/false on success/failure
      */
      virtual bool set_jointAccFilterCutoffInHz(const double newCutoff);
      /**
      * Get the cutoff frequency (in Hz) for joint acceleration measurements
      * @return the cutoff frequency (in Hz)
      */
      virtual double get_jointAccFilterCutoffInHz();
      /**
       * Use the IMU as the kinematic source of
       * information for the acceleration of one link.
       */
      virtual bool useIMUAsKinematicSource();

      /**
       * Use a fixed frame (tipically root_link, l_sole or r_sole)
       * as the source of kinematic information. The assumption
       * is that the specified frame will remain fixed until
       * the kinematic source is changing, and the gravity
       * on this link is specified by the fixedFrameGravity (tipically
       * set to (0,0,-9.81) .
       */
      virtual bool useFixedFrameAsKinematicSource(const std::string& fixedFrame);

      /**
       * Set if to use or not the joint velocities in estimation.
       */
      virtual bool setUseOfJointVelocities(const bool enable);
      /**
       * Set if to use or not the joint velocities in estimation.
       */
      virtual bool setUseOfJointAccelerations(const bool enable);
      /**
       * Get the current settings in the form of a string.
       * @return the current settings as a human readable string.
       */
      virtual std::string getCurrentSettingsString();

    void setupCalibrationCommonPart(const int32_t nrOfSamples);
    bool setupCalibrationWithExternalWrenchOnOneFrame(const std::string & frameName, const int32_t nrOfSamples);
    bool setupCalibrationWithExternalWrenchesOnTwoFrames(const std::string & frame1Name, const std::string & frame2Name, const int32_t nrOfSamples);

     /**
      * RPC Calibration related attributes
      */
     yarp::os::Port  rpcPort;        // a port to handle rpc messages

     /**
      * Generic helper methods
      */
     size_t getNrOfFTSensors();
     void endCalibration();

     /**
      * List of frames in which a contact is assumed to be occuring
      * if no information about contacts is coming from the skin.
      * The order of the frames act as a priority:
      * the list is scanned if for a given subtree no contact is
      * specified, and a full wrench on the first suitable frame (i.e. frame belonging to the submodel)
      *  is added.
      */
     //std::string selectedContactFrames{"default"};
     std::vector<std::string> defaultContactFrames;
     std::vector<std::string> overrideContactFrames;
     std::vector<std::string> contactWrenchType;
     std::vector<std::vector<double>> contactWrenchDirection;
     std::vector<std::vector<double>> contactWrenchPosition;
     std::vector<iDynTree::FrameIndex> subModelIndex2DefaultContact;
     std::vector<std::vector<iDynTree::FrameIndex>> subModelIndex2OverrideContact;
     bool overrideContactFramesSelected{false};
     std::vector<int> subModelVarSize;

     /**
      * Port used to read the location of external contacts
      * obtained from the skin.
      */
     yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> portContactsInput;

     /**
      * \todo we should use directly the class in the BufferedPort portContactsInput
      */
     iCub::skinDynLib::skinContactList contactsReadFromSkin;

     /**
      * Port used to publish the external forces acting on the
      * robot.
      */
     yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> portContactsOutput;

     /**
      * \todo we should use directly the class in the BufferedPort portContactsOutput
      */
     iCub::skinDynLib::skinContactList contactsEstimated;

     /**
      * Helper to convert between iDynTree and skinDynLib related datastructures.
      */
     iDynTree::skinDynLibConversionsHelper conversionHelper;

     /**
      * External force methods.
      */
    // Data structures for wrenches to publish on individual external wrenches
    // (the complete external force information is published in the contacts:o
    //  port, but for backward compatibility we have to stream external wrenches
    //  informations on individual ports)
    std::vector< outputWrenchPortInformation > outputWrenchPorts;

    /**
     * Ports for streaming fitelerd ft data without offset
     */
    std::vector< std::unique_ptr <yarp::os::BufferedPort <yarp::sig::Vector> > > outputFTPorts;

    // Buffer for external forces
    /**
     * The element netExternalWrenchesExertedByTheEnviroment[i] is the
     * net external wrench excerted by the environment on the link i ,
     * expressed with the origin of link i and the orientation of link i.
     */
    iDynTree::LinkNetExternalWrenches netExternalWrenchesExertedByTheEnviroment;

    // Class for computing relative transforms (useful for net external wrench frame computations and gravity compensation)
    iDynTree::KinDynComputations kinDynComp;

    // Attributes for gravity compensation
    bool m_gravityCompensationEnabled;
    wholeBodyDynamics::GravityCompensationHelper m_gravCompHelper;
    std::vector<size_t> m_gravityCompesationJoints;
    iDynTree::JointDOFsDoubleArray m_gravityCompensationTorques;
    void resetGravityCompensation();

public:
    // CONSTRUCTOR
    WholeBodyDynamicsDevice();
    ~WholeBodyDynamicsDevice();

    // DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // IMULTIPLE WRAPPER
    virtual bool attachAll(const yarp::dev::PolyDriverList &p);
    virtual bool detachAll();

    // RATE THREAD
    virtual void run();
};

}
}

#endif /* CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H */

