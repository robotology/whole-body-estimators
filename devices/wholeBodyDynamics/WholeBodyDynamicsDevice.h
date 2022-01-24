#ifndef CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H
#define CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H



// YARP includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/GenericSensorInterfaces.h>

// iCub includes
#include <iCub/skinDynLib/skinContactList.h>

// iDynTree includes
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>
#include <iDynTree/skinDynLibConversions.h>
#include <iDynTree/KinDynComputations.h>

// Filters
#include "ctrlLibRT/filters.h"

#include <wholeBodyDynamicsSettings.h>
#include <wholeBodyDynamics_IDLServer.h>
#include "SixAxisForceTorqueMeasureHelpers.h"
#include "GravityCompensationHelpers.h"

#include <vector>
#include <mutex>


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
 * For a detailed documentation of the parameters used within this device,
 * please see the README.md in the directory of this header file.
 *
 */
class WholeBodyDynamicsDevice :  public yarp::dev::DeviceDriver,
                                 public yarp::dev::IMultipleWrapper,
                                 public yarp::os::PeriodicThread,
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
      * Double to set how much time in seconds to wait before checking a new temperature value.
      * Default value is 0.55 because current temperature streaming frequency is set to 1 second.
      * In this way, we are sure we do not miss any change in the measurements.
      */
    double checkTemperatureEvery_seconds;

    /**
     * Flag to use tactil skin data to update contact points and related data
     */
    bool useSkinForContacts;

    /**
     * Flag to check if IMU was attached during startup
     */
    bool isIMUAttached;

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
        yarp::dev::IControlMode    * ctrlmode;
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

    /** Remapped multiple analog sensors containing the sensors that implement multiple analog sensor interfaces*/
    yarp::dev::PolyDriver multipleAnalogRemappedDevice;
    struct
    {
        yarp::dev::ITemperatureSensors * temperatureSensors;
        yarp::dev::ISixAxisForceTorqueSensors * ftMultiSensors;
        yarp::dev::IMultipleWrapper * multwrap;
    } remappedMASInterfaces;
    std::vector<int> ftTempMapping;
    double prevFTTempTimeStamp;

    /** IMU interface */
    yarp::dev::IGenericSensor * imuInterface;

    yarp::dev::IThreeAxisLinearAccelerometers* masAccInterface;
    yarp::dev::IThreeAxisGyroscopes* masGyroInterface;
    bool useMasIMU{true};
    std::string masAccName{"rfeimu_acc"};
    std::string masGyroName{"rfeimu_gyro"};

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
    std::mutex deviceMutex;

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
    bool openMultipleAnalogSensorRemapper(os::Searchable& config);
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
     * implements the IAnalogSensor interface. It also ataches MAS Six Axis F/T sensors.
     * A device is identified as a MAS Six Axis F/T if it
     * implements the Multiple Analog Sensor interfaces.
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
    bool loadTemperatureCoefficientsSettingsFromConfig(yarp::os::Searchable& config);
    bool loadFTSensorOffsetFromConfig(yarp::os::Searchable& config);
    bool applyLPFSettingsFromConfig(const yarp::os::Property& config, const std::string& setting_name);

    /**
     * Class actually doing computations.
     */
    iDynTree::ExtWrenchesAndJointTorquesEstimator estimator;

    /**
     * Buffers related methods
     */
    void resizeBuffers();

    /**
     * Reset related methods
     */
    void setFTSensorOffsetsToZero();

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
    yarp::sig::VectorOf<double>              tempMeasurements;

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
      * Calibrate the force/torque sensors when on double support and with jet engines turned ON and on idle thrust
      * (WARNING: works only with iRonCub-Mk1_1).
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the sole).
      * For this calibration the strong assumption of simmetry of the robot and its pose is done. Also, only pure forces are
      * assumed to be acting on the soles
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calibStandingWithJetsiRonCubMk1_1(const std::string& calib_code, const int32_t nr_of_samples = 100);

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
     * Use the sensor offset estimated offline.
     * @return true/false on success/failure
     * For now it checks if the norm of the offline offset is 0.0, which is the case for the default values when no offline offset was inserted in the configuration file.
     */
    virtual bool usePreEstimatedOffset();

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
    bool setupCalibrationWithVerticalForcesOnTheFeetAndJetsONiRonCubMk1_1(const int32_t nrOfSamples);

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
     std::vector<std::string> contactFramesNames;
     std::vector<std::string> contactWrenchType;
     std::vector<std::vector<double>> contactWrenchDirection;
     std::vector<std::vector<double>> contactWrenchPosition;
     bool overrideContactFramesSelected{false};
     std::vector<int> nrUnknownsInSubModel;
     std::vector<int> nrUnknownsInExtWrench;
     std::vector<iDynTree::UnknownWrenchContact> unknownExtWrench;
     std::vector<iDynTree::FrameIndex> contactFramesIdxValidForSubModel; //array of contact frames that don't cause nrUnknownsInSubModel > 6

     /**
      * Fills the variables `overrideContactFrames`, `contactWrenchType`, `contactWrenchDirection` and `contactWrenchPosition` in case the parameter `` exists in the configuration file.
      */
     bool parseOverrideContactFramesData(yarp::os::Bottle *_propOverrideContactFrames, yarp::os::Bottle *_propContactWrenchType, yarp::os::Bottle *_propContactWrenchDirection, yarp::os::Bottle *_propContactWrenchPosition);

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
     * An enumerator and a variable for storing the type of information
     * published in the YARP ports of the group
     * "WBD_OUTPUT_EXTERNAL_WRENCH_PORTS". It is either
     * "netWrench" or "contactWrenches"
     */
    enum outputWrenchPortInfoType {
        netWrench,
        contactWrenches
    };
    outputWrenchPortInfoType m_outputWrenchPortInfoType{netWrench};

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

    /**
     * The bottle to be sent containing the net external wrenches
     * applied on each link. The wrench applied to a link i is expressed
     * in the link frame. The bottle is a list of pairs. The first element is
     * the link name, the second is the wrench.
     */
    yarp::os::Bottle netExternalWrenchesBottle;

    /**
     * Port for streaming the netWrenchesBottle;
     */
    yarp::os::BufferedPort<yarp::os::Bottle> netExternalWrenchesPort;

    /**
     * Flag to publish the next external wrenches on each link.
     */
    bool enablePublishNetExternalWrenches{false};

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
