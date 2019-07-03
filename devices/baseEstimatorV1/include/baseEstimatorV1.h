/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BASE_ESTIMATOR_V1_H
#define BASE_ESTIMATOR_V1_H

#include <iDynTree/Estimation/AttitudeMahonyFilter.h>
#include <iDynTree/Estimation/SimpleLeggedOdometry.h>
#include <iDynTree/Estimation/BipedFootContactClassifier.h>
#include <iDynTree/Estimation/AttitudeQuaternionEKF.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/IFrameTransform.h>
#include <memory>
#include <string>
#include <WalkingLogger.hpp>
#include <thrifts/floatingBaseEstimationRPC.h>

#include <iCub/ctrl/filters.h>

inline double deg2rad(const double angleInDeg)
{
    return angleInDeg*M_PI/180.0;
}

inline double rad2deg(const double angleInRad)
{
    return angleInRad*180.0/M_PI;
}

inline void convertVectorFromDegreesToRadians(iDynTree::VectorDynSize & vector)
{
    for(size_t i=0; i < vector.size(); i++)
    {
        vector(i) = deg2rad(vector(i));
    }

    return;
}

namespace yarp {
    namespace dev {


    class baseEstimatorV1 : public yarp::dev::DeviceDriver,
                                        public yarp::dev::IMultipleWrapper,
                                        public yarp::os::PeriodicThread,
                                        public floatingBaseEstimationRPC
    {
    public:
        explicit baseEstimatorV1(double period, yarp::os::ShouldUseSystemClock useSystemClock = yarp::os::ShouldUseSystemClock::No);
        baseEstimatorV1();
        ~baseEstimatorV1();

        /**
         * @brief Open the estimator device
         * @param[in] config Searchable object of configuration parameters
         * @return true/false success/failure
         */
        virtual bool open(yarp::os::Searchable& config);

        /**
         * @brief Close the estimator device
         * @return true/false success/failure
         */
        virtual bool close();

        /**
         * @brief Attach other required devices to the estimator. Calls other attach methods.
         * @param[in] p List of devices to be attached to the estimator
         * @return true/false success/failure
         */
        virtual bool attachAll(const yarp::dev::PolyDriverList& p);

        /**
         * @brief detach other devices from the estimator device
         *
         * @return bool
         */
        virtual bool detachAll();

        /**
         * @brief the periodic method that is called every update period,
         *        core of the estimator
         *
         * @return void
         */
        virtual void run();

    private:
        /**
         * @brief Load estimator device settings from configuration file
         *
         * @param[in] config parsed configuration file as a Searchable object
         * @return bool
         */
        bool loadEstimatorParametersFromConfig(const yarp::os::Searchable& config);

        /**
         * @brief Load legged odometry settings from configuration file
         *
         * @param[in] config parsed configuration file as a Searchable object
         * @return bool
         */
        bool loadLeggedOdometryParametersFromConfig(const yarp::os::Searchable& config);

        /**
         * @brief Load foot contact classifier settings from configuration file
         *
         * @param[in] config parsed configuration file as a Searchable object
         * @return bool
         */
        bool loadBipedFootContactClassifierParametersFromConfig(const yarp::os::Searchable& config);

        /**
         * @brief Load attitude estimator (Mahony-Hamel filter) settings from configuration file
         *
         * @param[in] config parsed configuration file as a Searchable object
         * @return bool
         */
        bool loadIMUAttitudeMahonyEstimatorParametersFromConfig(const yarp::os::Searchable& config);

        /**
         * @brief Load attitude estimator (Quaternion EKF) settings from configuration file
         *
         * @param[in] config parsed configuration file as a Searchable object
         * @return bool
         */
        bool loadIMUAttitudeQEKFParamtersFromConfig(const yarp::os::Searchable& config);

        /**
         * @brief Instantiates the transform broadcaster to publish the world to base transform
         *        over a ROS topic for RViz visualization
         *
         * @return bool
         */
        bool loadTransformBroadcaster();

        /**
         * @brief function taking in a pointer of read<Sensor>Func()
         *        to perform a dry run initial check of sensors
         *
         * @param[in] verbose verbose argument to be passed to function pointer
         * @param[in] func_t pointer of read<Sensor>Func()
         * @return true/false success/failure
         */
        bool  sensorReadDryRun(bool verbose, bool (baseEstimatorV1::*func_t)(bool));

        /**
         * @brief read IMU sensors
         *
         * @param[in] verbose verbose flag
         * @return true/false success/failure
         */
        bool readIMUSensors(bool verbose);

        /**
         * @brief read FT sensors
         *
         * @param[in] verbose verbose flag
         * @return true/false success/failure
         */
        bool readFTSensors(bool verbose);

        /**
         * @brief read Encoders
         *
         * @param[in] verbose verbose flag
         * @return true/false success/failure
         */
        bool readEncoders(bool verbose);

        /**
         * @brief parent function calling all read sensors methods
         *
         * @param[in] verbose verbose flag
         * @return true/false success/failure
         */
        bool readSensors(bool verbose);

        /**
         * @brief Loads SimpleLeggedOdometry, BipedFootContactClassifier and AttitudeMahonyFilter
         *        also KinDynComputations if necessary
         * @return true/false success/failure
         */
        bool loadEstimator();

        /**
         * @brief instantiate the legged odometry with loaded config parameters
         *
         * @return true/false success/failure
         */
        bool loadLeggedOdometry();

        /**
         * @brief instantiate the biped foot contact classifier with loaded config parameters
         *
         * @return true/false success/failure
         */
        bool loadBipedFootContactClassifier();

        /**
         * @brief instantiate the Mahony-Hamel filter with loaded config parameters
         *
         * @return true/false success/failure
         */
        bool loadIMUAttitudeMahonyEstimator();

        /**
         * @brief instantiate the Quaternion EKF with loaded config parameters
         *
         * @return true/false success/failure
         */
        bool loadIMUAttitudeQEKF();

        /**
         * @brief Attach all the control board units to the estimator
         * @param[in] p List of devices to be attached to the estimator
         * @return true/false success/failure
         */
        bool attachAllControlBoards(const yarp::dev::PolyDriverList &p);

        /**
         * @brief Attach all the inertial measurement units to the estimator
         * @param[in] p List of devices to be attached to the estimator
         * @return true/false success/failure
         */
        bool attachAllInertialMeasurementUnits(const yarp::dev::PolyDriverList &p);

        /**
         * @brief Attach all the 6 axis force-torque sensor units to the estimator
         * @param[in] p List of devices to be attached to the estimator
         * @return true/false success/failure
         */
        bool attachAllForceTorqueSensors(const yarp::dev::PolyDriverList &p);

        /**
         * @brief Attach the multiple analog sensor interface to the estimator
         * @param[in] p List of devices to be attached to the estimator
         * @return true/false success/failure
         */
        bool attachMultipleAnalogSensors(const yarp::dev::PolyDriverList &p);

        /**
         * @brief Allocating data buffers
         */
        void resizeBuffers();

        /**
         * @brief Get the list of joints to be loaded by the estimator
         *
         * @param[in] config parsed configuration file as a Searchable object
         * @param[out] joint_list list of joint names obtained from the configuration settings
         * @return bool
         */
        bool getJointNamesList(const yarp::os::Searchable& config,
                               std::vector<std::string>& joint_list);

        /**
         * @brief Open ports for publish and service calls
         * @return true/false success/failure
         */
        bool openComms();

        /**
         * @brief Close the estimator devicee
         */
        void closeDevice();

        /**
         * @brief initialize legged odometry
         * @return true/false success/failure
         */
        bool initializeLeggedOdometry();

        /**
         * @brief initialize biped foot contact classifier
         * @return true/false success/failure
         */
        bool initializeBipedFootContactClassifier();

        /**
         * @brief initialize Mahony-Hamel filter
         * @return true/false success/failure
         */
        bool initializeIMUAttitudeEstimator();

        /**
         * @brief initialize quaternion EKF
         * @return true/false success/failure
         */
        bool initializeIMUAttitudeQEKF();

        /**
         * @brief compute transform of IMU's world frame with estimator world frame
         * @return true/false success/failure
         */
        bool alignIMUFrames();

        /**
         * @brief Get world to base rotation from attitude estimation using IMU
         * @return iDynTree::Rotation return w_R_b computed using IMU rotation estimates
         */
        iDynTree::Rotation getBaseOrientationFromIMU();

        /**
         * @brief Get the home transform of the neck to IMU to be used for correcting the
         *        transform due to neck kinematics. this is useful only when using the head IMU
         * @note this method needs to be used only if using the head IMU, not applicable for the waist IMU
         * @return iDynTree::Transform return HEADIMU_H_NeckBaseAtHomePosition
         */
        iDynTree::Transform getHeadIMU_H_NeckBaseAtZero();

        /**
         * @brief Get the correction transform to be used for correcting the
         *        transform due to neck kinematics. this is useful only when using the head IMU
         * @note this method needs to be used only if using the head IMU, not applicable for the waist IMU
         * @return iDynTree::Transform return HEADIMU_H_NeckBase
         */
        iDynTree::Transform getHeadIMUCorrectionWithNeckKinematics();

        /**
         * @brief update the fixed frame and kinematic measurements for legged odometry and
         *        update the world to base transform computed through legged odometery
         * @return true/false success/failure
         */
        bool updateLeggedOdometry();

        /**
         * @brief  propagate the internal state of the Mahony-Hamel filter
         *         update the Mahony-Hamel filter with IMU measurements and
         * @return true/false success/failure
         */
        bool updateIMUAttitudeEstimator();

        /**
         * @brief propagate the internal state of the QEKF
         *        update the QEKF with IMU measurements and
         * @return true/false success/failure
         */
        bool updateIMUAttitudeQEKF();

        /**
         * @brief correct the orientation estimate of the Head IMU obtained from the
         *        attitude estimator with the correction transform HEADIMU_H_NeckBase
         * @note this method needs to be used only if using the head IMU, not applicable for the waist IMU
         * @return true/false success/failure
         */
        bool correctHeadIMUWithNeckKinematics();

        /**
         * @brief Fuses the world to base transform
         *        obtained from legged odometry and IMU attitude estimation
         * @return true/false success/failure
         */
        bool updateBasePoseWithIMUEstimates();

        /**
         * @brief compute the floating base velocity using
         *        fixed frame contact Jacobian and holonomic constraint dynamics
         * @return true/false success/failure
         */
        bool updateBaseVelocity();

        /**
         * @brief computes the floating base velocity using IMU measurements
         *        linear velocity is obtained through Euler-integration of linear acceleration
         *        obtained from the proper sensor acceleration of the IMU
         *        angular velocity is given by the attitude estimator
         * @return true/false success/failure
         */
        bool updateBaseVelocityWithIMU();

        /**
         * @brief parent method for other publish methods
         */
        void publish();

        /**
         * @brief publish internal state of attitude estimator through YARP port
         *        roll pitch yaw omegax omegay omegaz gyrobiasx gyrobiasy gyrobiasz
         */
        void publishIMUAttitudeEstimatorStates();

        /**
         * @brief publish internal state of QEKF through YARP port
         *        roll pitch yaw
         */
        void publishIMUAttitudeQEKFEstimates();

        /**
         * @brief publish floating base state through YARP port
         *        x y z roll pitch yaw joint_positions
         *        roll pitch yaw
         */
        void publishFloatingBaseState();

        /**
         * @brief publish floating base pose and velocity through YARP port
         *        x y z roll pitch yaw vx vy vz omegax omegay omegaz
         */
        void publishFloatingBasePoseVelocity();

        /**
         * @brief publish feet contact state through YARP port
         *        fz_lf fz_rf state_lf state_rf fixed_frame
         */
        void publishContactState();

        /**
         * @brief publish world to base tranform to /tf ROS topic using IFrameTransform interface
         */
        void publishTransform();

        /**
         * @brief configure whole body dynamics device and
         *        get the output ports to connect to in order
         *        to get the end-effector contact wrenches
         * @return true/false success/failure
         */
        bool configureWholeBodyDynamics(const yarp::os::Searchable& config);

        /**
         * @brief calibrate the force-torque sensor offsets through a RPC call to wholebodydynamics
         *        this calibration is necessary for proper estimation of contact wrenches
         *        the calibration is internally taken care of by the wholebodydynamics device
         * @return true/false success/failure
         */
        bool calibFTSensorsStanding();

        /**
         * @brief get the foot contact normal forces
         */
        void getFeetCartesianWrenches();

        /**
         * @brief read the foot contact wrenches from the output ports of the wholebodydynamics device
         * @return true/false success/failure
         */
        bool readWholeBodyDynamicsContactWrenches(bool verbose);

        /**
         * @brief initialize logger with the necessary variables to be logged
         * @return true/false success/failure
         */
        bool initializeLogger();

        /**
         * @brief update the logger
         * @return true/false success/failure
         */
        bool updateLogger();

        // RPC methods
        virtual std::string getEstimationJointsList();
        virtual bool setMahonyKp(const double kp);
        virtual bool setMahonyKi(const double ki);
        virtual bool setMahonyTimeStep(const double timestep);
        virtual bool setContactSchmittThreshold(const double l_fz_break, const double l_fz_make,
                                                const double r_fz_break, const double r_fz_make);
        virtual bool setPrimaryFoot(const std::string& primary_foot);
        virtual std::string getRefFrameForWorld();
        virtual Pose6D getRefPose6DForWorld();
        virtual bool resetLeggedOdometry();
        virtual bool resetLeggedOdometryWithRefFrame(const std::string& ref_frame,
                                                    const double x, const double y, const double z,
                                                    const double roll, const double pitch, const double yaw);
        virtual bool startFloatingBaseFilter();
        virtual bool useJointVelocityLPF(const bool flag);
        virtual bool setJointVelocityLPFCutoffFrequency(const double freq);

        // device options
        bool m_verbose{true}; ///< verbose outputs
        std::string m_port_prefix{"/base-estimator"};

        yarp::os::Mutex m_device_mutex; ///< mutex to avoid resource clash

        // status flags
        bool m_device_initialized_correctly{false};
        bool m_legged_odometry_update_went_well{false};
        bool m_use_debug_ports{true};

        // configuration parameters
        enum class FilterFSM{IDLE, CONFIGURED, RUNNING};
        FilterFSM m_state{FilterFSM::IDLE};
        double m_device_period_in_s{0.01}; ///< Thread period of the estimator device in seconds
        std::string m_model_file_name{"model.urdf"}; ///< URDF file name of the robot
        std::string m_robot{"icubSim"}; ///< name of the robot to identify between real and sim
        std::vector<std::string> m_estimation_joint_names; ///< list of joints used for estimation
        std::string m_base_link_name{"root_link"}; ///< floating base link
        std::string m_initial_fixed_frame; ///< initial fixed frame for legged odometry
        std::string m_initial_reference_frame_for_world; ///< frame in which initial world is expressed
        iDynTree::Transform m_initial_reference_frame_H_world; ///< pose of the world w.r.t initial reference frame
        iDynTree::SchmittParams m_left_foot_contact_schmitt_params, m_right_foot_contact_schmitt_params; ///< contact Schmitt trigger parameters for the feet
        std::string m_initial_primary_foot{"left"}; ///< initial primary foot for the contact classifier
        iDynTree::AttitudeMahonyFilterParameters m_imu_attitude_observer_params; ///< parameters for the attitude observer
        iDynTree::AttitudeQuaternionEKFParameters m_imu_attitude_qekf_params;
        std::string m_head_imu_name{"head_imu_acc_1x1"};

        // robot model and sensors
        iDynTree::Model m_model; ///< iDynTree object of loaded robot model
        iDynTree::SensorsList m_sensors_list; ///< iDynTree object of loaded sensors list from URDF

        const double m_sensor_timeout_in_seconds{2.0}; ///< Timeout to check for sensor measurements during dry run initial check
        const size_t m_nr_of_channels_in_YARP_IMU_sensor{12}; ///< Number of channels available in YARP IMU sensor output port
        const size_t m_nr_of_channels_in_YARP_FT_sensor{6}; ///< Number of channels available in YARP FT sensor output port
        bool m_use_multiple_analog_sensor_interface{false}; ///< Flag to switch between analog sensor interface or multiple analog sensor interface

        iDynTree::JointPosDoubleArray m_joint_positions; ///< joint positions array
        iDynTree::VectorDynSize m_joint_velocities; ///< joint velocities array

        // struct maintaining the IMU measurement serialization
        struct IMU_measurements
        {
            iDynTree::Vector3 linear_proper_acceleration;
            iDynTree::Vector3 angular_velocity;
            iDynTree::Vector3 angular_acceleration;
            bool sensor_status{true};
            std::string sensor_name;
        };

        // struct for the remapped control board interfaces
        struct
        {
            // cannot be a unique_ptr because it is just a reference to a resource owned by someone else
           yarp::dev::IEncoders *encs{nullptr};
        }m_remapped_control_board_interfaces;

        iDynTree::SensorsMeasurements m_sensor_measurements;

        std::vector<yarp::dev::IGenericSensor*> m_whole_body_imu_interface; ///< generic sensor interface for maintaining IMU sensors across the whole body
        size_t m_nr_of_IMUs_detected{0}; ///< number of IMUs attached to the estimator device
        std::vector<IMU_measurements> m_raw_IMU_measurements; ///< a vector of IMU measurements associated to the vector of IMU sensors

        std::vector<yarp::dev::IAnalogSensor*> m_whole_body_forcetorque_interface; ///< analog sensor interface for maintaining FT seneor across whole body
        size_t m_nr_of_forcetorque_sensors_detected{0}; ///< number of FT sensors attached to the estimator device

        // YARP Buffers
        std::vector<yarp::sig::Vector> m_imu_meaaurements_from_yarp_server; ///< YARP buffer for IMU measuremnts coming from different IMU sensors
        yarp::sig::Vector m_ft_measurements_from_yarp_server; ///< YARP buffer for FT measuremnts coming from different FT sensors

        // Estimation interfaces
        std::unique_ptr<iDynTree::SimpleLeggedOdometry> m_legged_odometry;  ///< legged odometry
        std::unique_ptr<iDynTree::BipedFootContactClassifier> m_biped_foot_contact_classifier; ///< foot contact classifier based on an internal contact force Schmitt Trigger
        std::unique_ptr<iDynTree::AttitudeMahonyFilter> m_imu_attitude_observer; ///< attitude observer to estimate IMU orientation from IMU measurements
        std::unique_ptr<iDynTree::AttitudeQuaternionEKF> m_imu_attitude_qekf;

        std::string m_attitude_estimator_type{"qekf"};
        iDynTree::VectorDynSize m_initial_attitude_estimate_as_quaternion;
        double m_imu_confidence_roll{0.5};
        double m_imu_confidence_pitch{0.5};

        yarp::os::BufferedPort<yarp::os::Bottle> m_floating_base_state_port; ///< port to publish floating base pose plus joint positions
        yarp::os::BufferedPort<yarp::os::Bottle> m_floating_base_pose_port; ///< port to publish floating base pose
        yarp::os::BufferedPort<yarp::os::Bottle> m_contact_state_port; ///< port to publish (fzleft, fzright, left_contact, right_contact, fixedLinkIndex)
        yarp::os::BufferedPort<yarp::os::Bottle> m_imu_attitude_observer_estimated_state_port; ///< port to publish rotation as RPY and IMU gyro bias
        yarp::os::BufferedPort<yarp::os::Bottle> m_imu_attitude_qekf_estimated_state_port; ///< port to publish rotation as RPY and IMU gyro bias
        yarp::os::Port m_estimator_rpc_port; ///< RPC port for service calls

        iDynTree::RPY m_imu_attitude_estimate_as_rpy;

        iDynTree::Rotation m_head_imu_calibration_matrix;
        iDynTree::Transform m_imu_H_neck_base_at_zero;
        bool m_imu_aligned{false};

        std::string m_current_fixed_frame; ///< current frame associated to the fixed link in legged odometery
        std::string m_previous_fixed_frame; ///< previous frame associated to the fixed link in legged odometery
        bool m_no_foot_in_contact{false}; ///< flag to check if contact on both feet is lost
        double m_left_foot_contact_normal_force, m_right_foot_contact_normal_force; ///< foot contact force z-direction

        yarp::sig::Vector m_world_pose_base_in_R6; ///< 6D vector pose of floating base frame in world reference frame
        yarp::sig::Matrix m_world_H_base;  ///< Homogeneous transformation matrix from base to world reference frame
        yarp::sig::Vector m_world_velocity_base; ///< 6D vector velocity of floating base frame in the world reference frame
        yarp::sig::Vector m_world_velocity_base_from_imu;

        std::string m_left_foot_ft_sensor{"l_foot_ft_sensor"};
        unsigned int m_left_foot_ft_sensor_index, m_right_foot_ft_sensor_index;
        std::string m_right_foot_ft_sensor{"r_foot_ft_sensor"};
        std::string m_right_sole{"r_sole"};
        std::string m_left_sole{"l_sole"};
        iDynTree::Rotation m_l_sole_R_l_ft_sensor, m_r_sole_R_r_ft_sensor;
        iDynTree::KinDynComputations m_kin_dyn_comp;

        std::string m_left_foot_cartesian_wrench_port_name, m_right_foot_cartesian_wrench_port_name;
        yarp::os::BufferedPort<yarp::sig::Vector> m_left_foot_cartesian_wrench_wbd_port, m_right_foot_cartesian_wrench_wbd_port;
        yarp::sig::Vector m_left_foot_cartesian_wrench, m_right_foot_cartesian_wrench;
        bool m_wbd_is_open{false};

        yarp::dev::PolyDriver  m_transform_broadcaster;
        yarp::dev::IFrameTransform *m_transform_interface{nullptr};

        std::unique_ptr<WalkingLogger> m_logger;
        bool m_dump_data{false};

        std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_joint_velocities_filter;
        double m_joint_vel_filter_cutoff_freq{0.0};
        double m_joint_vel_filter_sample_time_in_s{0.0};
        bool m_use_lpf{false};
    };
    }
}

#endif
