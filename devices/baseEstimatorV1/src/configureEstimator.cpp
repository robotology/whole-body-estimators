/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <baseEstimatorV1.h>

bool yarp::dev::baseEstimatorV1::loadEstimatorParametersFromConfig(const yarp::os::Searchable& config)
{
    if (config.check("model_file") && config.find("model_file").isString())
    {
        m_model_file_name = config.find("model_file").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"model_file\" parameter in configuration file." <<
                                                     " Loading default file with name " << m_model_file_name;
    }

    if (config.check("robot") && config.find("robot").isString())
    {
        m_robot = config.find("robot").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"robot\" parameter in configuration file." <<
                                                     " Loading default robot name " << m_robot;
    }

    if (config.check("device_period_in_seconds") && config.find("device_period_in_seconds").isFloat64())
    {
        m_device_period_in_s = config.find("device_period_in_seconds").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"device_period_in_seconds\" parameter in configuration file." <<
                                                     " Loading default device period " << m_device_period_in_s;
    }

    if (!getJointNamesList(config, m_estimation_joint_names))
    {
        return false;
    }

    if (config.check("base_link") && config.find("base_link").isString())
    {
        m_base_link_name = config.find("base_link").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"base_link\" parameter in configuration file." <<
                                                     " Loading default robot name " << m_base_link_name;
    }

    if (config.check("left_foot_ft") && config.find("left_foot_ft").isString())
    {
        m_left_foot_ft = config.find("left_foot_ft").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"left_foot_ft\" parameter in configuration file." <<
                                                     " Loading default FT sensor name " << m_left_foot_ft;
    }

    if (config.check("right_foot_ft") && config.find("right_foot_ft").isString())
    {
        m_right_foot_ft = config.find("right_foot_ft").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"right_foot_ft\" parameter in configuration file." <<
                                                     " Loading default FT sensor name  " << m_right_foot_ft;
    }

    if (config.check("attitude_filter_type") && config.find("attitude_filter_type").isString())
    {
        m_attitude_estimator_type = config.find("attitude_filter_type").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"attitude_filter_type\" parameter in configuration file." <<
                                                     " Loading default attitude estimator type " << m_attitude_estimator_type;
    }

    if (config.check("initial_attitude_estimate_as_quaternion") && config.find("initial_attitude_estimate_as_quaternion").isList())
    {
        yarp::os::Bottle *attitude = config.find("initial_attitude_estimate_as_quaternion").asList();
        if (!attitude || attitude->size() != 4)
        {
            yWarning() << "floatingBaseEstimatorV1: " << "please mention \"initial_attitude_estimate_as_quaternion\" parameter as a list of 4 doubles.";
            yWarning() << "floatingBaseEstimatorV1: " << "uing default value for initial attitude estimate: " << m_initial_reference_frame_for_world;
        }

        m_initial_attitude_estimate_as_quaternion.resize(4);

        for (size_t i =0; i < 4; i++)
        {
            m_initial_attitude_estimate_as_quaternion(i) = attitude->get(i).asFloat64();
        }
    }
    else
    {
        m_initial_attitude_estimate_as_quaternion.resize(4);
        m_initial_attitude_estimate_as_quaternion.zero();
        m_initial_attitude_estimate_as_quaternion(0) = 1.0;
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"initial_attitude_estimate_as_quaternion\" parameter in configuration file." <<
                                                     "uing default value for initial attitude estimate: " << m_initial_attitude_estimate_as_quaternion.toString();
    }

    if (config.check("imu_confidence_roll") && config.find("imu_confidence_roll").isFloat64())
    {
        m_imu_confidence_roll = config.find("imu_confidence_roll").asFloat64();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"imu_confidence_roll\" parameter in configuration file." <<
                                                   " using default value " << m_imu_confidence_roll ;
    }

    if (config.check("imu_confidence_pitch") && config.find("imu_confidence_pitch").isFloat64())
    {
        m_imu_confidence_pitch = config.find("imu_confidence_pitch").asFloat64();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"imu_confidence_pitch\" parameter in configuration file." <<
                                                   " using default value " << m_imu_confidence_pitch ;
    }

    if (config.check("publish_debug_ports") && config.find("publish_debug_ports").isBool())
    {
        m_use_debug_ports = config.find("publish_debug_ports").asBool();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "debug mode set to set to " << m_use_debug_ports;
    }

    bool ok =  loadLeggedOdometryParametersFromConfig(config);
    ok = loadBipedFootContactClassifierParametersFromConfig(config) && ok;

    if (config.check("imu_name") && config.find("imu_name").isString())
    {
        m_imu_name = config.find("imu_name").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will get IMU measurements from default IMU with name: " << m_imu_name ;
    }

    if (config.check("head_imu_link") && config.find("head_imu_link").isString())
    {
        m_head_imu_link = config.find("head_imu_link").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Head IMU link is assumed as link with the name: " << m_head_imu_link ;
    }

    yarp::os::Value* upper_body_joint_names_list;
    if(!config.check("head_to_base_joints_list", upper_body_joint_names_list))
    {
        yError() <<  "floatingBaseEstimatorV1: " <<" Unable to find the head to base joints list";
        return false;
    }

    if(!YarpHelper::yarpListToStringVector(upper_body_joint_names_list, m_head_to_base_joints_list))
    {
        return false;
    }

    if (config.check("head_to_base_joints_list_zero_pos") && config.find("head_to_base_joints_list_zero_pos").isList())
    {
        yarp::os::Bottle* upper_body_joint_zero_pos;
        upper_body_joint_zero_pos = config.find("head_to_base_joints_list_zero_pos").asList();

        if (upper_body_joint_zero_pos == nullptr || upper_body_joint_zero_pos->size() != m_head_to_base_joints_list.size())
        {
            m_head_to_base_joints_list_zero_pos.resize(m_head_to_base_joints_list.size());
            m_head_to_base_joints_list_zero_pos.zero();
        }

        m_head_to_base_joints_list_zero_pos.resize(upper_body_joint_zero_pos->size());
        for (auto idx = 0; idx < upper_body_joint_zero_pos->size(); idx++)
        {
            m_head_to_base_joints_list_zero_pos(idx) = upper_body_joint_zero_pos->get(idx).asFloat64();
        }
    }
    else
    {
        m_head_to_base_joints_list_zero_pos.resize(m_head_to_base_joints_list.size());
        m_head_to_base_joints_list_zero_pos.zero();
    }

    if (m_attitude_estimator_type == "mahony")
    {
        ok = loadIMUAttitudeMahonyEstimatorParametersFromConfig(config) && ok;
    }
    else if (m_attitude_estimator_type == "qekf")
    {
        ok = loadIMUAttitudeQEKFParamtersFromConfig(config) && ok;
    }

    m_dump_data = config.check("dump_data", yarp::os::Value(false)).asBool();

    m_use_lpf = config.check("use_low_pass_filters", yarp::os::Value(false)).asBool();

    if (m_use_lpf)
    {
        m_joint_vel_filter_cutoff_freq = config.check("joint_vel_lpf_cutoff_freq", yarp::os::Value(0.0)).asFloat64();
        m_joint_vel_filter_sample_time_in_s = m_device_period_in_s;
    }

    return ok;
}

bool yarp::dev::baseEstimatorV1::loadLeggedOdometryParametersFromConfig(const yarp::os::Searchable& config)
{
    if (config.check("initial_fixed_frame") && config.find("initial_fixed_frame").isString())
    {
        m_initial_fixed_frame = config.find("initial_fixed_frame").asString();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"initial_fixed_frame\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    if (config.check("initial_reference_frame_for_world") && config.find("initial_reference_frame_for_world").isString())
    {
        m_initial_reference_frame_for_world = config.find("initial_reference_frame_for_world").asString();
    }
    else
    {
        m_initial_reference_frame_for_world = m_initial_fixed_frame;
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"initial_reference_frame_for_world\" parameter in configuration file." <<
                                                    "Setting same value as initial fixed frame, " << m_initial_reference_frame_for_world;
        m_initial_reference_frame_H_world.Identity();
    }

    if (config.check("initial_reference_frame_xyzrpy_pose_world") && config.find("initial_reference_frame_xyzrpy_pose_world").isList())
    {
        yarp::os::Bottle *pose = config.find("initial_reference_frame_xyzrpy_pose_world").asList();
        if (!pose || pose->size() != 6)
        {
            yWarning() << "floatingBaseEstimatorV1: " << "please mention \"initial_reference_frame_xyzrpy_pose_world\" parameter as a list of 6 doubles.";
            return false;
        }

        iDynTree::Position initial_reference_frame_p_world;
        initial_reference_frame_p_world(0) = pose->get(0).asFloat64();
        initial_reference_frame_p_world(1) = pose->get(1).asFloat64();
        initial_reference_frame_p_world(2) = pose->get(2).asFloat64();

        iDynTree::Rotation initial_reference_frame_R_world = iDynTree::Rotation::RPY(pose->get(3).asFloat64(),
                                                                                     pose->get(4).asFloat64(),
                                                                                     pose->get(5).asFloat64());

        m_initial_reference_frame_H_world = iDynTree::Transform(initial_reference_frame_R_world, initial_reference_frame_p_world);
    }
    else
    {
        m_initial_reference_frame_for_world = m_initial_fixed_frame;
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"initial_reference_frame_for_world\" parameter in configuration file." <<
                                                     " Setting same value as initial fixed frame, " << m_initial_reference_frame_for_world;
        m_initial_reference_frame_H_world.Identity();
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::loadBipedFootContactClassifierParametersFromConfig(const yarp::os::Searchable& config)
{
    if (config.check("initial_primary_foot") && config.find("initial_primary_foot").isString())
    {
        m_initial_primary_foot = config.find("initial_primary_foot").asString();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Could not find \"initial_primary_foot\" parameter in configuration file." <<
                                                   " Loading default initial primary foot " << m_initial_primary_foot;
    }

    if (config.check("schmitt_stable_contact_make_time") && config.find("schmitt_stable_contact_make_time").isFloat64())
    {
        m_left_foot_contact_schmitt_params.stableTimeContactMake = config.find("schmitt_stable_contact_make_time").asFloat64();
        m_right_foot_contact_schmitt_params.stableTimeContactMake = m_left_foot_contact_schmitt_params.stableTimeContactMake;
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"schmitt_stable_contact_make_time\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    if (config.check("schmitt_stable_contact_break_time") && config.find("schmitt_stable_contact_break_time").isFloat64())
    {
        m_left_foot_contact_schmitt_params.stableTimeContactBreak = config.find("schmitt_stable_contact_break_time").asFloat64();
        m_right_foot_contact_schmitt_params.stableTimeContactBreak = m_left_foot_contact_schmitt_params.stableTimeContactBreak;
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"schmitt_stable_contact_break_time\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    if (config.check("left_schmitt_contact_make_force_threshold") && config.find("left_schmitt_contact_make_force_threshold").isFloat64())
    {
        m_left_foot_contact_schmitt_params.contactMakeForceThreshold = config.find("left_schmitt_contact_make_force_threshold").asFloat64();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"left_schmitt_contact_make_force_threshold\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    if (config.check("left_schmitt_contact_break_force_threshold") && config.find("left_schmitt_contact_break_force_threshold").isFloat64())
    {
        m_left_foot_contact_schmitt_params.contactBreakForceThreshold = config.find("left_schmitt_contact_break_force_threshold").asFloat64();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"left_schmitt_contact_break_force_threshold\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    if (config.check("right_schmitt_contact_make_force_threshold") && config.find("right_schmitt_contact_make_force_threshold").isFloat64())
    {
        m_right_foot_contact_schmitt_params.contactMakeForceThreshold = config.find("right_schmitt_contact_make_force_threshold").asFloat64();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"right_schmitt_contact_make_force_threshold\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    if (config.check("right_schmitt_contact_break_force_threshold") && config.find("right_schmitt_contact_break_force_threshold").isFloat64())
    {
        m_right_foot_contact_schmitt_params.contactBreakForceThreshold = config.find("right_schmitt_contact_break_force_threshold").asFloat64();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"right_schmitt_contact_break_force_threshold\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::loadIMUAttitudeMahonyEstimatorParametersFromConfig(const yarp::os::Searchable& config)
{
    if (config.check("mahony_kp") && config.find("mahony_kp").isFloat64())
    {
        m_imu_attitude_observer_params.kp = config.find("mahony_kp").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default gain kp: " << m_imu_attitude_observer_params.kp ;
    }

    if (config.check("mahony_ki") && config.find("mahony_ki").isFloat64())
    {
        m_imu_attitude_observer_params.ki = config.find("mahony_ki").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default gain ki: " << m_imu_attitude_observer_params.ki ;
    }

    if (config.check("mahony_use_magnetometer") && config.find("mahony_use_magnetometer").isBool())
    {
        m_imu_attitude_observer_params.use_magnetometer_measurements = config.find("mahony_use_magnetometer").asBool();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "use magnetometer flag set to " << m_imu_attitude_observer_params.use_magnetometer_measurements;
    }

    if (config.check("mahony_discretization_time_step_in_seconds") && config.find("mahony_discretization_time_step_in_seconds").isFloat64())
    {
        m_imu_attitude_observer_params.time_step_in_seconds = config.find("mahony_discretization_time_step_in_seconds").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default discretization time step: " << m_imu_attitude_observer_params.time_step_in_seconds ;
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::loadIMUAttitudeQEKFParamtersFromConfig(const yarp::os::Searchable& config)
{
    if (config.check("qekf_discretization_time_step_in_seconds") && config.find("qekf_discretization_time_step_in_seconds").isFloat64())
    {
        m_imu_attitude_qekf_params.time_step_in_seconds = config.find("qekf_discretization_time_step_in_seconds").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default discretization time step: " << m_imu_attitude_qekf_params.time_step_in_seconds ;
    }

    if (config.check("qekf_accelerometer_noise_variance") && config.find("qekf_accelerometer_noise_variance").isFloat64())
    {
        m_imu_attitude_qekf_params.accelerometer_noise_variance = config.find("qekf_accelerometer_noise_variance").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default accelerometer noise variance: " << m_imu_attitude_qekf_params.accelerometer_noise_variance ;
    }

    if (config.check("qekf_magnetometer_noise_variance") && config.find("qekf_magnetometer_noise_variance").isFloat64())
    {
        m_imu_attitude_qekf_params.magnetometer_noise_variance = config.find("qekf_magnetometer_noise_variance").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default magnetometer noise variance: " << m_imu_attitude_qekf_params.magnetometer_noise_variance ;
    }

    if (config.check("qekf_gyroscope_noise_variance") && config.find("qekf_gyroscope_noise_variance").isFloat64())
    {
        m_imu_attitude_qekf_params.gyroscope_noise_variance = config.find("qekf_gyroscope_noise_variance").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default gyroscope noise variance: " << m_imu_attitude_qekf_params.gyroscope_noise_variance ;
    }

    if (config.check("qekf_gyro_bias_noise_variance") && config.find("qekf_gyro_bias_noise_variance").isFloat64())
    {
        m_imu_attitude_qekf_params.gyro_bias_noise_variance = config.find("qekf_gyro_bias_noise_variance").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default gyro bias noise variance: " << m_imu_attitude_qekf_params.gyro_bias_noise_variance ;
    }

    if (config.check("qekf_initial_orientation_error_variance") && config.find("qekf_initial_orientation_error_variance").isFloat64())
    {
        m_imu_attitude_qekf_params.initial_orientation_error_variance = config.find("qekf_initial_orientation_error_variance").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default initial state orientation variance: " << m_imu_attitude_qekf_params.initial_orientation_error_variance ;
    }

    if (config.check("qekf_initial_ang_vel_error_variance") && config.find("qekf_initial_ang_vel_error_variance").isFloat64())
    {
        m_imu_attitude_qekf_params.initial_ang_vel_error_variance = config.find("qekf_initial_ang_vel_error_variance").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default initial state angular velocity variance: " << m_imu_attitude_qekf_params.initial_ang_vel_error_variance ;
    }

    if (config.check("qekf_initial_gyro_bias_error_variance") && config.find("qekf_initial_gyro_bias_error_variance").isFloat64())
    {
        m_imu_attitude_qekf_params.initial_gyro_bias_error_variance = config.find("qekf_initial_gyro_bias_error_variance").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default initial state gyro bias variance: " << m_imu_attitude_qekf_params.initial_gyro_bias_error_variance ;
    }

    if (config.check("qekf_bias_correlation_time_factor") && config.find("qekf_bias_correlation_time_factor").isFloat64())
    {
        m_imu_attitude_qekf_params.bias_correlation_time_factor = config.find("qekf_bias_correlation_time_factor").asFloat64();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "Attitude estimator will use default bias forgetting factor: " << m_imu_attitude_qekf_params.bias_correlation_time_factor ;
    }

    if (config.check("qekf_use_magnetometer_measurements") && config.find("qekf_use_magnetometer_measurements").isBool())
    {
        m_imu_attitude_qekf_params.use_magnetometer_measurements = config.find("qekf_use_magnetometer_measurements").asBool();
    }
    else
    {
        yWarning() << "floatingBaseEstimatorV1: " << "qekf use magnetometer flag set to " << m_imu_attitude_qekf_params.use_magnetometer_measurements;
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::openComms()
{
    bool ok{false};
    ok = m_imu_attitude_observer_estimated_state_port.open(m_port_prefix + "/mahony_state/state:o");
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open port " << m_port_prefix + "/mahony_state/state:o";
        return false;
    }

    ok = m_imu_attitude_qekf_estimated_state_port.open(m_port_prefix + "/qekf/state:o");
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open port " << m_port_prefix + "/qekf/state:o";
        return false;
    }

    ok = m_floating_base_state_port.open(m_port_prefix + "/floating_base/configuration:o");
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open port " << m_port_prefix + "/floating_base/configuration:o";
        return false;
    }

    ok = m_floating_base_pose_port.open(m_port_prefix + "/floating_base/state:o");
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open port " << m_port_prefix + "/floating_base/state:o";
        return false;
    }

    ok = m_contact_state_port.open(m_port_prefix + "/feet_contact/state:o");
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open port " << m_port_prefix + "/feet_contact/state:o";
        return false;
    }

    floatingBaseEstimationRPC::yarp().attachAsServer(m_estimator_rpc_port);
    ok = m_estimator_rpc_port.open(m_port_prefix + "/rpc");
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open port " << m_port_prefix + "rpc";
        return false;
    }

    ok = m_com_port.open(m_port_prefix + "/center_of_mass/state:o");
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open port " << m_port_prefix + "/center_of_mass/state:o";
        return false;
    }

    return true;
}

