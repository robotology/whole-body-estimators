/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <baseEstimatorV1.h>


bool  yarp::dev::baseEstimatorV1::sensorReadDryRun(bool verbose, bool (yarp::dev::baseEstimatorV1::*func_t)(bool))
{
    double tic{yarp::os::Time::now()};
    double time_elapsed_trying_to_read_sensors{0.0};
    bool read_success{false};

    while ((time_elapsed_trying_to_read_sensors < m_sensor_timeout_in_seconds) && !read_success)
    {
        read_success = (this->*func_t)(verbose);
        time_elapsed_trying_to_read_sensors = (yarp::os::Time::now() - tic);
    }

    if (!read_success)
    {
        yError() << "floatingBaseEstimatorV1: " <<  "unable to read from sensor correctly for " << m_sensor_timeout_in_seconds <<  " seconds.. exiting.";
    }
    return read_success;
}

bool yarp::dev::baseEstimatorV1::attachAll(const yarp::dev::PolyDriverList& p)
{
    yarp::os::LockGuard guard(m_device_mutex);
    if (!attachAllControlBoards(p))
    {
        yError() << "baseEstimatorV1: " <<  "Could not attach the control boards";
        return false;
    }

    if (!loadEstimator())
    {
        yError() << "floatingBaseEstimatorV1: " <<  "Could not load estimator";
        return false;
    }

    if (m_use_multiple_analog_sensor_interface)
    {
        if (!attachMultipleAnalogSensors(p))
        {
           yError() << "floatingBaseEstimatorV1: " <<  "Could not attach the multiple analog sensor interface";
           return false;
        }
    }
    else
    {
        if (!attachAllInertialMeasurementUnits(p))
        {
           yError() << "floatingBaseEstimatorV1: " <<  "Could not attach the inertial measurement units";
           return false;
        }

        if (!attachAllForceTorqueSensors(p))
        {
           yError() << "floatingBaseEstimatorV1: " <<  "Could not attach the force-torque sensors";
           return false;
        }
    }

    start();
    return true;
}

bool yarp::dev::baseEstimatorV1::attachAllControlBoards(const yarp::dev::PolyDriverList& p)
{
    bool ok{false};
    for (size_t dev_idx = 0; dev_idx < (size_t)p.size(); dev_idx++)
    {
        ok = p[dev_idx]->poly->view(m_remapped_control_board_interfaces.encs);
        if (ok)
        {
            break;
        }
    }

    if (!ok)
    {
        return false;
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::loadEstimator()
{
    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string model_file_path = rf.findFileByName(m_model_file_name);
    yInfo() << "floatingBaseEstimatorV1: " <<  "Loading model from " + model_file_path;

    iDynTree::ModelLoader model_loader; ///< iDynTree object to load robot model
    bool ok = model_loader.loadReducedModelFromFile(model_file_path, m_estimation_joint_names);
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " <<  "Could not load model from specified path.";
        return false;
    }

    m_model = model_loader.model();
    m_sensors_list = model_loader.sensors();
    m_sensor_measurements.resize(m_sensors_list);

    m_kin_dyn_comp.loadRobotModel(m_model);

    resizeBuffers();
    setPeriod(m_device_period_in_s);

    ok = m_sensors_list.getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE, m_left_foot_ft_sensor, m_left_foot_ft_sensor_index) && ok;
    ok = m_sensors_list.getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE, m_right_foot_ft_sensor, m_right_foot_ft_sensor_index) && ok;

    m_r_sole_R_r_ft_sensor = m_kin_dyn_comp.getRelativeTransform(m_model.getFrameIndex(m_right_sole), m_right_foot_ft_sensor_index).getRotation();
    m_l_sole_R_l_ft_sensor = m_kin_dyn_comp.getRelativeTransform(m_model.getFrameIndex(m_left_sole), m_left_foot_ft_sensor_index).getRotation();

    ok = loadLeggedOdometry();
    ok = loadBipedFootContactClassifier() && ok;
    if (m_attitude_estimator_type == "mahony")
    {
        ok = loadIMUAttitudeMahonyEstimator() && ok;
    }
    else if (m_attitude_estimator_type == "qekf")
    {
        ok = loadIMUAttitudeQEKF() && ok;
    }
    ok = loadTransformBroadcaster() && ok;

    if (!ok)
    {
        return false;
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::attachMultipleAnalogSensors(const yarp::dev::PolyDriverList& p)
{
    return true;
}

bool yarp::dev::baseEstimatorV1::attachAllForceTorqueSensors(const yarp::dev::PolyDriverList& p)
{
    std::vector<yarp::dev::IAnalogSensor* > ft_sensor_list;
    std::vector<std::string> ft_sensor_name;
    for (size_t dev_idx = 0; dev_idx < (size_t)p.size(); dev_idx++)
    {
        // check if an analog sensor has 6 channels implying it is a forcetorque sensor
        yarp::dev::IAnalogSensor* p_forcetorque = 0;
        if (p[dev_idx]->poly->view(p_forcetorque))
        {
            if (p_forcetorque->getChannels() == (int)m_nr_of_channels_in_YARP_FT_sensor)
            {
                ft_sensor_list.push_back(p_forcetorque);
                ft_sensor_name.push_back(p[dev_idx]->key);
            }
        }
    }

    if (ft_sensor_list.size() != m_sensors_list.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE))
    {
        yError() << "floatingBaseEstimatorV1: " <<  "Obtained " << m_sensors_list.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) << "from the model, but trying to attach " << (int)ft_sensor_list.size() << " FT sensors in the attach list.";
        return false;
    }

    m_whole_body_forcetorque_interface.resize(ft_sensor_list.size());
    for (size_t iDyn_sensor_idx = 0; iDyn_sensor_idx < m_whole_body_forcetorque_interface.size(); iDyn_sensor_idx++)
    {
        std::string sensor_name = m_sensors_list.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, iDyn_sensor_idx)->getName();
        // search in sensors list for ft sensor  with same name as attach list
        int idx_of_device_with_same_name{-1};
        for (size_t dev_idx = 0; dev_idx < ft_sensor_list.size(); dev_idx++)
        {
            if (ft_sensor_name[dev_idx] == sensor_name)
            {
                idx_of_device_with_same_name = dev_idx;
                break;
            }
        }

        if (idx_of_device_with_same_name == -1)
        {
            yError() << "floatingBaseEstimatorV1: " << "was expecting a FT sensor with name " << sensor_name;
            return false;
        }
        m_whole_body_forcetorque_interface[iDyn_sensor_idx] = ft_sensor_list[idx_of_device_with_same_name];
    }

    m_nr_of_forcetorque_sensors_detected = m_whole_body_forcetorque_interface.size();
    // dry run of readFTSensors() to check if all FTs work and to initialize buffers
    m_ft_measurements_from_yarp_server.resize(m_nr_of_channels_in_YARP_FT_sensor);
    bool verbose{false};
    if (!sensorReadDryRun(verbose, &yarp::dev::baseEstimatorV1::readFTSensors))
    {
        return false;
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::attachAllInertialMeasurementUnits(const yarp::dev::PolyDriverList& p)
{
    std::vector<yarp::dev::IGenericSensor* > imu_sensor_list;
    std::vector<std::string> imu_sensor_name;
    for (size_t dev_idx = 0; dev_idx < (size_t)p.size(); dev_idx++)
    {
        // check if a generic sensor has 12 channels implying it is a IMU sensor
        yarp::dev::IGenericSensor* p_IMU = 0;
        if (p[dev_idx]->poly->view(p_IMU))
        {
            int nr_of_channels{0};
            p_IMU->getChannels(&nr_of_channels);
            if (nr_of_channels == (int)m_nr_of_channels_in_YARP_IMU_sensor)
            {
                imu_sensor_list.push_back(p_IMU);
                imu_sensor_name.push_back(p[dev_idx]->key);
            }
        }
    }

    m_whole_body_imu_interface = imu_sensor_list;
    m_nr_of_IMUs_detected = m_whole_body_imu_interface.size();

    if (m_nr_of_IMUs_detected == 0)
    {
        yError() << "floatingBaseEstimatorV1: " <<  "Expecting atleast one IMU.";
        return false;
    }


    m_raw_IMU_measurements.resize(m_nr_of_IMUs_detected);
    // check this so that we can get sensor transforms from idyntree
    // note this is a reverse check (different from the check on FT sensors)
    for (size_t imu = 0; imu < m_nr_of_IMUs_detected; imu++)
    {
        bool found_imu{false};
        for (size_t iDyn_sensor_idx = 0; iDyn_sensor_idx < m_sensors_list.getNrOfSensors(iDynTree::ACCELEROMETER); iDyn_sensor_idx++)
        {
            std::string imu_name = m_sensors_list.getSensor(iDynTree::ACCELEROMETER, iDyn_sensor_idx)->getName();
            if (imu_sensor_name[imu] == imu_name)
            {
                found_imu = true;
                break;
            }
        }

        if (!found_imu)
        {
            yError() << "floatingBaseEstimatorV1: " << "was expecting IMU with name from " << imu_sensor_name[imu] << " iDyntree Model";
            return false;
        }
        m_raw_IMU_measurements[imu].sensor_name = imu_sensor_name[imu];
    }

    // dry run of readIMUSensors() to check if all IMUs work and to initialize buffers
    m_imu_meaaurements_from_yarp_server.resize(m_whole_body_imu_interface.size());
    for (size_t imu; imu < (size_t)m_whole_body_imu_interface.size(); imu++)
    {
        m_imu_meaaurements_from_yarp_server[imu].resize(m_nr_of_channels_in_YARP_IMU_sensor);
    }
    bool verbose{false};
    if (!sensorReadDryRun(verbose, &yarp::dev::baseEstimatorV1::readIMUSensors))
    {
        return false;
    }
    return true;
}

bool yarp::dev::baseEstimatorV1::readIMUSensors(bool verbose)
{
    bool all_IMUs_read_correctly{true};
    for (size_t imu = 0; imu < m_nr_of_IMUs_detected; imu++)
    {
        // TODO: get sensor name and associated transformation matrix
        bool ok{true};
        m_raw_IMU_measurements[imu].angular_acceleration.zero();
        m_raw_IMU_measurements[imu].angular_velocity.zero();
        m_raw_IMU_measurements[imu].linear_proper_acceleration.zero();

        ok =  m_whole_body_imu_interface[imu]->read(m_imu_meaaurements_from_yarp_server[imu]);
        m_raw_IMU_measurements[imu].sensor_status = ok;
        if (!ok && verbose)
        {
            yWarning() << "floatingBaseEstimatorV1: " << "unable to read from IMU sensor " << m_raw_IMU_measurements[imu].sensor_name << " correctly. using old measurements.";
        }

        if (ok)
        {
            m_raw_IMU_measurements[imu].angular_velocity(0) = deg2rad(m_imu_meaaurements_from_yarp_server[imu][6]);
            m_raw_IMU_measurements[imu].angular_velocity(1) = deg2rad(m_imu_meaaurements_from_yarp_server[imu][7]);
            m_raw_IMU_measurements[imu].angular_velocity(2) = deg2rad(m_imu_meaaurements_from_yarp_server[imu][8]);

            m_raw_IMU_measurements[imu].linear_proper_acceleration(0) = m_imu_meaaurements_from_yarp_server[imu][3];
            m_raw_IMU_measurements[imu].linear_proper_acceleration(1) = m_imu_meaaurements_from_yarp_server[imu][4];
            m_raw_IMU_measurements[imu].linear_proper_acceleration(2) = m_imu_meaaurements_from_yarp_server[imu][5];
        }

        all_IMUs_read_correctly = all_IMUs_read_correctly && ok;
    }

     return all_IMUs_read_correctly;
}

bool yarp::dev::baseEstimatorV1::readFTSensors(bool verbose)
{
    bool ft_sensors_read_correctly{true};
    for (size_t ft = 0; ft < m_nr_of_forcetorque_sensors_detected; ft++)
    {
        iDynTree::Wrench buffer_wrench;
        int ft_ret_value = m_whole_body_forcetorque_interface[ft]->read(m_ft_measurements_from_yarp_server);
        bool ok = (ft_ret_value == yarp::dev::IAnalogSensor::AS_OK);
        ft_sensors_read_correctly = ft_sensors_read_correctly && ok;

        if (!ok && verbose)
        {
            yWarning() << "floatingBaseEstimatorV1: " << "unable to read from FT sensor " << m_sensors_list.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, ft)->getName() << " correctly. using old measurements.";
        }

        bool is_NaN = false;
        for (size_t i = 0; i < m_ft_measurements_from_yarp_server.size(); i++)
        {
            if (std::isnan(m_ft_measurements_from_yarp_server[i]))
            {
                is_NaN = true;
                break;
            }
        }

        if (is_NaN)
        {
            yError() << "floatingBaseEstimatorV1: " << "FT sensor " << m_sensors_list.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, ft)->getName() << " contains nan: . using old measurements."<< m_ft_measurements_from_yarp_server.toString();
            return false;
        }

        if (ok)
        {
            iDynTree::toiDynTree(m_ft_measurements_from_yarp_server, buffer_wrench);
            m_sensor_measurements.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE, ft, buffer_wrench);
        }
    }
    return ft_sensors_read_correctly;
}

bool yarp::dev::baseEstimatorV1::readEncoders(bool verbose)
{
    int ax; m_remapped_control_board_interfaces.encs->getAxes(&ax);
    bool encoders_read_correctly = m_remapped_control_board_interfaces.encs->getEncoders(m_joint_positions.data());

    encoders_read_correctly = m_remapped_control_board_interfaces.encs->getEncoderSpeeds(m_joint_velocities.data()) && encoders_read_correctly;
    if (!encoders_read_correctly && verbose)
    {
        yWarning() << "floatingBaseEstimatorV1: " << "unable to read from encoders interface properly";
    }
    if (m_use_lpf)
    {
        if (!m_device_initialized_correctly)
        {
            m_joint_velocities_filter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(m_joint_vel_filter_cutoff_freq, m_device_period_in_s);
            m_joint_velocities_filter->setSampleTime(m_device_period_in_s);
            m_joint_velocities_filter->setCutFrequency(m_joint_vel_filter_cutoff_freq);
            yarp::sig::Vector init_vel;
            init_vel.resize(m_joint_velocities.size());
            iDynTree::toYarp(m_joint_velocities, init_vel);
            m_joint_velocities_filter->init(init_vel);
        }
        else
        {
            yarp::sig::Vector unfilt_vel(m_joint_velocities.size());
            iDynTree::toYarp(m_joint_velocities, unfilt_vel);
            const yarp::sig::Vector& filtered_velocities = m_joint_velocities_filter->filt(unfilt_vel);
            iDynTree::toiDynTree(filtered_velocities, m_joint_velocities);
        }
    }
    convertVectorFromDegreesToRadians(m_joint_positions);
    convertVectorFromDegreesToRadians(m_joint_velocities);
    return encoders_read_correctly;
}


bool yarp::dev::baseEstimatorV1::configureWholeBodyDynamics(const yarp::os::Searchable& config)
{
    if (config.check("left_foot_cartesian_wrench_port") && config.find("left_foot_cartesian_wrench_port").isString())
    {
        m_left_foot_cartesian_wrench_port_name = config.find("left_foot_cartesian_wrench_port").asString();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"left_foot_cartesian_wrench_port\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    if (config.check("right_foot_cartesian_wrench_port") && config.find("right_foot_cartesian_wrench_port").isString())
    {
        m_right_foot_cartesian_wrench_port_name = config.find("right_foot_cartesian_wrench_port").asString();
    }
    else
    {
        yError() << "floatingBaseEstimatorV1: " << "Could not find \"right_foot_cartesian_wrench_port\" parameter in configuration file." <<
                                                   " Exiting..." ;
        return false;
    }

    bool ok = m_left_foot_cartesian_wrench_wbd_port.open(m_port_prefix + "/left_foot_cartesian_wrench:i");
    ok = yarp::os::Network::connect(m_left_foot_cartesian_wrench_port_name, m_port_prefix + "/left_foot_cartesian_wrench:i") && ok;
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << " could not connect to " << m_left_foot_cartesian_wrench_port_name;
        return false;
    }

    ok = m_right_foot_cartesian_wrench_wbd_port.open(m_port_prefix + "/right_foot_cartesian_wrench:i");
    ok = yarp::os::Network::connect(m_right_foot_cartesian_wrench_port_name, m_port_prefix + "/right_foot_cartesian_wrench:i") && ok;
    if (!ok)
    {
        yError() << "floatingBaseEstimatorV1: " << " could not connect to " << m_right_foot_cartesian_wrench_port_name;
        return false;
    }

    m_right_foot_cartesian_wrench.resize(6);
    m_left_foot_cartesian_wrench.resize(6);

    bool verbose{false};
    if (!sensorReadDryRun(verbose, &yarp::dev::baseEstimatorV1::readWholeBodyDynamicsContactWrenches))
    {
        return false;
    }

    m_wbd_is_open = true;
    return true;
}

bool yarp::dev::baseEstimatorV1::calibFTSensorsStanding()
{
    yarp::os::RpcClient wbd_rpc_port;
    wbd_rpc_port.open("/wholeBodyDynamics/local/rpc");
    yarp::os::Network::connect("/wholeBodyDynamics/local/rpc", "/wholeBodyDynamics/rpc");
    yarp::os::Bottle calib_command;
    calib_command.addString("calibStanding");
    calib_command.addString("all");
    yarp::os::Bottle calib_response;

    // writing five times for sanity check
    for (int i = 0; i < 5; i++)
    {
        yarp::os::Time::delay(0.01);
        wbd_rpc_port.write(calib_command, calib_response);
    }

    if (calib_response.toString() != "[ok]")
    {
        return false;
    }
    return true;
}

bool readCartesianWrenchesFromPorts(yarp::os::BufferedPort<yarp::sig::Vector>& port, yarp::sig::Vector& wrench, bool verbose)
{
    yarp::sig::Vector* raw_wrench;

    raw_wrench = port.read(false);
    if (raw_wrench != nullptr)
    {
        wrench = *raw_wrench;
        if (wrench.size() !=6 && verbose)
        {
            yError() << "floatingBaseEstimatorV1: " << "wrench size mismatch in left foot port.";
        }

        bool is_NaN = false;
        for (size_t i = 0; i < wrench.size(); i++)
        {
            if (std::isnan(wrench[i]))
            {
                is_NaN = true;
                break;
            }
        }

        if (is_NaN)
        {
            yError() << "floatingBaseEstimatorV1: " << "foot cartesian wrench contains nan: . using old measurements." << wrench.toString();
            return false;
        }
    }
    return true;
}

bool yarp::dev::baseEstimatorV1::readWholeBodyDynamicsContactWrenches(bool verbose)
{
    bool ok = readCartesianWrenchesFromPorts(m_left_foot_cartesian_wrench_wbd_port, m_left_foot_cartesian_wrench, m_verbose);
    ok = readCartesianWrenchesFromPorts(m_right_foot_cartesian_wrench_wbd_port, m_right_foot_cartesian_wrench, m_verbose) && ok;

    return ok;
}

bool yarp::dev::baseEstimatorV1::readSensors(bool verbose)
{
    bool all_sensors_read_correctly{true};
    all_sensors_read_correctly = readEncoders(m_verbose) && all_sensors_read_correctly;
    all_sensors_read_correctly = readFTSensors(m_verbose) && all_sensors_read_correctly;
    all_sensors_read_correctly = readIMUSensors(m_verbose) && all_sensors_read_correctly;
    all_sensors_read_correctly = readWholeBodyDynamicsContactWrenches(m_verbose) && all_sensors_read_correctly;

    return all_sensors_read_correctly;
}

bool yarp::dev::baseEstimatorV1::loadTransformBroadcaster()
{
    yarp::os::Property tf_broadcaster_settings;
    tf_broadcaster_settings.put("device", "transformClient");
    tf_broadcaster_settings.put("remote", "/transformServer");
    tf_broadcaster_settings.put("local", m_port_prefix + "/transformClient");

    tf_broadcaster_settings.addGroup("axesNames");
    yarp::os::Bottle& axes_bottle = tf_broadcaster_settings.findGroup("axesNames").addList();


    for (size_t i = 0; i < m_estimation_joint_names.size(); i++)
    {
        axes_bottle.addString(m_estimation_joint_names[i]);
    }

    if (!m_transform_broadcaster.open(tf_broadcaster_settings))
    {
        yError() << "floatingBaseEstimatorV1: " << "could not open transform broadcaster.";
        return false;
    }

    if (!m_transform_broadcaster.view(m_transform_interface))
    {
        yError() << "floatingBaseEstimatorV1: " << "could not access transform interface";
        return false;
    }

    return true;
}

bool yarp::dev::baseEstimatorV1::detachAll()
{
    yarp::os::LockGuard guard(m_device_mutex);
    m_device_initialized_correctly = false;
    if (isRunning())
    {
        stop();
    }
    return true;
}

