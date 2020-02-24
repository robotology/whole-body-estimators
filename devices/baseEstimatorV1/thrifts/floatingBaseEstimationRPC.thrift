/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

struct Pose6D
{
    1: double x; 2: double y; 3: double z;
    4: double roll; 5: double pitch; 6: double yaw;
}

service floatingBaseEstimationRPC
{
    string getEstimationJointsList();
    bool setMahonyKp(1: double kp);
    bool setMahonyKi(1: double ki);
    bool setMahonyTimeStep(1: double timestep);
    bool setContactSchmittThreshold(1: double l_fz_break, 2: double l_fz_make,
                                    3: double r_fz_break, 4: double r_fz_make);

    bool setPrimaryFoot(1: string primary_foot);

    bool resetIMU();
    bool resetLeggedOdometry();
    bool resetLeggedOdometryWithRefFrame(1: string ref_frame, 2: double x, 3: double y, 4: double z,
                                       5: double roll, 6: double pitch, 7: double yaw);
    bool resetEstimator();
    bool resetEstimatorWithRefFrame(1: string ref_frame, 2: double x, 3: double y, 4: double z,
                                 5: double roll, 6: double pitch, 7: double yaw);

    string getRefFrameForWorld();
    Pose6D getRefPose6DForWorld();
    bool useJointVelocityLPF(1: bool flag);
    bool setJointVelocityLPFCutoffFrequency(1: double freq);
    bool startFloatingBaseFilter();
}

