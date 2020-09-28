This is a new and incomplete version of http://wiki.icub.org/wiki/Force_Control , updated with 
the details on the new `wholeBodyDynamicsDevice` module, which is in charge of estimating mainly joint torques and external forces, developed for the CoDyCo European Project.

# History of joint-level torque control on iCub 
The first versions of iCub were not designed to perform joint-level torque control. 
To enable joint-level torque control, the iCub v1.1 was modified to include 4 six-axis 
force-torque sensors, embedded inside the arms and the legs of robot (2008-2010). [1]

The original plan was to develop the iCub v2 with real strain-gauge based joint torque sensors, 
but this process [2] was not successfull and all existing iCub's uses the embedded six-axis force-torque
sensors to estimate the joint torques for low-level joint torque control.

During the CoDyCo project (2013-2017), the software originally developed in [1] was rewritten to improve 
its computational efficency and its generality (i.e. the possibility to run it on several different robots).

# `wholebodydynamics` YARP device 
The `wholebodydynamics` YARP device (contained in the C++ class [`yarp::dev::WholeBodyDynamicsDevice`](http://wiki.icub.org/codyco/dox/html/classyarp_1_1dev_1_1WholeBodyDynamicsDevice.html))
is reading measurements of the embedded force-torque sensors, of the joint position and low-level estimates of joint velocity and accelerations and of 
one IMU mounted in the robot, and from this reading is estimating the external force-torques and internal joint torques of the robot. 

## Differences with respect to the wholeBodyDynamics YARP module 
From the user perpective, the main differences w.r.t. to the wholeBodyDynamics YARP module are: 
* The estimation is performed using the iDynTree library, replacing the use of the iDyn library.
* The model of the robot and of the sensor is loaded from a URDF model, as documented in https://github.com/robotology/idyntree/blob/master/doc/model_loading.md . 
  This permits to run the estimation algorithm on arbitrary robot without modifyng the code, as in the case of the iCubHeidelberg01 that w.r.t. to normal iCub is missing the head and the arms. 
* The RPC interface is implemented using [YARP Thrift](http://www.yarp.it/thrift_tutorial_simple.html).
  This means that the `0` shorthand for performing the calibration of the force-torque offset is not supported anymore. The `calib` command however is compatible between the wholeBodyDynamics 
  YARP module and the `wholebodydynamics` YARP device, so please use that one in your code to be compatible with both interfaces. 
* The functionality of the gravityCompensator module are now integrated in the `wholebodydynamics` device, and can be enabled/disabled using the parameters in the `GRAVITY_COMPENSATION` group. 
  Furthermore, the gravity compensation torque offset is not sent anymore to the board if the axis control mode is set to `VOCAB_CM_TORQUE` . 


## Run the `wholebodydynamics` YARP device
Being a YARP device, it can run on the robot main PC in the robot's yarprobotinterface, or on an external pc using a separate yarprobotinterface.
The `wholebodydynamics` device requires robot-specific configuration files, and currently this configuration files are provided for the following robots:

| `YARP_ROBOT_NAME` | Number of F/T sensors | Internal electronics architecture | Support for running wholebodydynamics on the robot's yarprobotinterface |
|:---:|:---:|:---:|:---:|
| `icubGazeboSim`   | 6   | N/A | NO  |
| `iCubDarmstadt01` | 6   | ETH | YES |
| `iCubGenova02`    | 6   | ETH | YES |
| `iCubGenova04`    | 6   | ETH | YES |
| `iCubNancy01`     | 6   | ETH | YES |
| `iCubHeidelberg01` | 4  | ETH | YES |
| `iCubGenova01`    | 6   | CAN | NO  |
| `iCubGenova03`    | 6   | CAN | NO  |
| `iCubParis01`     | 6   | CAN | NO  |
| `iCubParis02`     | 6   | CAN | NO  |

Note however that  over time the configuration of this robots can change, and the configuration files contained in this repository may not be updated. Please check the status of the configuration files with the maintainer of this repository before using this configuration files.

### Run `wholebodydynamics` on an external PC
This is the recommended procedure in general. To launch the `wholebodydynamics` on an external PC running a *nix based OS, just run:
~~~
YARP_ROBOT_NAME=<yarp_robot_name> yarprobotinterface --config launch-wholebodydynamics.xml  
~~~
where `<yarp_robot_name>` is the robot for which you are launching the estimator.

For example, if you want to run the `wholebodydynamics` for the Gazebo simulation, you will need to run:
~~~
YARP_ROBOT_NAME=icubGazeboSim yarprobotinterface --config launch-wholebodydynamics.xml
~~~

Note that you can avoid to preprend the `YARP_ROBOT_NAME=icubGazeboSim` environmental variable.  

### Run `wholebodydynamics` on the robot 
**Please note that this configuration is not officially supported by the iCub Facility support team, so please request support only from this repo.**

This setup requires adding one configuration file to the robot `yarprobotinterface` configuration files as distributed in the [robots-configuration](https://github.com/robotology/robots-configuration) repository. For doing this, please import the configuration files for your robot on the `~/.local/share/yarp/robots/${YARP_ROBOT_NAME}` using the `yarp-config` utility program. 

Assuming that `<codyco_prefix>` is the build or installation directory of this repo, copy the file `<codyco_prefix>/share/codyco/robots/${YARP_ROBOT_NAME}/estimators/wholebodydynamics.xml` 
in `~/.local/share/yarp/robots/${YARP_ROBOT_NAME}/estimators/wholebodydynamics.xml` (create the `estimators` directory if necessary). 
Then create a copy of the `icub_all.xml` file (or of any file that you want to use) called `icub_wbd.xml`. 
In `icub_wbd.xml`, add the following two lines before the `</robot>` closing tag:
~~~
<!--  ESTIMATORS   -->
<devices file="estimators/wholebodydynamics.xml" />
~~~
Now we can launch the `yarprobotinterface` with the `wholebodydynamics` device using the following command on the robot's PC:
~~~
yarprobotinterface --config icub_wbd.xml 
~~~

It may be convenient to create a modified version of the `iCubStartup.xml` `yarpmanager` application on the terminal PC on which you launch the `yarpmanager`, 
called `iCubStartupWithWBD.xml`, in which the `--config icub_wbd.xml` parameter is explicitly added when launching the `yarprobotinterface`. 

# Credits 
The original iCub force control people: Matteo Fumagalli, Serena Ivaldi, Marco Randazzo, Francesco Nori.

The people working on estimation during the CoDyCo project : Silvio Traversaro, Marco Randazzo, Daniele Pucci, Francesco Romano, Andrea Del Prete, Jorhabib Eljaik, Francisco Javier Andrade Chavez, Nuno Guedelha, Francesco Nori.   

# Citations 
[1] : http://ieeexplore.ieee.org/abstract/document/6100813/

[2] : http://ieeexplore.ieee.org/abstract/document/5379525/