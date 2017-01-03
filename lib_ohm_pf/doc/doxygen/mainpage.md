\mainpage OhmPf - Introduction Page
\tableofcontents

\image html robot.png

This is the Documentation of the ohmPf package. It can be used for localizing a mobile robot in indoor environments.
For localization it fuses several sensor systems via a particle filter algorithm.

\section sec2 Structure of the Library
The particle filter fuses multible sensor systems for generating an estimation on the robots actual pose. 
The following figure illustrates the structure of the library:
\image html structure.jpg
To use the filter one map represenation and one odometry sensor must be provided. The kinematic model of the
robot must match whether a differential drive system or an omni directional drive. The map data has to be 
provided one-time. Odometry datas frequenzy should be higher than the expexted output pose frequenzy of the 
Filter.

A good starting point for exploring the library is ohmPf::FilterController class.

___
\section sec3 About Sensor Data
To successfully localize a robot at least one more sensor system should be connected to the localization system.
A good choice would be a 2D-Laser-Scanner (LIDAR). You can connect multible laser scanners to the filter system.
Another way of integrating sensor data into the filter is to use the absolute pose input e.g. the CeilCam-System
developed at TH-Nürnberg. It provides absolute pose estimations to the filter.

___
\section sec4 Implementation (ROS)
The library is independently designed from a robots middleware. ROS is a common framework therefore the 
filters implementation on ROS side and with this the user interface of the library will be described in short.
An example ros package called ohm_pf which implements the Filter is shipped with the library. 

\subsection subsec41 Filter-Controller
The FilterController represents the user interface to the library. It provides the global
method ohmPf::IFilterController::createFilter. This method initializes the filter inside
the library and returns the interface for the user. The user has to implement objects
providing the following interfaces:

- ohmPf::IMap
- ohmPf::IOdomMeasurement
- ohmPf::ILaserMeasurement (If using at least one LIDAR)
- ohmPf::IPoseMeasurement (If using absolute pose measurements)
- ohmPf::IFilterOutput

The example ros package implements this classes in the following corresponding files:

- ROSMap.cpp
- ROSOdomMeasurement.cpp
- ROSLaserMeasurement.cpp
- ROSCeilCamMeasurement.cpp
- ROSFilterOutput.cpp

The users implementations can be connected to the library via: 

- ohmPf::IFilterController::setMap
- ohmPf::IFilterController::connectOdomMeasurement
- ohmPf::IFilterController::connectLaserMeasurement
- ohmPf::IFilterController::connectPoseMeasurement
- ohmPf::IFilterController::connectFilterOutput

\subsection subsec42 Filter-Output
The filters output data can be received via the following callback methods within
the connected filter output object: 

- ohmPf::IFilterOutput::onOutputPoseChanged
- ohmPf::IFilterOutput::onSampleSetChanged
- ohmPf::IFilterOutput::onFilterStateChanged

Every output datas position information is considered to be in map frame.

\subsection subsec43 Map
Before localization a map of the environment must be recorded. This can be done with
several mapping algorithms e.g. [ohm_tsd_slam](http://wiki.ros.org/ohm_tsd_slam) 
from TH-Nürnberg. Maps can be stored and connected to the filter via the 
[map_server](http://wiki.ros.org/map_server) packet from ROS. 

\subsection subsec44 Visualisation
Visualization of the filters particle cloud can easy be done with 
[rviz](http://wiki.ros.org/rviz):

\image html stdr.jpg

For more Information see ROSFilterOutput.cpp in example package.


\subsection subsec45 Parameters
Following table lists all available parameters for lib_ohm_pfs ROS implementation. 

name	|	description	|	typical values
------------- | ------------- | -------------
tfFixedFrame	|	Tf-Frame-Name of fixed frame. Often called "map".	|	map
tfOdomFrame	|	Tf-Frame-Name of odom frame. Often called "odom".	|	odom
tfBaseFootprintFrame	|	Tf-Frame-Name of robots base-footprint frame. Often called "base_link".	|	base_link
topMapSrv	|	Service topic for initially receiving the map. 	|	static_map
topScan	|	Topic of scan data. For multible scanners separate topics with ";". Additionaliy adjust count lasers according to the number of scanners to be used.	|	sick;hokuyo
countLasers	|	Ammount of used scanners. Must match number of topics in topScan param.	|	1 - 2
topOdometry	|	Topic of odometry data.	|	odom
top2dPoseEstimate	|	Topic for sending an init pose to the filter.	|	initialpose
topCeilCam	|	Topic for sending absolute pose measurements to the filter. For example measurements from the CeilCam package or GPS data.	|	pose_meas
topParticleCloud	|	Topic for visualization of the filters particlecloud.	|	particlecloud
topProbPose	|	Topic for the quality of the filters pose estimation. 0.0 indicates bad quality - 1.0 indicates good quality. The Filter is not (yet) able to detect localization failures. This value just represents an indicator for the spreading of all particles.	|	prob_pose
filterLoopRate	|	The filters loop rate. If filters loop duration is longer a debug message is printed. If filters loop duration is under filterLoopRate the node sleeps.	|	0.02
outputIntervallFilter	|	The intervall the filters actualizes its pose estimation and particle cloud visualization.	|	0.05
skipParticleForGui	|	Subsampling of particlecloud for visualization. 0 for no subsampling.	|	0 - 100
samplesMax	|	Number of particles to be used.	|	5000
additionalTranslationalNoise	|	adds additional translational noise to each particle at resampling step. This increases spreading of particles.	|	0.05
additionalRotationalNoise	|	adds rotational translational noise to each particle at resampling step. This increases spreading of particles.	|	0.01
resamplingIntervallFilter	|	Minimum elapsed time before resampling step is done in seconds. Higher resampling intervall ensures that resampling step is based on more measurement steps. So a low resampling intervall leads to resampling steps based on single measurements and therefore to low localization stability. A high resampling intervall leads to bad accuracy caused by high spreading of particles. 	|	0.1 - 1.5
resamplingMethod	|	Method for resampling particles. STD means standard resampling - LV means low variance resampling. Low variance resampling leads to less convergence speed.	|	STD
lowVarianceFactor	|	ammount of particles drawn in one iteration when low variance resampling is used.	|	3 - 10
initMode	|	Initialization mode. GL for global localization - POSE for initialization at given pose.	|	GL
initX	|	Initial particle cloud position x in meter if filter initMode is POSE.	|	0.0
initY	|	Initial particle cloud position y in meter if filter initMode is POSE.	|	0.0
initPhi	|	Initial particle cloud position yaw in rad if filter initMode is POSE.	|	0.0
initSigmaTrans	|	Initial translational particle cloud spreading (standard deviation in meter) if filter initMode is POSE.	|	0.2
initSigmaRot	|	Initial rotational particle cloud spreading (standard deviation in rad) if filter initMode is POSE.	|	0.3
subsamplingRateLaser	|	Linear subsampling of laser scan data before integration. Higher value leads to slower convergence speed but also affects pose accuracy. Choosing high subsampling rate means bad accuracy.	|	3 - 5
uncertaintyLaser	|	Additional uncertainty of laser measurements. Higher values lead to a slower convergence speed.	|	0.1 - 0.8
maxDistanceProbMap	|	Distance for inflation of occupied parts at creation of probability field in cells. Depends on map resolution. Higher value leads to slower convergence speed.	|	5 - 30
OCSRotToTransFactor	|	Factor for transforming radial motion into linear motion. OCS limits are in meter - rotational movement is transformed with this factor to linear movement.	|	6 - 10
OCSThresholdLaser	|	Minimum absolute movement of robot in meter before a new laser measurement gets processed. Prevents the Filter from updating particles with redundant information.	|	0.005
OCSThresholdOdom	|	Minimum absolute movement of robot in meter before a new odometry measurement gets processed. If odom data is noisy no odometry update should be done if the robot stops.	|	0.001
OCSThresholdResampler	|	Minimum absolute movement of robot in meter before resampling step takes place. Can be used to delaying the resampling step according to movement of the robot. This the same effect as a higher resampling intervall, but its scaled with the movement of the robot.	|	0.050
odomModel	|	Used odometry model. 0 means differential drive - 1 means omni drive.	|	1
odomAlpha1	|	DIFF: rot error from rot motion; OMNI: x error from x motion	|	0.2
odomAlpha2	|	DIFF: rot error from trans motion; OMNI: y error from y motion	|	0.2
odomAlpha3	|	DIFF: trans error from trans motion; OMNI: yaw error from yaw motion	|	0.2
odomAlpha4	|	DIFF: trans error from trot motion; OMNI: not used here	|	0.2
useAdaptiveMean	|	EXPERIMENTAL - adaptive mean approach tries to detect localization failures according to Probablistic Robotics book. It's an experimental feature and not yet finished. 	|	false
alphaFast	|	EXPERIMENTAL - alphaFast factor for adaptive mean approach.	|	-
alphaSlow	|	EXPERIMENTAL - alphaSlow factor for adaptive mean approach.	|	-
minStabwToResample	|	EXPERIMENTAL - resampling takes place if particle clouds stabw is higher than minStabwToResample.	|	0.0003
minimumValidScanRaysFactor	|	EXPERIMENTAL - a debug message is printed if scan rays according to this factor are invalid (inf, nan, <minRange, >maxRange). 0.85 means 85% of scan rays must be valid. 	|	0.85

___
\section sec5 References
Some useful informations and hints...

\subsection subsec51 Using EKF
Odometry data must be present with high frequenzies. If the robots hardware is not capable
of providing accurate and fast odometry data an Extended-Kalman-Filter (EKF) could be the
soulution. Its possible to fuse Odometry data, IMU data and the kinematic modell of the 
robot in a preliminary EKF to provide high frequent odometry data to the filter. The ROS
package [robot-localization](http://wiki.ros.org/robot_localization) provides this functionaity.

\subsection subsec52 No Odometry
If no odometry system is available on the target platform, following 
two ROS packages should be checked:

- [laser-scan-matcher](http://wiki.ros.org/laser_scan_matcher)
wich provides odometry signal based on a LIDAR.
- [viso2-ros](http://wiki.ros.org/viso2_ros)
which is a visual odometry framework to generate odometry data from camera images
 
\subsection subsec53 No LIDAR
If no LIDAR is available on the target platform but e.g. a kinect or 
intel realsense camera, following package should be checked:

- [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan)


