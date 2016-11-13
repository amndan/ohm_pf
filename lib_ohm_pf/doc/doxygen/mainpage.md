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
The full set of parameters is described in XY.

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




