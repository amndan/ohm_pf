/*
 * OhmPfNode.h
 *
 *  Created on: Jan 5, 2016
 *      Author: amndan
 */

#ifndef SRC_OHMPFNODE_H_
#define SRC_OHMPFNODE_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "assert.h"
#include "OhmPfNodeParams.h"
#include "RosLaserPMParams.h"

#include "interfaces/IFilterController.h"
#include "FilterParams.h"
#include "ROSOdomMeasurement.h"
#include "ROSMap.h"
#include "ROSLaserMeasurement.h"
#include "ROSFilterOutput.h"
#include "ROSCeilCamMeasurement.h"

namespace ohmPf
{

class OhmPfNode
{
public:
  OhmPfNode();
  virtual ~OhmPfNode();
  void spin();
  void spinOnce();
private:
  void calOdom(const nav_msgs::OdometryConstPtr& msg);
  void cal2dPoseEst(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void calClickedPoint(const geometry_msgs::PointStampedConstPtr& msg);
  void calCeilCam(const geometry_msgs::PoseArrayConstPtr& msg);
  void calScan(const sensor_msgs::LaserScanConstPtr& msg);
  void calResampleTimer(const ros::TimerEvent& event);
  void spawnFilter();
  void waitForMap();
  ros::Publisher _pubSampleSet;
  ros::Publisher _pubProbMap;
  ros::Subscriber _subOdometry;
  ros::Subscriber _subCeilCam;
  ros::Subscriber _subScan;
  ros::Subscriber _sub2dPoseEst;
  ros::Subscriber _subClickedPoint;
  ros::ServiceClient _cliMapSrv;
  ros::NodeHandle _nh;
  ros::NodeHandle _prvNh;
  ros::Rate _loopRate;
  ros::Timer _resampleTimer;
  OhmPfNodeParams_t _paramSet;
  OdomDiffParams_t _odomDiffParams;
  RosLaserPMParams_t _rosLaserPMParams;
  FilterParams_t _filterParams;
  unsigned int _maxDistanceProbMap;
  bool _odomInitialized;
  bool _laserInitialized;
  bool _ceilCamInitialized;
  IFilterController* _filterController;
  ROSOdomMeasurement* _odomMeasurement;
  ROSMap* _map;
  ROSLaserMeasurement* _laserMeasurement;
  ROSFilterOutput* _filterOutput;
  ROSCeilCamMeasurement* _ceilCamMeasurement;

  int odomCounter;
};

} /* namespace ohmPf */

#endif /* SRC_OHMPFNODE_H_ */
