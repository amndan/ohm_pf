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

#include "CeilCamUpdater.h"
#include "SampleSet.h"
#include "OhmPfNodeParams.h"
#include "OdomDiffParams.h"
#include "FilterParams.h"
#include "Filter.h"
#include "EnumSensor.h"
#include "LaserUpdater.h"
#include "MapUpdater.h"
#include "OdomUpdater.h"
#include "Resampler.h"
#include "RosLaserPMParams.h"

namespace ohmPf
{

class OhmPfNode
{
public:
  OhmPfNode();
  virtual ~OhmPfNode();
  void spin();
  void spinOnce();
  void printSampleSet(SampleSet* sampleSet);
private:
  void calOdom(const nav_msgs::OdometryConstPtr& msg);
  void cal2dPoseEst(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void calCeilCam(const geometry_msgs::PoseArrayConstPtr& msg);
  void calScan(const sensor_msgs::LaserScanConstPtr& msg);
  void calResampleTimer(const ros::TimerEvent& event);
  void spawnOdom();
  void spawnFilter();
  double _cumSumRot;
  double _cumSumtrans;
  Eigen::Vector3d _lastOdomPose;
  ros::Publisher _pubSampleSet;
  ros::Publisher _pubProbMap;
  ros::Subscriber _subOdometry;
  ros::Subscriber _subCeilCam;
  ros::Subscriber _subScan;
  ros::Subscriber _sub2dPoseEst;
  ros::ServiceClient _cliMapSrv;
  ros::NodeHandle _nh;
  ros::NodeHandle _prvNh;
  ros::Rate _loopRate;
  ros::Timer _resampleTimer;
  OhmPfNodeParams_t _paramSet;
  ohmPf::OdomDiffParams_t _odomDiffParams;
  ohmPf::RosLaserPMParams_t _rosLaserPMParams;
  ohmPf::OdomUpdater* _odomDiff;
  ohmPf::Filter* _filter;
  ohmPf::FilterParams_t _filterParams;
  unsigned int _maxDistanceProbMap;
  bool _odomInitialized;

  int odomCounter;
};

} /* namespace ohmPf */

#endif /* SRC_OHMPFNODE_H_ */
