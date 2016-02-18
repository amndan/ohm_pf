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
#include "SampleSet.h"
#include "OhmPfNodeParams.h"
#include "OdomDiffParams.h"
#include "CeilCam.h"
#include "FilterParams.h"
#include "OdomDiff.h"
#include "Filter.h"
#include "RosLaserPM.h"
#include "RosMap.h"



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
  void spawnOdom();
  void spawnFilter();
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
  OhmPfNodeParams_t _paramSet;
  ohmPf::OdomDiffParams_t _odomDiffParams;
  ohmPf::OdomDiff* _odomDiff;
  ohmPf::CeilCam* _ceilCam;
  ohmPf::RosLaserPM* _rosLaserPm;
  ohmPf::Filter* _filter;
  ohmPf::FilterParams_t _filterParams;

  bool _odomInitialized;

  int odomCounter;
};

} /* namespace ohmPf */

#endif /* SRC_OHMPFNODE_H_ */
