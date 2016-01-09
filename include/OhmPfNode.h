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
#include "tf/transform_datatypes.h"
#include "SampleSet.h"
#include "OhmPfNodeParams.h"
#include "OdomDiffParams.h"
#include "FilterParams.h"
#include "OdomDiff.h"
#include "Filter.h"


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
  void spawnOdom();
  void spawnFilter();
  ros::Publisher _pubSampleSet;
  ros::Subscriber _subOdometry;
  ros::NodeHandle _nh;
  ros::NodeHandle _prvNh;
  ros::Rate loopRate;
  OhmPfNodeParams_t _paramSet;
  ohmPf::OdomDiffParams_t _odomDiffParams;
  ohmPf::OdomDiff* _odomDiff;
  ohmPf::Filter* _filter;
  ohmPf::FilterParams_t _filterParams;
  bool _odomInitialized;
};

} /* namespace ohmPf */

#endif /* SRC_OHMPFNODE_H_ */
