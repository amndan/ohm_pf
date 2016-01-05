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
#include "tf/transform_datatypes.h"
#include "SampleSet.h"

namespace ohmPf
{

class OhmPfNode
{
public:
  OhmPfNode();
  virtual ~OhmPfNode();
  void spin();
  void spinOnce();
  void printSampleSet(std::vector<Sample_t> samples);
private:
  ros::Publisher _pubSampleSet;
  ros::NodeHandle _nh;
  ros::NodeHandle _prvNh;
  ros::Rate loopRate;
};

} /* namespace ohmPf */

#endif /* SRC_OHMPFNODE_H_ */
