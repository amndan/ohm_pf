/*
 * OhmPfNode.cpp
 *
 *  Created on: Jan 5, 2016
 *      Author: amndan
 */

#include "OhmPfNode.h"

namespace ohmPf
{

OhmPfNode::OhmPfNode() :
    _nh(), _prvNh("~"), loopRate(5)
{
  _pubSampleSet = _nh.advertise<geometry_msgs::PoseArray>("particleCloud", 1, true);
}

OhmPfNode::~OhmPfNode()
{
  // TODO Auto-generated destructor stub
}

void OhmPfNode::spin()
{
  ros::spin();
}

void OhmPfNode::spinOnce()
{
  if(ros::ok())
  {
    loopRate.sleep();
    ros::spinOnce();
  }
  else
  {
    exit(EXIT_FAILURE);
  }
}

void OhmPfNode::printSampleSet(std::vector<Sample_t> samples){
  geometry_msgs::PoseArray poseArray;
  geometry_msgs::Pose pose;

  for(unsigned int i = 0; i < samples.size(); i++)
  {
    pose.position.x
    poseArray.poses.push_back(pose);
  }

}

} /* namespace ohmPf */
