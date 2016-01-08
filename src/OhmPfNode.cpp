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
    _nh(), _prvNh("~"), loopRate(25)
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

void OhmPfNode::printSampleSet(SampleSet* sampleSet){
  std::vector<Sample_t> samples;
  samples = *(sampleSet->getSamples());

  geometry_msgs::PoseArray poseArray;
  geometry_msgs::Pose pose;

  //todo: use frame id from parameter
  poseArray.header.frame_id = "map";

  for(unsigned int i = 0; i < samples.size(); i++)
  {
    pose.position.x = samples[i].pose(0);
    pose.position.y = samples[i].pose(1);
    pose.position.z = 0;
    //todo: use a more efficient method here
    tf::Quaternion orientation = tf::createQuaternionFromYaw( (double) samples[i].pose(2) );
    pose.orientation.w = orientation.getW();
    pose.orientation.x = orientation.getX();
    pose.orientation.y = orientation.getY();
    pose.orientation.z = orientation.getZ();

    poseArray.poses.push_back(pose);
  }
  _pubSampleSet.publish(poseArray);
}

} /* namespace ohmPf */
