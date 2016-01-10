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
  _prvNh.param<std::string>("topFixedFrame", _paramSet.topFixedFrame, "map");
  _prvNh.param<std::string>("topOdometry", _paramSet.topOdometry, "/robot0/odom");
  _prvNh.param<std::string>("top2dPoseEst", _paramSet.top2dPoseEst, "initialpose");

  _pubSampleSet = _nh.advertise<geometry_msgs::PoseArray>("particleCloud", 1, true);
  _subOdometry = _nh.subscribe(_paramSet.topOdometry, 1, &OhmPfNode::calOdom, this);
  _sub2dPoseEst = _nh.subscribe(_paramSet.top2dPoseEst, 1, &OhmPfNode::cal2dPoseEst, this);

  _odomInitialized = false;

  spawnOdom();
  spawnFilter();
}

OhmPfNode::~OhmPfNode()
{
  delete _odomDiff;
  delete _filter;
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
  poseArray.header.frame_id = _paramSet.topFixedFrame;

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

void OhmPfNode::calOdom(const nav_msgs::OdometryConstPtr& msg)
{
  Eigen::Vector3d measurement;
  measurement(0) = msg->pose.pose.position.x;
  measurement(1) = msg->pose.pose.position.y;
  measurement(2) = tf::getYaw(msg->pose.pose.orientation);

  if(!_odomInitialized)
  {
    ROS_INFO_STREAM("Received first odom message - initializing odom...");
    _odomDiff->addSingleMeasurement(measurement);
    _odomInitialized = true;
    ROS_INFO_STREAM("odom initialized");

    //todo remove init from here
    _filter->initWithPose(measurement);
    printSampleSet(_filter->getSampleSet());

    return;
  }

  _odomDiff->addSingleMeasurement(measurement);
  _odomDiff->updateFilter(_filter);
  printSampleSet(_filter->getSampleSet());
}

void OhmPfNode::cal2dPoseEst(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  Eigen::Vector3d measurement;
  measurement(0) = msg->pose.pose.position.x;
  measurement(1) = msg->pose.pose.position.y;
  measurement(2) = tf::getYaw(msg->pose.pose.orientation);

  _filter->initWithPose(measurement);
  printSampleSet(_filter->getSampleSet());
}

void OhmPfNode::spawnOdom()
{
  //todo: get odom Params from Launchfile
  _odomDiffParams.a1 = 0;
  _odomDiffParams.a2 = 0;
  _odomDiffParams.a3 = 0;
  _odomDiffParams.a4 = 0;

  _odomDiff = new ohmPf::OdomDiff(_odomDiffParams);
}

void OhmPfNode::spawnFilter()
{
  _filterParams.samplesMax = 500;
  _filterParams.samplesMin = 20;

  _filter = new ohmPf::Filter(_filterParams);
}

} /* namespace ohmPf */
