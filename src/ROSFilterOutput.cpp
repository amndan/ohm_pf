/*
 * ROSFilterOutput.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "ROSFilterOutput.h"

namespace ohmPf
{
// here comes filter params...
ROSFilterOutput::ROSFilterOutput(OhmPfNodeParams_t paramSet) :
    _tfBroadcaster(), _tfListener()
{
  _paramSet = paramSet;
  ros::NodeHandle nh = ros::NodeHandle();
  _pubPoseArray = nh.advertise<geometry_msgs::PoseArray>(_paramSet.topParticleCloud, 1, true);
  _pubProbPose = nh.advertise<std_msgs::Float32>(_paramSet.topProbPose, 1, true);
  _pubPose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true); // TODO: launchfile param

  _skipParticleForGui = std::abs(paramSet.skipParticleForGui);
  // TODO: we schould separate the pub gui output from the resampling step
}

void ROSFilterOutput::onOutputPoseChanged(Eigen::Vector3d pose)
{
  // map_odom * odom_bf = map_ohmPf
  // map_odom = map_ohmPf * odom_bf.inverse() --> pose correction from map to odom frame
  // map_odom = map_ohmPf * bf_odom

  tf::StampedTransform tmpTransform;

  if (!_tfListener.waitForTransform(_paramSet.tfBaseFootprintFrame, _paramSet.tfOdomFrame, ros::Time(0),
                                    ros::Duration(10)))
  {
    ROS_ERROR_STREAM(
        "cannot publish filters output because tf from " << _paramSet.tfBaseFootprintFrame << " to " << _paramSet.tfOdomFrame << " is not available --> will continue without output...");
    return;
  }

  _tfListener.lookupTransform(_paramSet.tfBaseFootprintFrame, _paramSet.tfOdomFrame, ros::Time(0), tmpTransform);
  //TIMING?

  tf::Transform tf_bf_odom = tmpTransform;

  tf::Transform tf_map_pf;
  tf_map_pf.setOrigin(tf::Vector3(pose(0), pose(1), 0.0));
  tf_map_pf.setRotation(tf::createQuaternionFromYaw(pose(2)));

  tf::Transform map_odom = tf_map_pf * tf_bf_odom;

  ros::Time now(ros::Time::now());

  _tfBroadcaster.sendTransform(tf::StampedTransform(tf_map_pf, now, _paramSet.tfFixedFrame, _paramSet.tfOutputFrame));
  _tfBroadcaster.sendTransform(tf::StampedTransform(map_odom, now, _paramSet.tfFixedFrame, _paramSet.tfOdomFrame));

  //pose publisher
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = _paramSet.tfFixedFrame;
  poseStamped.header.stamp = now;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pose(2)), poseStamped.pose.orientation);
  poseStamped.pose.position.x = pose(0);
  poseStamped.pose.position.y = pose(1);
  poseStamped.pose.position.z = 0.0;
  _pubPose.publish(poseStamped);
}

void ROSFilterOutput::onSampleSetChanged(const std::vector<Sample_t>& samples)
{
  geometry_msgs::PoseArray poseArray;
  geometry_msgs::Pose pose;

  poseArray.header.frame_id = _paramSet.tfFixedFrame;

  for (unsigned int i = 0; i < samples.size(); i = i + 1 + _skipParticleForGui)
  {
    pose.position.x = samples[i].pose(0);
    pose.position.y = samples[i].pose(1);
    pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(samples[i].pose(2)), pose.orientation);
    poseArray.poses.push_back(pose);
  }
  _pubPoseArray.publish(poseArray);
}

void ROSFilterOutput::onFilterStateChanged(FilterState_t state)
{
  std_msgs::Float32 msg;
  msg.data = state.probPose;
  _pubProbPose.publish(msg);
}



} /* namespace ohmPf */
