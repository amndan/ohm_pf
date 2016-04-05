/*
 * ROSFilterOutput.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "ROSFilterOutput.h"

namespace ohmPf
{

  ROSFilterOutput::ROSFilterOutput(std::string fixedFrame, std::string outputFrame) : _tfBroadcaster()
  {
    ros::NodeHandle nh = ros::NodeHandle();
    _pubPoseArray = nh.advertise<geometry_msgs::PoseArray>("particleCloud", 1, true);
    _fixedFrame = fixedFrame;
    _outputFrame = outputFrame;
  }

  ROSFilterOutput::~ROSFilterOutput()
  {
    // TODO Auto-generated destructor stub
  }

  void ROSFilterOutput::actualizeTF(Eigen::Vector3d pose)
  {
    tf::Transform tf;
    tf.setOrigin( tf::Vector3(pose(0), pose(1), 0.0) );
    tf.setRotation( tf::createQuaternionFromYaw( pose(2) ) );
    _tfBroadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), _fixedFrame, _outputFrame));
  }

  void ROSFilterOutput::printSampleSet(std::vector<Sample_t>&  samples)
  {
    geometry_msgs::PoseArray poseArray;
    geometry_msgs::Pose pose;

    poseArray.header.frame_id = _fixedFrame;

    for(unsigned int i = 0; i < samples.size(); i++)
    {
      pose.position.x = samples[i].pose(0);
      pose.position.y = samples[i].pose(1);
      pose.position.z = 0.0;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw( samples[i].pose(2) ), pose.orientation);
      poseArray.poses.push_back(pose);
    }
    _pubPoseArray.publish(poseArray);

  }

} /* namespace ohmPf */
