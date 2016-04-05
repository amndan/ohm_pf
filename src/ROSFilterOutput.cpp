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
  ROSFilterOutput::ROSFilterOutput(OhmPfNodeParams_t paramSet) : _tfBroadcaster(), _tfListener()
  {
    ros::NodeHandle nh = ros::NodeHandle();
    _pubPoseArray = nh.advertise<geometry_msgs::PoseArray>("particleCloud", 1, true);
    _paramSet = paramSet;
  }

  ROSFilterOutput::~ROSFilterOutput()
  {
    // TODO Auto-generated destructor stub
  }

  void ROSFilterOutput::actualizeTF(Eigen::Vector3d pose)
  {
    // map_odom * odom_bf = map_ohmPf
    // map_odom = map_ohmPf * odom_bf.inverse() --> pose correction from map to odom frame
    // map_odom = map_ohmPf * bf_odom

    ros::Time now(ros::Time::now());

    tf::StampedTransform tmpTransform;

    // TODO: better solution here!
    assert(_tfListener.waitForTransform(_paramSet.tfBaseFootprintFrame, _paramSet.tfOdomFrame, ros::Time(0), ros::Duration(10)));

    try
    {
      _tfListener.lookupTransform(_paramSet.tfBaseFootprintFrame, _paramSet.tfOdomFrame, ros::Time(0), tmpTransform);
      // TODO: TIMING!
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      exit(EXIT_FAILURE);
    }
    tf::Transform tf_bf_odom = tmpTransform;

    tf::Transform tf_map_pf;
    tf_map_pf.setOrigin( tf::Vector3(pose(0), pose(1), 0.0) );
    tf_map_pf.setRotation( tf::createQuaternionFromYaw( pose(2) ) );

    tf::Transform map_odom = tf_map_pf * tf_bf_odom;

    _tfBroadcaster.sendTransform(tf::StampedTransform(tf_map_pf, now, _paramSet.tfFixedFrame, _paramSet.tfOutputFrame));
    _tfBroadcaster.sendTransform(tf::StampedTransform(map_odom, now, _paramSet.tfFixedFrame, _paramSet.tfOdomFrame));
  }

  void ROSFilterOutput::printSampleSet(std::vector<Sample_t>&  samples)
  {
    geometry_msgs::PoseArray poseArray;
    geometry_msgs::Pose pose;

    poseArray.header.frame_id = _paramSet.tfFixedFrame;

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
