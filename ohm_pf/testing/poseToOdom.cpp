/*
 * poseToOdom.cpp
 *
 *  Created on: 12.01.2016
 *      Author: amndan
 */

//using namespace ohmPf;
using namespace std;

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include <ctime>
#include <cmath>
#include <iostream>

void init();
void calPose(const geometry_msgs::PoseStampedConstPtr& msg);

int odomCounter;
ros::Publisher pubOdom;
ros::Subscriber subPose;
ros::NodeHandle* nh = NULL;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poseToOdom");

  nh = new ros::NodeHandle();

  subPose = nh->subscribe("pose", 1, &calPose);

  pubOdom = nh->advertise<nav_msgs::Odometry>("odom", 1, true);

  ros::spin();
}


void calPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  nav_msgs::Odometry odom;

  odom.header = msg->header;
  odom.child_frame_id = "odom";
  odom.pose.pose.position = msg->pose.position;
  odom.pose.pose.orientation = msg->pose.orientation;

  pubOdom.publish(odom);
}
