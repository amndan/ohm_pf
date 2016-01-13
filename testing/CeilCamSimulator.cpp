/*
 * CeilCamSimulator.cpp
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
void calOdom(const nav_msgs::OdometryConstPtr& msg);

int odomCounter;
ros::Publisher pubSampleSet;
ros::Subscriber subOdom;
ros::NodeHandle* nh = NULL;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ceilCamSimulator");

  nh = new ros::NodeHandle();

  subOdom = nh->subscribe("robot0/odom", 1, &calOdom);

  pubSampleSet = nh->advertise<geometry_msgs::PoseArray>("ceilCamPoseArray", 1, true);

  srand48(std::time(NULL));

  ros::spin();
}


void calOdom(const nav_msgs::OdometryConstPtr& msg)
{
  if(odomCounter > 10)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    geometry_msgs::PoseArray poseArray;
    poseArray.header.stamp = ros::Time::now();

    geometry_msgs::Pose pose;
    pose.position.x = msg->pose.pose.position.x;
    pose.position.y = msg->pose.pose.position.y;
    pose.orientation = msg->pose.pose.orientation;

    poseArray.poses.push_back(pose);
    poseArray.header.frame_id = "map";

    for(unsigned int i = 0; i < 4; i++)
    {
      pose.position.x = drand48() * 10;
      pose.position.y = drand48() * 10;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(drand48() * 2* M_PI - M_PI), pose.orientation);

      poseArray.poses.push_back(pose);
    }

    pubSampleSet.publish(poseArray);

    odomCounter = 0;
  }
  odomCounter++;
}
