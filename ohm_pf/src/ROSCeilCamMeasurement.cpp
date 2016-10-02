/*
 * ROSCeilCamMeasurement.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "ROSCeilCamMeasurement.h"

namespace ohmPf
{

  ROSCeilCamMeasurement::ROSCeilCamMeasurement()
  {
    // TODO Auto-generated constructor stub

  }

  std::vector<Eigen::Vector3d> ROSCeilCamMeasurement::getPoses()
  {
    return _poses;
  }

  std::vector<double> ROSCeilCamMeasurement::getProbabilities()
  {
    return _probs;
  }

  void ROSCeilCamMeasurement::setMeasurement(const geometry_msgs::PoseArrayConstPtr& msgs)
  {
    _poses.clear();
    _probs.clear();

    for(unsigned int i = 0; i < msgs->poses.size(); i++)
    {
      Eigen::Vector3d pose;
      pose(0) = msgs->poses.at(i).position.x;
      pose(1) = msgs->poses.at(i).position.y;
      pose(2) = tf::getYaw(msgs->poses.at(i).orientation);

      _poses.push_back(pose);
      _probs.push_back(msgs->poses.at(i).position.z); // z coordinate is used as weight of ceil cam measurement
    }

    _stamp = msgs->header.stamp;
  }

  ros::Time ROSCeilCamMeasurement::getStamp()
  {
    return _stamp;
  }

} /* namespace ohmPf */
