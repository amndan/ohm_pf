/*
 * ROSOdomMeasurement.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "ROSOdomMeasurement.h"

namespace ohmPf
{

  ROSOdomMeasurement::ROSOdomMeasurement()
  {
    // TODO Auto-generated constructor stub

  }

  ROSOdomMeasurement::~ROSOdomMeasurement()
  {
    // TODO Auto-generated destructor stub
  }

  void ROSOdomMeasurement::setMeasurement(const nav_msgs::OdometryConstPtr& msgs)
  {
    _measurement(0) = msgs->pose.pose.position.x;
    _measurement(1) = msgs->pose.pose.position.y;
    _measurement(2) = tf::getYaw(msgs->pose.pose.orientation);
    _stamp = msgs->header.stamp;
  }

  double ROSOdomMeasurement::getX()
  {
    return _measurement(0);
  }

  double ROSOdomMeasurement::getY()
  {
    return _measurement(1);
  }

  double ROSOdomMeasurement::getPhi()
  {
    return _measurement(2);
  }

  Eigen::Vector3d ROSOdomMeasurement::getMeasurement()
  {
    return _measurement;
  }

  ros::Time ROSOdomMeasurement::getStamp()
  {
    return _stamp;
  }

} /* namespace ohmPf */
