/*
 * ROSOdomMeasurement.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ROSODOMMEASUREMENT_H_
#define SRC_ROSODOMMEASUREMENT_H_

#include "IOdomMeasurement.h"
#include "Eigen/Dense"
#include "nav_msgs/Odometry.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"
#include "IOdomMeasurement.h"

namespace ohmPf
{

/**
 * @brief An implementaion of IOdomMeasurement for connecting a ros
 * odom message to the filter. For detailed
 * description of implementation of abstract members
 * please see IOdomMeasurement.
 */
class ROSOdomMeasurement : public IOdomMeasurement
{
public:
  ROSOdomMeasurement(){};
  virtual ~ROSOdomMeasurement(){};

  /**
   * @brief Sets a new measurement.
   * @param msgs ROS odom measurement.
   */
  void setMeasurement(const nav_msgs::OdometryConstPtr& msgs);

  void setMeasurement(const tf::StampedTransform& tf);

  /**
   * IMPLEMENTATION OF ABSTRACT MEMBERS
   */
  double getX();
  double getY();
  double getPhi();
  Eigen::Vector3d getMeasurement();
  ros::Time getStamp();

private:
  ros::Time _stamp;
  Eigen::Vector3d _measurement;
};

} /* namespace ohmPf */

#endif /* SRC_ROSODOMMEASUREMENT_H_ */
