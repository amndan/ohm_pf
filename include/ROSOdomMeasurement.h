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

  class ROSOdomMeasurement : public IOdomMeasurement
  {
  public:
    ROSOdomMeasurement();
    virtual ~ROSOdomMeasurement();
    double getX();
    double getY();
    double getPhi();
    Eigen::Vector3d getMeasurement();
    ros::Time getStamp();
    void setMeasurement(const nav_msgs::OdometryConstPtr& msgs);

  private:
    ros::Time _stamp;
    Eigen::Vector3d _measurement;
  };

} /* namespace ohmPf */

#endif /* SRC_ROSODOMMEASUREMENT_H_ */
