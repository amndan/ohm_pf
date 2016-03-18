/*
 * ROSCeilCamMeasurement.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ROSCEILCAMMEASUREMENT_H_
#define SRC_ROSCEILCAMMEASUREMENT_H_

#include "geometry_msgs/PoseArray.h"
#include "ICeilCamMeasurement.h"
#include "Eigen/Dense"
#include "tf/transform_datatypes.h"

namespace ohmPf
{

  class ROSCeilCamMeasurement : public ICeilCamMeasurement
  {
  public:
    ROSCeilCamMeasurement();
    virtual ~ROSCeilCamMeasurement();
    std::vector<Eigen::Vector3d> getPoses();
    std::vector<double> getProbabilities();
    void setMeasurement(const geometry_msgs::PoseArrayConstPtr& msgs);
    ros::Time getStamp();
  private:
    std::vector<Eigen::Vector3d> _poses;
    std::vector<double> _probs;
    ros::Time _stamp;
  };

} /* namespace ohmPf */

#endif /* SRC_ROSCEILCAMMEASUREMENT_H_ */
