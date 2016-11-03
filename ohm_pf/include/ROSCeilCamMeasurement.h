/*
 * ROSCeilCamMeasurement.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ROSCEILCAMMEASUREMENT_H_
#define SRC_ROSCEILCAMMEASUREMENT_H_

#include "geometry_msgs/PoseArray.h"
#include "IPoseMeasurement.h"
#include "Eigen/Dense"
#include "tf/transform_datatypes.h"

namespace ohmPf
{

/**
 * @brief A measurement container for ROS ceil cam measurements.
 * It implements IPoseMeasurement to provide ceil cam measurements
 * from ROS to the filter under a generalized interface.
 */
class ROSCeilCamMeasurement : public IPoseMeasurement
{
public:

  /**
   * @brief Constructor
   */
  ROSCeilCamMeasurement();

  /**
   * @brief Destructor (empty)
   */
  virtual ~ROSCeilCamMeasurement(){};

  /**
   * @brief Poses getter.
   * @return Array of poses.
   */
  std::vector<Eigen::Vector3d> getPoses();

  /**
   * @brief Probabilities getter.
   * @return Array of probabilities corresponding to poses.
   */
  std::vector<double> getProbabilities();

  /**
   * @brief Set actual measurement.
   * @param msgs ROS pose array.
   */
  void setMeasurement(const geometry_msgs::PoseArrayConstPtr& msgs);

  /**
   * @brief get measurements timestamp.
   * @return ROS timestamp.
   * @todo Implement timing. Its easy to let every updater set one filter stamp
   * with a function in the filter to track or log timing. E.g. something like:
   * _filter.setStamp(ros::Time stamp, string id)...
   */
  evo::Time getStamp();

private:
  std::vector<Eigen::Vector3d> _poses;
  std::vector<double> _probs;
  ros::Time _stamp;
};

} /* namespace ohmPf */

#endif /* SRC_ROSCEILCAMMEASUREMENT_H_ */
