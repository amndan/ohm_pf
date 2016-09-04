/*
 * ICeilCamMeasurement.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ICEILCAMMEASUREMENT_H_
#define SRC_ICEILCAMMEASUREMENT_H_

#include "IMeasurement.h"
#include <vector>
#include "Eigen/Dense"

namespace ohmPf
{

/**
 * An abstract class for providing a generalized ceil cam
 * measurement to the filter.
 */
class ICeilCamMeasurement : public IMeasurement
{
public:
  ICeilCamMeasurement(){};
  virtual ~ICeilCamMeasurement(){};

  /**
   * @brief An abstract function to get the poses of the
   * actual measurement. Poses are in map frame.
   * Structure is: x y yaw in: m m rad
   * @return The poses vector.
   */
  virtual std::vector<Eigen::Vector3d> getPoses() = 0;

  /**
   * @brief An abstract function for getting the probabilities
   * of the corresponding poses from getPoses().
   * @return The probabilities vector.
   */
  virtual std::vector<double> getProbabilities() = 0;
};

} /* namespace ohmPf */

#endif /* SRC_ICEILCAMMEASUREMENT_H_ */
