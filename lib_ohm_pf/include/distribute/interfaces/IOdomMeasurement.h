/*
 * IOdomMeasurement.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_IODOMMEASUREMENT_H_
#define SRC_IODOMMEASUREMENT_H_

#include "IMeasurement.h"
#include "Eigen/Dense"

namespace ohmPf
{

/**
 * @brief An Interface for odom measurements.
 */
class IOdomMeasurement : public IMeasurement
{
public:
  IOdomMeasurement(){};
  virtual ~IOdomMeasurement(){};

  /**
   * @return returns the x value of the actual odom measurement in m.
   */
  virtual double getX() = 0;

  /**
   * @return returns the y value of the actual odom measurement in m.
   */
  virtual double getY() = 0;

  /**
   * @return returns the phi value of the actual odom measurement in rad.
   */
  virtual double getPhi() = 0;

  /**
   * @return returns the actual measurement as vector - Format:
   * x in meter
   * y in meter
   * phi in rad
   */
  virtual Eigen::Vector3d getMeasurement() = 0;
};

} /* namespace ohmPf */

#endif /* SRC_IODOMMEASUREMENT_H_ */
