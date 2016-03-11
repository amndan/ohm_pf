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

  class IOdomMeasurement : public IMeasurement
  {
  public:
    IOdomMeasurement(){};
    virtual ~IOdomMeasurement(){};
    virtual bool isInitialized() = 0;
    virtual double getX() = 0;
    virtual double getY() = 0;
    virtual double getPhi() = 0;
    virtual Eigen::Vector3d getMeasurement() = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_IODOMMEASUREMENT_H_ */
