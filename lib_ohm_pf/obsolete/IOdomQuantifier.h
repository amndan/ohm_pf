/*
 * IOdomQuantifier.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_IODOMQUANTIFIER_H_
#define SRC_IODOMQUANTIFIER_H_

#include "IOdomMeasurement.h"
#include "Filter.h"

namespace ohmPf
{

/**
 * @brief Interface for implementing different odometry models (e.g. diff drive, omni drive, ...)
 */
class IOdomQuantifier
{
public:
  IOdomQuantifier(){};
  virtual ~IOdomQuantifier(){};

  /**
   * @brief Abstract calculate function. Each OdomQuantifier
   * @param filter
   * @param measurement
   */
  virtual void calculate(Filter& filter, IOdomMeasurement& measurement) = 0;

};

} /* namespace ohmPf */

#endif /* SRC_IODOMQUANTIFIER_H_ */
