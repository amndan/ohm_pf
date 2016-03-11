/*
 * ILaserQuantifier.h
 *
 *  Created on: Mar 10, 2016
 *      Author: amndan
 */

#ifndef SRC_ILASERQUANTIFIER_H_
#define SRC_ILASERQUANTIFIER_H_

#include "ILaserMeasurement.h"
#include "IMap.h"
#include "IFilter.h"

namespace ohmPf
{

class ILaserQuantifier
{
public:
  ILaserQuantifier(){};
  virtual ~ILaserQuantifier(){};
  virtual void calculate(IFilter& filter, ILaserMeasurement& measurement, IMap& map) = 0;
};

} /* namespace ohmPf */

#endif /* SRC_ILASERQUANTIFIER_H_ */
