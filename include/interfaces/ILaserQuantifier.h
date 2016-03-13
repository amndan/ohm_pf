/*
 * ILaserQuantifier.h
 *
 *  Created on: Mar 10, 2016
 *      Author: amndan
 */

#ifndef SRC_ILASERQUANTIFIER_H_
#define SRC_ILASERQUANTIFIER_H_

#include "Filter.h"
#include "ILaserMeasurement.h"
#include "IMap.h"
#include "IUpdateFilterMap.h"

namespace ohmPf
{

class ILaserQuantifier
{
public:
  ILaserQuantifier(){};
  virtual ~ILaserQuantifier(){};
  virtual void calculate(Filter& filter, ILaserMeasurement& measurement, IMap& map, IUpdateFilterMap& updateFilterMap) = 0;
};

} /* namespace ohmPf */

#endif /* SRC_ILASERQUANTIFIER_H_ */
