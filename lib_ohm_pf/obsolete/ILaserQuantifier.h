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
#include "MapUpdater.h"

namespace ohmPf
{

/**
 * @brief ILaserQuantifier provides an interface for integrating an ILaserMeasurement
 * into a Filter instance. With connecting the IMap with the ILaserMeasurement the
 * Filters sample set gets updated by the Laser quantifier.
 */
class ILaserQuantifier
{
public:
  ILaserQuantifier(){};
  virtual ~ILaserQuantifier(){};

  /**
   * @brief An abstract class which must be implemented from every laser quantifier algorithm.
   * @param filter Reference to the filter to be updated.
   * @param measurement Reference to the laser measurement.
   * @param map Reference to the map.
   * @param MapUpdater pointer to the filters map updater.
   * @todo Use a function like filter->getMapUpdater to update filter with map!
   */
  virtual void calculate(Filter& filter, ILaserMeasurement& measurement, IMap& map, MapUpdater* updateFilterMap) = 0;
};

} /* namespace ohmPf */

#endif /* SRC_ILASERQUANTIFIER_H_ */
