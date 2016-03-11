/*
 * LaserProbMapMethod.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_LASERPROBMAPMETHOD_H_
#define SRC_LASERPROBMAPMETHOD_H_

#include "ILaserQuantifier.h"
#include "IMap.h"
#include "ILaserMeasurement.h"
#include "IFilter.h"
#include "UtilitiesOhmPf.h"
#include "Eigen/Dense"

namespace ohmPf
{
  class LaserProbMapMethod : public ILaserQuantifier
  {
  public:
    LaserProbMapMethod();
    virtual ~LaserProbMapMethod();
    void calculate(IFilter& filter, ILaserMeasurement& measurement, IMap& map, IUpdateFilterMap& updateFilterMap);
  private:
    Eigen::Matrix3Xd rangesToCoordinates(ILaserMeasurement& measurement);
  };
} /* namespace ohmPf */

#endif /* SRC_LASERPROBMAPMETHOD_H_ */
