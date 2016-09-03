/*
 * LaserProbMapUpdater.h
 *
 *  Created on: Sep 3, 2016
 *      Author: amndan
 */

#ifndef SRC_LASERPROBMAPUPDATER_H_
#define SRC_LASERPROBMAPUPDATER_H_

#include "IMap.h"
#include "ILaserMeasurement.h"
#include "MapUpdater.h"
#include "LaserUpdater.h"
#include "Filter.h"
#include "UtilitiesOhmPf.h"
#include "Eigen/Dense"

namespace ohmPf
{

class LaserProbMapUpdater: public LaserUpdater
{
public:

  LaserProbMapUpdater(Filter* filter, IMap* map, ILaserMeasurement* measurement, MapUpdater* updateFilterMap, double minValidRaysFactor);

  virtual ~LaserProbMapUpdater(){};

  void calculate();

private:
  Eigen::Matrix3Xd rangesToCoordinates(ILaserMeasurement& measurement);

  double _minValidRaysFactor;
};

} /* namespace ohmPf */

#endif /* SRC_LASERPROBMAPUPDATER_H_ */
