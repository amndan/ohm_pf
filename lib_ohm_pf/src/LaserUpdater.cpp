/*
 * RosLaserPM.cpp
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#include "../include/LaserUpdater.h"

namespace ohmPf
{

  LaserUpdater::LaserUpdater(Filter* filter, IMap* map, ILaserMeasurement* measurement, MapUpdater* updateFilterMap, std::string idString) :
    FilterUpdaterMeasurementOCS(measurement, filter, idString)
  {
    _map = map;
    _laserMeasurement = measurement;
    _updateFilterMap = updateFilterMap;
  }

  void LaserUpdater::update()
  {
    calculate();
  }

} /* namespace ohmPf */
