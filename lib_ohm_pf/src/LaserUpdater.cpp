/*
 * RosLaserPM.cpp
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#include "../include/LaserUpdater.h"

namespace ohmPf
{

  LaserUpdater::LaserUpdater(Filter* filter, IMap* map, ILaserMeasurement* measurement, MapUpdater* updateFilterMap) :
    FilterUpdaterMeasurementOCS(measurement, filter)
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
