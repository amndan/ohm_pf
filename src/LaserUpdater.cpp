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
      FilterUpdater(filter)
  {
    _map = map;
    _measurement = measurement;
    _updateFilterMap = updateFilterMap;
  }

  void LaserUpdater::update()
  {

    if (this->getOCSFlag() == true)
    {
      calculate();
    }
  }

} /* namespace ohmPf */

