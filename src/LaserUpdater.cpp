/*
 * RosLaserPM.cpp
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#include "../include/LaserUpdater.h"

namespace ohmPf
{

  LaserUpdater::LaserUpdater(Filter* filter, IMap* map, ILaserMeasurement* measurement, ILaserQuantifier* quantifier, MapUpdater* updateFilterMap) :
      FilterUpdater(filter)
  {
    _map = map;
    _quantifier = quantifier;
    _measurement = measurement;
    _updateFilterMap = updateFilterMap;
  }

  void LaserUpdater::update()
  {

    if (this->getOCSFlag() == true)
    {
      _quantifier->calculate(*_filter, *_measurement, *_map, _updateFilterMap);
    }
  }

} /* namespace ohmPf */

