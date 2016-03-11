/*
 * MapUpdater.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "MapUpdater.h"

namespace ohmPf
{

  MapUpdater::MapUpdater(IFilter* filter, IMap* map) :
      FilterUpdater(filter)
  {
    _map = map;
  }

  MapUpdater::~MapUpdater()
  {
    // TODO Auto-generated destructor stub
  }

  void MapUpdater::update() // update weights
  {

  }

  void MapUpdater::updateForce() // update weights and replace particles
  {

  }

} /* namespace ohmPf */
