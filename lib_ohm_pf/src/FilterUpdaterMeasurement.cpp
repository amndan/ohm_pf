/*
 * FilterUpdaterMeasurement.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#include "FilterUpdaterMeasurement.h"

namespace ohmPf
{

FilterUpdaterMeasurement::FilterUpdaterMeasurement(IMeasurement* measurement, Filter* filter, std::string idString) :
    FilterUpdater(filter, idString)
{
  //init last stamp
  _lastStamp = measurement->getStamp();
  _measurement = measurement;
}

bool FilterUpdaterMeasurement::tryToUpdate()
{
  if(_lastStamp != _measurement->getStamp()) // if stamp has changed
  {
    update();
    _lastStamp = _measurement->getStamp();
    return true;
  }
  else
  {
    //std::cout << __PRETTY_FUNCTION__ << "--> will not update; stamp has not changed!" << std::endl;
    return false;
  }
}

} /* namespace ohmPf */
