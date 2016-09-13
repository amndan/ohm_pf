/*
 * FilterUpdaterMeasurement.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#include "FilterUpdaterMeasurement.h"

namespace ohmPf
{

FilterUpdaterMeasurement::FilterUpdaterMeasurement(IMeasurement* measurement, Filter* filter) :
    FilterUpdater(filter)
{
  //init last stamp
  _lastStamp = measurement->getStamp();
  _measurement = measurement;
}

void FilterUpdaterMeasurement::tryToUpdate()
{
  if(_lastStamp != _measurement->getStamp()) // if stamp has changed
  {
    update();
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << "--> will not update; stamp has not changed!" << std::endl;
  }
}

} /* namespace ohmPf */
