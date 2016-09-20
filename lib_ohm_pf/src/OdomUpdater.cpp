/*
 * OdomUpdater.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "OdomUpdater.h"

namespace ohmPf
{

OdomUpdater::OdomUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver, std::string idString) :
    FilterUpdaterMeasurementOCS(measurement, filter, idString)
{
  _odomMeasurement = measurement;
  _ocsObserver = ocsObserver;
}

bool OdomUpdater::tryToUpdate()
{
  _ocsObserver->update(_odomMeasurement);
  return FilterUpdaterMeasurementOCS::tryToUpdate();
}

void OdomUpdater::update()
{
    calculate();
}

} /* namespace ohmPf */
