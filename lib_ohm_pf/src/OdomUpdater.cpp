/*
 * OdomUpdater.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "OdomUpdater.h"

namespace ohmPf
{

OdomUpdater::OdomUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver) :
    FilterUpdaterMeasurementOCS(measurement, filter)
{
  _odomMeasurement = measurement;
  _ocsObserver = ocsObserver;
}

void OdomUpdater::tryToUpdate()
{
  _ocsObserver->update(_odomMeasurement);
  FilterUpdaterMeasurementOCS::tryToUpdate();
}

void OdomUpdater::update()
{
    calculate();
}

} /* namespace ohmPf */
