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
      FilterUpdater(filter)
  {
    _measurement = measurement;
    _ocsObserver = ocsObserver;
  }

  void OdomUpdater::update()
  {
    _ocsObserver->update(_measurement);

    if(this->getOCSFlag() == true)
    {
      calculate();
    }
  }

} /* namespace ohmPf */
