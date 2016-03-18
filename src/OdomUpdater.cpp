/*
 * OdomUpdater.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "OdomUpdater.h"

namespace ohmPf
{

  OdomUpdater::OdomUpdater(Filter* filter, IOdomQuantifier* quantifier, IOdomMeasurement* measurement, OCSObserver* ocsObserver) :
      FilterUpdater(filter)
  {
    _OCSFlag = true;
    _quantifier = quantifier;
    _measurement = measurement;
    _ocsObserver = ocsObserver;
  }

  OdomUpdater::~OdomUpdater()
  {
    // TODO Auto-generated destructor stub
  }

  void OdomUpdater::update()
  {
    _ocsObserver->update(_measurement);
    // if ocs true...
    _quantifier->calculate(*_filter, *_measurement);
    // perhaps create function _quantifier.getCumultativeOdom() and sum it up for each client
  }

  void OdomUpdater::setOCSFlagTrue()
  {
    _OCSFlag = true;
  }

} /* namespace ohmPf */
