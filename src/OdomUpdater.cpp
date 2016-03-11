/*
 * OdomUpdater.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "OdomUpdater.h"

namespace ohmPf
{

  OdomUpdater::OdomUpdater(IFilter* filter, IOdomQuantifier* quantifier, IOdomMeasurement* measurement, IUpdateFilterMap* updateFilterMap) :
      FilterUpdater(filter)
  {
    _OCSFlag = true;
    _quantifier = quantifier;
    _measurement = measurement;
    _updateFilterMap = updateFilterMap;
  }

  OdomUpdater::~OdomUpdater()
  {
    // TODO Auto-generated destructor stub
  }

  void OdomUpdater::update()
  {
    //TODO: here comes the ocs management?

    _quantifier->calculate(*_filter, *_measurement, *_updateFilterMap);
    // perhaps create function _quantifier.getCumultativeOdom() and sum it up for each client
  }

  void OdomUpdater::setOCSFlagTrue()
  {
    _OCSFlag = true;
  }

} /* namespace ohmPf */
