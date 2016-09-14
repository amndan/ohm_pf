/*
 * FilterUpdaterMeasurementOCS.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#include "FilterUpdaterMeasurementOCS.h"

namespace ohmPf
{
//class FilterUpdaterMeasurementOCS : public FilterUpdaterMeasurement, public OCSClient
FilterUpdaterMeasurementOCS::FilterUpdaterMeasurementOCS(IMeasurement* measurement, Filter* filter) :
    FilterUpdaterMeasurement(measurement, filter), OCSClient()
{
  //filter->getOCSObserver.registerClient(this);
  // inherit private from ocs client??

  _OCSactive = true;

}


bool FilterUpdaterMeasurementOCS::tryToUpdate()
{
  if( _lastStamp == _measurement->getStamp())
  {
    //std::cout << __PRETTY_FUNCTION__ << "--> will not update; stamp has not changed!" << std::endl;
    return false;
  }

  if(!getOCSFlag(true) && _OCSactive)
  {
    //std::cout << __PRETTY_FUNCTION__ << "--> will not update; no OCS!" << std::endl;
    return false;
  }

  update();
  _lastStamp = _measurement->getStamp();
  return true;

}

void FilterUpdaterMeasurementOCS::activateOCS()
{
  std::cout << __PRETTY_FUNCTION__ << "--> activated OCS." << std::endl;
  _OCSactive = true;
}

void FilterUpdaterMeasurementOCS::deactivateOCS()
{
  std::cout << __PRETTY_FUNCTION__ << "--> deactivated OCS" << std::endl;
  _OCSactive = false;
}

} /* namespace ohmPf */
