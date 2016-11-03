/*
 * FilterUpdaterTimed.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#include "FilterUpdaterTimed.h"

namespace ohmPf
{

FilterUpdaterTimed::FilterUpdaterTimed(Filter* filter, evo::Duration intervall, std::string idString) :
    FilterUpdater(filter, idString)
{

  _intervall = intervall;
  _lastStamp = evo::Time::now() - intervall; // force update at first iter

}

bool FilterUpdaterTimed::tryToUpdate()
{
  evo::Time now = evo::Time::now();

  if(now - _lastStamp >= _intervall)
  {
    update();
    _lastStamp = now;
    return true;
  }

  return false;
}

} /* namespace ohmPf */
