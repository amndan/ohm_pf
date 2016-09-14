/*
 * FilterUpdaterTimed.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#include "FilterUpdaterTimed.h"

namespace ohmPf
{

FilterUpdaterTimed::FilterUpdaterTimed(Filter* filter, ros::Duration intervall) :
    FilterUpdater(filter)
{
  _intervall = intervall;
  _lastStamp = ros::Time::now() - intervall; // force update at first iter

}

bool FilterUpdaterTimed::tryToUpdate()
{
  ros::Time now = ros::Time::now();

  if(now - _lastStamp >= _intervall)
  {
    update();
    _lastStamp = now;
    return true;
  }

  return false;
}

} /* namespace ohmPf */
