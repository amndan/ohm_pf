/*
 * FilterUpdater.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "FilterUpdater.h"

namespace ohmPf
{
  FilterUpdater::FilterUpdater(IFilter* filter)
  {
    _filter = filter;
  }

  ros::Time FilterUpdater::getStamp()
  {
    return _stamp;
  }

} /* namespace ohmPf */
