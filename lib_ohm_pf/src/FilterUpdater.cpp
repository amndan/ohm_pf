/*
 * FilterUpdater.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "FilterUpdater.h"

namespace ohmPf
{

FilterUpdater::FilterUpdater(Filter* filter)
{
  _filter = filter;
}

void FilterUpdater::tryToUpdate()
{
  update();
}

} /* namespace ohmPf */
