/*
 * FilterUpdater.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "FilterUpdater.h"

namespace ohmPf
{

FilterUpdater::FilterUpdater(Filter* filter, std::string idString)
{
  _filter = filter;
  _idString = idString;
}

bool FilterUpdater::tryToUpdate()
{
  update();
  return true;
}

std::string FilterUpdater::getIdString()
{
  return _idString;
}

} /* namespace ohmPf */
