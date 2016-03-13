/*
 * FilterController.cpp
 *
 *  Created on: Mar 13, 2016
 *      Author: amndan
 */

#include "FilterController.h"

namespace ohmPf
{

FilterController::FilterController(FilterParams_t params)
{
  _map = NULL;
  _odomMeasurement = NULL;
  _laserMeasurement = NULL;
  _filterOutput = NULL;

  _odomUpdater = NULL;
  _laserUpdater = NULL;
  _outputUpdater = NULL;
  _mapUpdater = NULL;
  _resampler = NULL;
  _filter = NULL;

  _filter = new Filter(params);
}

FilterController::~FilterController()
{
  // TODO Auto-generated destructor stub
}

bool FilterController::setMap(IMap* map)
{
  if(map != NULL)
  {
  _mapUpdater = new MapUpdater(_filter, map);
  return true;
  }
  std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
  return false;
}

bool FilterController::setOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params)
{
  if(odom != NULL)
  {
  _odomUpdater = new OdomUpdater(_filter, new OdomDiff(params), odom);
  return true;
  }
  std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
  return false;
}

bool FilterController::setLaserMeasurement(ILaserMeasurement* laser)
{
  if(_mapUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> please init map before init laser" << std::endl;
    return false;
  }
  else
  {
    _laserUpdater = new LaserUpdater(_filter, _map, laser, new LaserProbMapMethod(), _mapUpdater);
    return true;
  }
}
bool FilterController::setFilterOutput(IFilterOutput* output)
{
  if (output != NULL)
  {
    _filterOutput = output;
    return true;
  }
  std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
  return false;
}

bool FilterController::updateLaser()
{
  assert(_laserUpdater != NULL);
  _laserUpdater->update();
}

bool FilterController::updateOdom()
{
  assert(_odomUpdater != NULL);
  _odomUpdater->update();
}

bool FilterController::updateOutput()
{
  assert(_outputUpdater != NULL);
  _outputUpdater->update();
}

bool FilterController::resample()
{
  assert(_resampler != NULL);
  _resampler->resample(_filter);
}

bool FilterController::initFilterMap()
{
  assert(_mapUpdater != NULL);
  _mapUpdater->initFilter();
}

// factory method
IFilterController* IFilterController::createFilter(FilterParams_t params)
{
  return new FilterController(params);
}

} /* namespace ohmPf */

