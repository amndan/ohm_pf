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
  _ocsObserver = NULL;
  _laserUpdater = NULL;
  _ceilCamUpdater = NULL;
  _outputUpdater = NULL;
  _mapUpdater = NULL;
  _resampler = NULL;
  _filter = NULL;

  _filterParams = params;
  _filter = new Filter(_filterParams);
  _ocsObserver = new OCSObserver();

  if(params.resamplingMethod == "STD")
  {
    _resampler = new STDResampler();
  }
  else if(params.resamplingMethod == "LV")
  {
    _resampler = new LVResampler();
  }
  else
  {
    std::cout << "Unknown resampler method! --> exit" << std::endl;
    exit(EXIT_FAILURE);
  }

  _ocsObserver->registerClient(_resampler, _filterParams.OCSThresholdResampler);


}

FilterController::~FilterController()
{
  // TODO Auto-generated destructor stub
}

bool FilterController::setMap(IMap* map)
{
  if(map != NULL)
  {
  _map = map;
  _mapUpdater = new MapUpdater(_filter, _map);
  return true;
  }
  std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
  return false;
}

bool FilterController::setOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params)
{
  if(odom != NULL)
  {
  _odomUpdater = new OdomUpdater(_filter, new OdomDiff(params), odom, _ocsObserver);
  _ocsObserver->registerClient(_odomUpdater, _filterParams.OCSThresholdOdom);
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
    _ocsObserver->registerClient(_laserUpdater, _filterParams.OCSThresholdLaser);
    return true;
  }
}

bool FilterController::setCeilCamMeasurement(ICeilCamMeasurement* ceilCam)
{
  if(_mapUpdater == NULL)
    {
      std::cout << __PRETTY_FUNCTION__ << "--> please init map before init ceilCam" << std::endl;
      return false;
    }
    else
    {
      _ceilCamUpdater = new CeilCamUpdater(_filter, ceilCam, _mapUpdater);
      return true;
    }
}

bool FilterController::setFilterOutput(IFilterOutput* output)
{
  if (output != NULL)
  {
    _filterOutput = output;
    _outputUpdater = new FilterOutputUpdater(_filterOutput, _filter);
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

bool FilterController::updateCeilCam()
{
  assert(_ceilCamUpdater != NULL);
  _ceilCamUpdater->update();
}

bool FilterController::updateOdom()
{
  assert(_odomUpdater != NULL);
  _odomUpdater->update();
}

bool FilterController::updateOutput()
{
  if(_outputUpdater != NULL)
  {
    _outputUpdater->update();
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
  }

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

bool FilterController::initFilterPose(Eigen::Vector3d pose, double sigTrans, double sigPhi)
{
  assert(_filter->getSamplesMax() != 0);

  std::vector<Sample_t> samples;

  for(unsigned int i = 0; i < _filter->getSamplesMax(); i++)
  {
    Sample_t sample;
    sample.pose = pose;
    sample.weight = 1.0 / (double) _filter->getSamplesMax();
    addGaussianRandomness(sample, sigTrans, sigPhi);
    samples.push_back(sample);
  }

  _filter->setSamples(samples);
  _filter->getSampleSet()->normalize();

  return true;
}

// factory method
IFilterController* IFilterController::createFilter(FilterParams_t params)
{
  return new FilterController(params);
}

} /* namespace ohmPf */

