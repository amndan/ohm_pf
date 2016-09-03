/*
 * FilterController.cpp
 *
 *  Created on: Mar 13, 2016
 *      Author: amndan
 */

#include "FilterController.h"

namespace ohmPf
{

FilterController::FilterController(FilterParams_t params) :
    _laserMeasurements(params.countLasers, NULL),
    _laserUpdaters(params.countLasers, NULL)
{
  _filterParams = params;

  _map = NULL;
  _odomMeasurement = NULL;
  _filterOutput = NULL;

  _odomUpdater = NULL;
  _ocsObserver = NULL;
  _ceilCamUpdater = NULL;
  _outputUpdater = NULL;
  _mapUpdater = NULL;
  _resampler = NULL;
  _filter = NULL;

  _laserQuantifier = new LaserProbMapMethod(_filterParams.minValidScanRaysFactor);

  _filter = new Filter(_filterParams);
  _ocsObserver = new OCSObserver(_filterParams.OCSRotToTransFactor);

  if(params.resamplingMethod == "STD")
  {
    _resampler = new STDResampler(
        params.resamplerAdditionalTranslationalNoise,
        params.resamplerAdditionalRotationalNoise);
  }
  else if(params.resamplingMethod == "LV")
  {
    _resampler = new LVResampler(
        params.resamplerAdditionalTranslationalNoise,
        params.resamplerAdditionalRotationalNoise,
        params.resamplerLowVarianceFactor);
  }
  else
  {
    std::cout << "Unknown resampler method! --> exit" << std::endl;
    exit(EXIT_FAILURE);
  }

  _ocsObserver->registerClient(_resampler, _filterParams.OCSThresholdResampler);
}


bool FilterController::setMap(IMap* map)
{
  // input map is NULL
  if(map == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
    return false;
  }

  // map is already set
  if(_map != NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> Cannot set multible maps!" << std::endl;
    return false;
  }

  // everything ok...
  _map = map;
  _mapUpdater = new MapUpdater(_filter, _map);
  return true;

}

bool FilterController::setOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params)
{
  // input pointer is NULL
  if(odom == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
    return false;
  }

  // odom measurement already set
  if(_odomUpdater != NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> cannot set multible odom measurements!" << std::endl;
    return false;
  }

  // everything ok...
  _odomUpdater = new DiffDriveUpdater(_filter, odom, _ocsObserver, params);
  _ocsObserver->registerClient(_odomUpdater, _filterParams.OCSThresholdOdom);
  return true;
}

bool FilterController::setLaserMeasurement(ILaserMeasurement* laser, unsigned int laserId)
{
  if(laserId >= _filterParams.countLasers)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> laser id must be less than count lasers" << std::endl;
    return false;
  }

  if(laser == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
    return false;
  }

  if(_mapUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> please init map before init laser" << std::endl;
    return false;
  }

  if(_laserUpdaters.at(laserId) != NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> cannot set multible laser measurements for one id" << std::endl;
    return false;
  }

  _laserUpdaters.at(laserId) = new LaserUpdater(_filter, _map, laser, _laserQuantifier, _mapUpdater);
  _ocsObserver->registerClient(_laserUpdaters.at(laserId), _filterParams.OCSThresholdLaser);
  return true;
}

bool FilterController::setCeilCamMeasurement(ICeilCamMeasurement* ceilCam)
{
  // wrong input
  if(ceilCam == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
    return false;
  }

  // no map inizialized
  if(_mapUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> please init map before init ceilCam" << std::endl;
    return false;
  }

  // ceilcam already initialized
  if(_ceilCamUpdater != NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> cannot init multible ceil cam updater" << std::endl;
    return false;
  }

  // everything ok...
  _ceilCamUpdater = new CeilCamUpdater(_filter, ceilCam, _mapUpdater);
  return true;
}

bool FilterController::setFilterOutput(IFilterOutput* output)
{
  if(output == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
    return false;
  }

  if(_filterOutput != NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> cannot init multible filter outputs" << std::endl;
    return false;
  }

  _filterOutput = output;
  _outputUpdater = new FilterOutputUpdater(_filterOutput, _filter);
  return true;
}

bool FilterController::updateLaser(unsigned int laserId)
{
  if(laserId >= _filterParams.countLasers)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> laser id must be less than count lasers" << std::endl;
    return false;
  }

  _laserUpdaters.at(laserId)->update();

  return true;
}

bool FilterController::updateCeilCam()
{
  if(_ceilCamUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> must set ceil cam measurement before update ceil cam" << std::endl;
    return false;
  }

  _ceilCamUpdater->update();

  return true;
}

bool FilterController::updateOdom()
{
  if(_odomUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> must set odom measurement before update odom" << std::endl;
    return false;
  }

  _odomUpdater->update();

  return true;
}

bool FilterController::updateOutput()
{

  if(_outputUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> must set filter output before update output" << std::endl;
    return false;
  }

  _outputUpdater->update();

  return true;

}

bool FilterController::resample()
{
  _resampler->resample(_filter);
  return true;
}

bool FilterController::initFilterMap()
{

  if(_mapUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> must set filter map befor init filter with map" << std::endl;
    return false;
  }

  _mapUpdater->initFilter();

  return true;
}

bool FilterController::initFilterPose(Eigen::Vector3d pose, double sigTrans, double sigPhi)
{
  if(_filter->getSamplesMax() == 0)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> cannot init filter wich filters samples max of 0" << std::endl;
    return false;
  }

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

