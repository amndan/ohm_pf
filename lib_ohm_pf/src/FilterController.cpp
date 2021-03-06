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
  _probMap = NULL;
  _odomMeasurement = NULL;
  _filterOutput = NULL;

  _odomUpdater = NULL;
  _ocsObserver = NULL;
  _poseUpdater = NULL;
  _outputUpdater = NULL;
  _mapUpdater = NULL;
  _resampler = NULL;
  _filter = NULL;

  _filter = new Filter(_filterParams);
  _ocsObserver = new OCSObserver(_filterParams.OCSRotToTransFactor);

  if(params.resamplingMethod == "STD")
  {
    STDResampler* tmp = new STDResampler(
        params.resamplerAdditionalTranslationalNoise,
        params.resamplerAdditionalRotationalNoise,
        _filter,
        "STR");
    _ocsObserver->registerClient( tmp, _filterParams.OCSThresholdResampler);
    _resampler = tmp;
  }
  else if(params.resamplingMethod == "LV")
  {
    LVResampler* tmp = new LVResampler(
        params.resamplerAdditionalTranslationalNoise,
        params.resamplerAdditionalRotationalNoise,
        params.resamplerLowVarianceFactor,
        _filter,
        "LVR");
    _ocsObserver->registerClient( tmp, _filterParams.OCSThresholdResampler);
    _resampler = tmp;
  }
  else
  {
    std::cout << "Unknown resampler method! --> exit" << std::endl;
    exit(EXIT_FAILURE);
  }

  _periodicFilterUpdatersWithoutMeasurement.push_back(_resampler);

}

void FilterController::filterSpinOnce()
{

  std::sort(
      _periodicFilterUpdatersWithMeasurement.begin(),
      _periodicFilterUpdatersWithMeasurement.end(),
      FilterController::sortMeasurementsLessThanOperator);

  for(unsigned int i = 0; i < _periodicFilterUpdatersWithMeasurement.size(); i++)
  {

#if TIMING == 1
    Timer timer("/tmp/" + _periodicFilterUpdatersWithMeasurement.at(i)->getIdString());
#endif

    if(_periodicFilterUpdatersWithMeasurement.at(i)->tryToUpdate())
    {

#if TIMING == 1
      timer.stopAndWrite();
#endif

    }
  }

  for(unsigned int i = 0; i < _periodicFilterUpdatersWithoutMeasurement.size(); i++)
  {

#if TIMING == 1
    Timer timer("/tmp/" + _periodicFilterUpdatersWithoutMeasurement.at(i)->getIdString());
#endif

    if(_periodicFilterUpdatersWithoutMeasurement.at(i)->tryToUpdate())
    {

#if TIMING == 1
      timer.stopAndWrite();
#endif

    }
  }

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
  _probMap = new ProbMap(*map,_filterParams.maxDistanceProbMap);
  _mapUpdater = new MapUpdater(_filter, _map, "MAP");
  return true;
}

bool FilterController::connectOdomMeasurement(IOdomMeasurement* odom, OdomParams_t params)
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

  if(params.model == 0) //diff
  {
    _odomUpdater = new DiffDriveUpdater(_filter, odom, _ocsObserver, params, "ODM");
  }
  else if(params.model == 1) //omni
  {
    _odomUpdater = new OmniDriveUpdater(_filter, odom, _ocsObserver, params, "ODM");
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << "--> Wrong odom model id. Use 0:diff or 1:omni." << std::endl;
    return false;
  }

  _ocsObserver->registerClient(_odomUpdater, _filterParams.OCSThresholdOdom);
  _periodicFilterUpdatersWithMeasurement.push_back(_odomUpdater);
  return true;
}

bool FilterController::connectLaserMeasurement(ILaserMeasurement* laser, unsigned int laserId)
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

  if(laserId == 0 && _filter->getParams().useAdaptiveMean)
  {
    // set activateAdaptiveMean to true for more sample analysis
    // only one laser updater should set this to true
    _laserUpdaters.at(laserId) = new LaserProbMapUpdater(
        _filter, _probMap, laser, _mapUpdater, _filterParams.minValidScanRaysFactor, "LPM", true);
  }
  else
  {
    _laserUpdaters.at(laserId) = new LaserProbMapUpdater(
        _filter, _probMap, laser, _mapUpdater, _filterParams.minValidScanRaysFactor, "LPM", false);
  }

  _ocsObserver->registerClient(_laserUpdaters.at(laserId), _filterParams.OCSThresholdLaser);
  _periodicFilterUpdatersWithMeasurement.push_back(_laserUpdaters.at(laserId));
  return true;
}

bool FilterController::connectPoseMeasurement(IPoseMeasurement* pose)
{
  // wrong input
  if(pose == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> no NULL pointer here!" << std::endl;
    return false;
  }

  // no map inizialized
  if(_mapUpdater == NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> please init map before init pose measurement" << std::endl;
    return false;
  }

  // pose measurement already initialized
  if(_poseUpdater != NULL)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> cannot init multible pose measurement updater" << std::endl;
    return false;
  }

  // everything ok...
  _poseUpdater = new PoseUpdater(_filter, pose, _mapUpdater, "CCM");
  _periodicFilterUpdatersWithMeasurement.push_back(_poseUpdater);
  return true;
}

bool FilterController::connectFilterOutput(IFilterOutput* output)
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
  _outputUpdater = new FilterOutputUpdater(_filterOutput, _filter, "OUT");
  _periodicFilterUpdatersWithoutMeasurement.push_back(_outputUpdater);
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

void FilterController::requestProbMap(unsigned int& width, unsigned int& height, double resolution,
                                     Eigen::Matrix3d originTf, std::vector<int8_t>& mapData)
{
  if(_probMap != NULL)
  {
    width = _probMap->getWidthInCells();
    height = _probMap->getHeighInCells();
    resolution = _probMap->getResolution();
    originTf = _probMap->getTfMapToMapOrigin();
    mapData = _probMap->getMapData();
  }
  else
  {
    width = 0;
    height = 0;
    mapData.clear();
  }
}

// factory method
IFilterController* IFilterController::createFilter(FilterParams_t params)
{
  return (IFilterController*) new FilterController(params);
}

} /* namespace ohmPf */

