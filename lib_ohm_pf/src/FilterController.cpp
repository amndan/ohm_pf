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
  _ceilCamUpdater = NULL;
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

  _periodicFilterUpdaters.push_back(_resampler);

}

void FilterController::filterSpinOnce()
{

  if(false)
  {
    std::vector<double> times;

    std::cout << "|";

    for(unsigned int i = 0; i < _periodicFilterUpdaters.size(); i++)
    {
      std::cout << _periodicFilterUpdaters.at(i)->getIdString() << "|";
    }

    std::cout << "|" << std::endl;

    std::cout << "|";

    for(unsigned int i = 0; i < _periodicFilterUpdaters.size(); i++)
    {
      Timer timer("/tmp/" + _periodicFilterUpdaters.at(i)->getIdString());

      if(_periodicFilterUpdaters.at(i)->tryToUpdate())
      {
        std::cout << " x |";
        timer.stopAndWrite();
        times.push_back( timer.getTimeInMs() );
      }
      else
      {
        std::cout << " o |";
      }
    }

    std::cout << "| ";

    for (unsigned int i = 0; i < times.size(); i++)
    {
      std::cout << (int)times.at(i) << " |";
    }

    std::cout << std::endl;
  }
  else
  {
    for(unsigned int i = 0; i < _periodicFilterUpdaters.size(); i++)
    {
      _periodicFilterUpdaters.at(i)->tryToUpdate();
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

bool FilterController::connectOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params)
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
  _odomUpdater = new DiffDriveUpdater(_filter, odom, _ocsObserver, params, "ODM");
  _ocsObserver->registerClient(_odomUpdater, _filterParams.OCSThresholdOdom);
  _periodicFilterUpdaters.push_back(_odomUpdater);
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

  _laserUpdaters.at(laserId) = new LaserProbMapUpdater(_filter, _probMap, laser, _mapUpdater, _filterParams.minValidScanRaysFactor, "LPM");
  _ocsObserver->registerClient(_laserUpdaters.at(laserId), _filterParams.OCSThresholdLaser);
  _periodicFilterUpdaters.push_back(_laserUpdaters.at(laserId));
  return true;
}

bool FilterController::connectCeilCamMeasurement(ICeilCamMeasurement* ceilCam)
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
  _ceilCamUpdater = new CeilCamUpdater(_filter, ceilCam, _mapUpdater, "CCM");
  _periodicFilterUpdaters.push_back(_ceilCamUpdater);
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
  _periodicFilterUpdaters.push_back(_outputUpdater);
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

