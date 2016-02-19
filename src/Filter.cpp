/*
 * Filter.cpp
 *
 *  Created on: 08.01.2016
 *      Author: amndan
 */

#include "Filter.h"

namespace ohmPf
{

  Filter::Filter(FilterParams_t paramSet)
  {
    _map = NULL;
    _paramSet = paramSet;
    _sampleSet = NULL;

    for(int i = 0; i < CNT_SENSORS; i++)
    {
      _sensors[i] = NULL;
    }

    _initialized = false;
  }

  Filter::~Filter()
  {
    delete _map;
  }

  SampleSet* Filter::getSampleSet()
  {
    assert(_initialized);
    return _sampleSet;
  }

  Sensor& Filter::getSensor(int sensorID)
  {
    assert(sensorID < CNT_SENSORS);
    return *_sensors[sensorID];
  }

  void Filter::setSamples(std::vector<Sample_t> samples)
  {
    // todo: check plausibility of sample set
    delete _sampleSet;
    _sampleSet = new SampleSet(samples);
  }

  MapModel* Filter::getMap()
  {
    return _map;
  }

  bool Filter::isInitialized()
  {
    return _initialized;
  }

  void Filter::initWithPose(const Eigen::Vector3d& pose)
  {
    // generate cloud
    std::vector<Sample_t> samples;

    for(unsigned int i = 0; i < _paramSet.samplesMax; i++)
    {
      samples.push_back(GaussianPdf::getRandomSample(pose, 0.5, 0.2));
    }

    _sampleSet = new SampleSet(samples);
    _initialized = true;
  }


  FilterParams_t const * const Filter::getParamSet()
  {
    return &_paramSet;
  }

  void Filter::initWithSensor(int sensorID)
  {
    assert(sensorID < CNT_SENSORS);
    assert(_sensors[sensorID] != NULL);

    if(_sensors[sensorID] == NULL)
    {
      std::cout << __PRETTY_FUNCTION__ << "sensorID " <<  sensorID << " not available; skip map update of filter" << std::endl;
      return;
    }

    _sensors[sensorID]->initFilter(*this);

    _initialized = true;

    return;
  }

  void Filter::updateWithSensor(int sensorID)
  {
    assert(sensorID < CNT_SENSORS);


    if(_sensors[sensorID] == NULL)
    {
      std::cout << __PRETTY_FUNCTION__ << "sensorID " <<  sensorID << " not available; skip update of filter" << std::endl;
      return;
    }

    _sensors[sensorID]->updateFilter(*this);

    return;
  }

  void Filter::setSensor(int sensorID, Sensor* pSensor)
  {
    assert(sensorID < CNT_SENSORS);
    _sensors[sensorID] = pSensor;
  }

} /* namespace ohmPf */

