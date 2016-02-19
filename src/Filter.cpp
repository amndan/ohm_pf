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
    _paramSet = paramSet;
    _sampleSet = NULL;

    for(int i = 0; i < CNT_SENSORS; i++)
    {
      _sensors[i] = NULL;
      _sensorsInitialized[i] = false;
      _sensorsUpdateDistance[i] = 0.5; // todo: magic number
      _sensorsOdomChangedSignificantly[i] = true;
    }

    _initialized = false;
  }

  Filter::~Filter()
  {

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

    delete _sampleSet;
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

    if(_sensorsOdomChangedSignificantly[sensorID])
    {
      _sensors[sensorID]->updateFilter(*this);
      _sensorsOdomChangedSignificantly[sensorID] = false; // no update til next sgnificant change in odom
    }

    return;
  }

  void Filter::setSensor(int sensorID, Sensor* pSensor)
  {
    assert(sensorID < CNT_SENSORS);
    _sensors[sensorID] = pSensor;
  }

  void Filter::triggerOdomChangedSignificantly()
  {
    for(int i = 0; i < CNT_SENSORS; i++)
    {
      _sensorsOdomChangedSignificantly[i] = true;
    }
  }

} /* namespace ohmPf */

