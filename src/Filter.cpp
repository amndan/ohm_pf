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
    _sampleSet = NULL;
    _map = NULL;
    this->_paramSet = paramSet;
    _initialized = false;
  }

  Filter::~Filter()
  {
    delete _sampleSet;
    delete _map;
  }

  SampleSet* Filter::getSampleSet()
  {
    assert(_initialized);
    return _sampleSet;
  }

  Map* Filter::getMap()
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

  void Filter::initWithMap(Map* map)
  {
    double xMin;
    double yMin;
    double xMax;
    double yMax;

    delete _sampleSet;
    delete _map;
    _map = map;

    _map->getMinEnclRect(xMin, yMin, xMax, yMax);

    // generate cloud
    std::vector<Sample_t> samples;

    for(unsigned int i = 0; i < _paramSet.samplesMax; i++)
    {
      Sample_t sample;
      sample.weight = 1.0;
      sample.pose(2) = drand48() * 2 * M_PI - M_PI;
      // todo: more efficient way here
      do
      {
        sample.pose(0) = drand48() * (xMax - xMin) + xMin;
        sample.pose(1) = drand48() * (yMax - yMin) + yMin;
      }while( _map->isOccupied( sample.pose(0), sample.pose(1)) );
      //todo: check if there is at least one field not occupied

      samples.push_back(sample);
    }

    _sampleSet = new SampleSet(samples);
    _initialized = true;
  }

  void Filter::updateWithMap()
  {
    //assert(_map != NULL);

    if(_map == NULL)
    {
      std::cout<< __PRETTY_FUNCTION__ << "no map available; skip map update of filter" << std::endl;
      return;
    }

    std::vector<Sample_t>* samples = _sampleSet->getSamples();

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      if( _map->isOccupied( it->pose(0),it->pose(1) ) )
      {
        it->weight = 0.0;
      }
    }

  }

} /* namespace ohmPf */
