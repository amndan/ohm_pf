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
    this->_paramSet = paramSet;
    _initialized = false;
  }

  Filter::~Filter()
  {
    delete _sampleSet;
  }

  SampleSet* Filter::getSampleSet()
  {
    assert(_initialized);
    return _sampleSet;
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

  void Filter::initWithMap()
  {
    double x = 20;
    double y = 20;

    // generate cloud
    std::vector<Sample_t> samples;

    for(unsigned int i = 0; i < _paramSet.samplesMax; i++)
    {
      Sample_t sample;
      sample.weight = 1;
      sample.pose(0) = drand48() * x;
      sample.pose(1) = drand48() * y;
      sample.pose(2) = drand48() * 2 * M_PI - M_PI;
      samples.push_back(sample);
    }

    _sampleSet = new SampleSet(samples);
    _initialized = true;
  }

} /* namespace ohmPf */
