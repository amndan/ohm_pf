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

  void Filter::initWithPose(Eigen::Vector3d pose)
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

} /* namespace ohmPf */
