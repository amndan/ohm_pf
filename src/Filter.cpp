/*
 * Filter.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "Filter.h"

namespace ohmPf
{

Filter::Filter(std::vector<Sample_t> samples) :
    _sampleSet(samples)
{

}

Filter::~Filter()
{
  // TODO Auto-generated destructor stub
}

std::vector<Sample_t>* Filter::getSamples()
{
  return _sampleSet.getSamples();
}

void Filter::setSamples(std::vector<Sample_t> samples)
{
  _sampleSet.setSamples(samples);
}

} /* namespace ohmPf */
