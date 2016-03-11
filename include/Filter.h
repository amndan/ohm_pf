/*
 * Filter.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_FILTER_H_
#define SRC_FILTER_H_

#include "IFilter.h"
#include "SampleSet.h"

namespace ohmPf
{

  class Filter// : public IFilter
  {
  public:
    Filter(std::vector<Sample_t> samples, unsigned int samplesMin, unsigned int samplesMax);
    virtual ~Filter();
    std::vector<Sample_t>* getSamples();
    void setSamples(std::vector<Sample_t> samples);
    unsigned int getSamplesMin();
    unsigned int getSamplesMax();
  private:
    SampleSet _sampleSet;
    unsigned int _samplesMin;
    unsigned int _samplesMax;
  };

} /* namespace ohmPf */

#endif /* SRC_FILTER_H_ */
