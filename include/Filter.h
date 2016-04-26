/*
 * Filter.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_FILTER_H_
#define SRC_FILTER_H_

#include "SampleSet.h"
#include "FilterParams.h"
#include "FilterState.h"

namespace ohmPf
{

  class Filter
  {
  public:
    Filter(FilterParams_t params);
    virtual ~Filter();
    std::vector<Sample_t>* getSamples();
    void setSamples(std::vector<Sample_t> samples);
    unsigned int getSamplesMin();
    unsigned int getSamplesMax();
    SampleSet* getSampleSet();
    FilterState_t* getFilterState();
  private:
    SampleSet _sampleSet;
    unsigned int _samplesMin;
    unsigned int _samplesMax;
    FilterState_t _filterState;
  };

} /* namespace ohmPf */

#endif /* SRC_FILTER_H_ */
