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
    Filter(std::vector<Sample_t> samples);
    virtual ~Filter();
    std::vector<Sample_t>* getSamples();
    void setSamples(std::vector<Sample_t> samples);
  private:
    SampleSet _sampleSet;
  };

} /* namespace ohmPf */

#endif /* SRC_FILTER_H_ */
