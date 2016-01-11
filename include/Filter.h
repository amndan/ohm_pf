/*
 * Filter.h
 *
 *  Created on: 08.01.2016
 *      Author: amndan
 */

#ifndef SRC_FILTER_H_
#define SRC_FILTER_H_

#include "SampleSet.h"
#include "FilterParams.h"
#include "Eigen/Dense"
#include "Sample.h"
#include "GaussianPdf.h"
#include <assert.h>

namespace ohmPf
{

  class Filter
  {
  public:
    Filter(FilterParams_t paramSet);
    virtual ~Filter();
    SampleSet* getSampleSet();
    void initWithPose(const Eigen::Vector3d& pose);
    void initWithMap(); // todo: init filter with map

  private:
    SampleSet* _sampleSet;
    FilterParams_t _paramSet;
    bool _initialized;
  };

} /* namespace ohmPf */

#endif /* SRC_FILTER_H_ */
