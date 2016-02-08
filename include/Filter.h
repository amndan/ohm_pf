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
#include "Map.h"
#include "ros/time.h" // ros time can be used without ros environment -- compile with "-lboost_system"
#include "ros/duration.h"

namespace ohmPf
{

  class Filter
  {
  public:
    Filter(FilterParams_t paramSet);
    virtual ~Filter();
    SampleSet* getSampleSet();
    void initWithPose(const Eigen::Vector3d& pose);
    void initWithMap(Map* map); // todo: init filter with map
    void updateWithMap();
    bool isInitialized();

  private:
    SampleSet* _sampleSet;
    Map* _map;
    FilterParams_t _paramSet;
    bool _initialized;
  };

} /* namespace ohmPf */

#endif /* SRC_FILTER_H_ */
