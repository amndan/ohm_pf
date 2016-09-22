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

/**
 * @brief Filter instance contains configuration values the
 * particles and variables for the actual state of the filter
 */
class Filter
{
public:
  /**
   * @brief Filter Constructor loads the initial filter params
   * @param params Initial filter params
   */
  Filter(FilterParams_t params);

  /**
   * Deconstructor (empty)
   */
  virtual ~Filter(){};

  /**
   * @brief function for accessing the filters samples.
   * @return pointer to the vector of the filters samples
   * used for updating and manipulating the filters samples
   */
  std::vector<Sample_t>* getSamples();

  /**
   * @brief function for setting initial samples for the filter
   * @param samples samples vector to set for the filter
   * @todo should we set the filters samples in the constructor?
   */
  void setSamples(std::vector<Sample_t> samples);

  /**
   * @brief getter for minimum ammount of samples
   * @return minimum ammount of samples
   */
  unsigned int getSamplesMin();

  /**
   * @brief getter for maximum ammount of samples
   * @return maximum ammount of samples
   */
  unsigned int getSamplesMax();

  /**
   * @brief function for accessing the sample set of the filter
   * @return pointer to the sample set of the filter
   * @todo functions for sample set and samples are redundant!
   */
  SampleSet* getSampleSet();

  /**
   * @brief get a pointer of the filters state struct to modify it
   * @return pointer to the filters state struct
   * @todo Would it be better to let the filter hold his updaters
   * in a way that they can access the filters sampleset without a method?
   */
  FilterState_t* getFilterState();

  /**
   * @brief returns a copy of filter params
   * @return copy of filter params
   */
  FilterParams_t getParams();

  /**
   * @param stamp setter for timestamp of filter. It is used for pose output of filter.
   */
  void setStamp(ros::Time stamp);

  /**
   * @return getter for timestamp of filter. It is used for pose output of filter.
   */
  ros::Time getStamp();

private:
  FilterParams_t _params;
  SampleSet _sampleSet;
  FilterState_t _filterState;
  ros::Time _stamp;
};

} /* namespace ohmPf */

#endif /* SRC_FILTER_H_ */
