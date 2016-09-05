/*
 * SampleSet.h
 *
 *  Created on: Jan 5, 2016
 *      Author: amndan
 */

#ifndef SRC_SAMPLESET_H_
#define SRC_SAMPLESET_H_

#include "Sample.h"
#include <vector>
#include <iostream>
#include <numeric>
#include <ctime>
#include "UtilitiesOhmPf.h"
#include "assert.h"

namespace ohmPf
{

/**
 * @brief A class for representing a set of samples (of particles)
 */
class SampleSet // TODO: eventually we dont need this class; we access samples through the filter class
{
public:

  /**
   * @brief Constructor initializes the samples in the sampleset.
   * @param numSamples Initialize numSamples samples in sampleSet.
   */
  SampleSet(unsigned int numSamples);
  virtual ~SampleSet();

  /**
   * @return returns a pointer to the samples.
   */
  std::vector<Sample_t>* getSamples();

  /**
   * @param samples Sets the input sample set as sample set (a copy of input)
   */
  void setSamples(std::vector<Sample_t> samples);

  /**
   * @brief a function for "boosting" the weights of the sample set.
   * Lower weights get higher influence according to y = sqrt(x)
   */
  void boostWeights();

  /**
   * @brief Normalize weights so that sum(sampleSet.weight) = 1.0.
   */
  void normalize();

  /**
   * @return True if set is normalized.
   * @todo Normalization should be up to the users of the sampleset.
   */
  bool isNormalized();

  /**
   * @return Returns the number of samples in the sample set.
   */
  int getCountSamples() const;

private:
  std::vector<Sample_t> _samples;
  int _countSamples;
  bool _normalized;
};

} /* namespace ohmPf */

#endif /* SRC_SAMPLESET_H_ */
