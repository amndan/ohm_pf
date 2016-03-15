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

class SampleSet // TODO: eventually we dont need this class; we access samples through the filter class
{
public:
  SampleSet(unsigned int numSamples);
  virtual ~SampleSet();

  std::vector<Sample_t>* getSamples();
  void setSamples(std::vector<Sample_t> samples);

  void boostWeights();
  void normalize();

  bool isNormalized();

  int getCountSamples() const;

private:
  std::vector<Sample_t> _samples;
  int _countSamples;
  bool _normalized;
};

} /* namespace ohmPf */

#endif /* SRC_SAMPLESET_H_ */
