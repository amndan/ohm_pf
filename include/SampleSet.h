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

namespace ohmPf
{

class SampleSet
{
public:
  SampleSet(std::vector<Sample_t> samples);
  virtual ~SampleSet();

  std::vector<Sample_t>* getSamples();
  void setSamples(std::vector<Sample_t> samples);

  int getCountSamples() const;
  void normalize();
  bool isNormalized() const;
  void boostWeights();
private:
  std::vector<Sample_t> _samples;
  int _countSamples;
  bool _normalized;
};

} /* namespace ohmPf */

#endif /* SRC_SAMPLESET_H_ */
