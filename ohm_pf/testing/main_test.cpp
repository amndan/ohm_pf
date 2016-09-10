/*
 * testPose.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#include <iostream>

#include "Sample.h"
#include "SampleSet.h"

using namespace std;

int main(int argc, char** argv)
{
  ohmPf::Sample_t particle;
  particle.pose(0) = 12;
  particle.pose(1) = 13;
  particle.pose(2) = 14;

  particle.weight = particle.pose(2) + 2;

  cout << "***" << particle.pose << endl;
  cout << "***" << particle.weight << endl;
  cout << "*******" << endl;

  // generate cloud
  std::vector<ohmPf::Sample_t> samples;
  for(unsigned int i = 0; i < 100; i++){
    ohmPf::Sample_t newSample;
    newSample.pose(0) = (double) std::rand() / (double) RAND_MAX;
    newSample.pose(1) = (double) std::rand() / (double) RAND_MAX;
    newSample.pose(2) = (double) std::rand() / (double) RAND_MAX;
    newSample.weight = (double) std::rand() / (double) RAND_MAX;
    samples.push_back(newSample);
  }

  // assign cloud
  ohmPf::SampleSet* sampleSet = new ohmPf::SampleSet(samples);

  // test set
  cout << "count samples is: " << sampleSet->getCountSamples() << endl;
  cout << "is normalized: " << sampleSet->isNormalized() << endl;
  sampleSet->normalize();
  cout << "is normalized: " << sampleSet->isNormalized() << endl;

  samples = *(sampleSet->getSamples());

  delete sampleSet;

  sampleSet = new ohmPf::SampleSet(samples);

  // test set
  cout << "count samples is: " << sampleSet->getCountSamples() << endl;
  cout << "is normalized: " << sampleSet->isNormalized() << endl;
  sampleSet->normalize();
  cout << "is normalized: " << sampleSet->isNormalized() << endl;

  sampleSet->setSamples(samples);

  // test set
  cout << "count samples is: " << sampleSet->getCountSamples() << endl;
  cout << "is normalized: " << sampleSet->isNormalized() << endl;
  sampleSet->normalize();
  cout << "is normalized: " << sampleSet->isNormalized() << endl;

  return 0;
}

