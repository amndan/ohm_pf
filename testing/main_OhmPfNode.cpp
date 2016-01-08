#include "ros/ros.h"
#include "OhmPfNode.h"
#include "SampleSet.h"
#include "GaussianPdf.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ohmPf");

  ohmPf::OhmPfNode* ohmPfNode = new ohmPf::OhmPfNode();

  ohmPf::Sample_t newSample;
  newSample.pose(0) = 2;
  newSample.pose(1) = 3;
  newSample.pose(2) = 1;
  newSample.weight = 1;

  ohmPf::GaussianPdf* sampler = new ohmPf::GaussianPdf();

  while(1)
  {
    // generate cloud
    std::vector<ohmPf::Sample_t> samples;
    for(unsigned int i = 0; i < 500; i++)
    {
      samples.push_back(sampler->getRandomSample(newSample, 0.5, 0.1));
    }

    ohmPfNode->printSampleSet(samples);
    ohmPfNode->spinOnce();
  }
}
