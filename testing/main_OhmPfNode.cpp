
#include "ros/ros.h"
#include "OhmPfNode.h"
#include "SampleSet.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ohmPf");

  ohmPf::OhmPfNode* ohmPfNode = new ohmPf::OhmPfNode();



  while(1)
  {

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

    ohmPfNode->printSampleSet(samples);
    ohmPfNode->spinOnce();
  }
}
