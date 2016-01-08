#include "ros/ros.h"
#include "OhmPfNode.h"
#include "SampleSet.h"
#include "GaussianPdf.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ohmPf");

  ohmPf::OhmPfNode* ohmPfNode = new ohmPf::OhmPfNode();

  Eigen::Vector3d mu;
  mu(0) = 2;
  mu(1) = 3;
  mu(2) = 1;


  while(1)
  {
    // generate cloud

    std::vector<ohmPf::Sample_t> samples;
    for(unsigned int i = 0; i < 500; i++)
    {
      samples.push_back(ohmPf::GaussianPdf::getRandomSample(mu, 0.5, 0.1));
    }

    ohmPf::SampleSet* sampleSet = new ohmPf::SampleSet(samples);

    ohmPfNode->printSampleSet(sampleSet);
    ohmPfNode->spinOnce();
  }
}
