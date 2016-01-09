#include "ros/ros.h"
#include "OhmPfNode.h"
#include "SampleSet.h"
#include "GaussianPdf.h"
#include "Filter.h"
#include "Eigen/Dense"
#include "OdomDiffParams.h"
#include "OdomDiff.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ohmPf");

  ohmPf::OhmPfNode* ohmPfNode = new ohmPf::OhmPfNode();

  ohmPfNode->spin();
}
