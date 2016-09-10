#include "ros/ros.h"

#include "../include/OdomUpdater.h"
#include "OhmPfNode.h"
#include "SampleSet.h"
#include "GaussianPdf.h"
#include "Filter.h"
#include "Eigen/Dense"
#include "OdomDiffParams.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ohmPf");

  ohmPf::OhmPfNode* ohmPfNode = new ohmPf::OhmPfNode();

  ohmPf::FilterParams_t filterParams;
  filterParams.samplesMax = 500;
  filterParams.samplesMin = 5;
  ohmPf::Filter* filter = new ohmPf::Filter(filterParams);

  Eigen::Vector3d initPose;
  initPose(0) = 1;
  initPose(1) = 2;
  initPose(2) = 0;

  filter->initWithPose(initPose);
  ohmPfNode->printSampleSet(filter->getSampleSet());

  ohmPf::OdomDiffParams_t odomParams;
  odomParams.a1 = 0.1;
  odomParams.a2 = 0;
  odomParams.a3 = 0.1;
  odomParams.a4 = 0;

  ohmPf::OdomUpdater* odom = new ohmPf::OdomUpdater(odomParams);

  Eigen::Vector3d odom0;
  odom0(0) = 0;
  odom0(1) = 0;
  odom0(2) = 0;
  Eigen::Vector3d odom1;
  odom1(0) = 0.5;
  odom1(1) = 0.3;
  odom1(2) = 0.1;

  odom->setMeasurement(odom0, odom1);

  while(1)
  {
    odom->updateFilter(filter);
    ohmPfNode->printSampleSet(filter->getSampleSet());
    ohmPfNode->spinOnce();
  }
}
