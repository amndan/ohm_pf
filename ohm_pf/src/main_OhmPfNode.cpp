#include "ros/ros.h"

#include "OhmPfNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ohmPf");

  ohmPf::OhmPfNode* ohmPfNode = new ohmPf::OhmPfNode();

  ohmPfNode->spin();
}
