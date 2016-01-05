/*
 * testPose.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#include <iostream>

#include "../include/Sample.h"

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

  return 0;
}

