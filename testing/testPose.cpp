/*
 * testPose.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#include <iostream>
#include "../include/Pose.h"
#include "../include/Particle.h"

using namespace std;

int main(int argc, char** argv)
{
  ohmPf::Particle_t particle;
  particle.pose.vector(0) = 12;
  particle.pose.vector(1) = 13;
  particle.pose.vector(2) = 14;

  particle.weight = particle.pose.vector(2) + 2;

  cout << "***" << particle.pose.vector << endl;
  cout << "***" << particle.weight << endl;

  return 0;
}

