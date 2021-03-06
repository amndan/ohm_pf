/*
 * OhmPfNodeParams.h
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_OHMPFNODEPARAMS_H_
#define INCLUDE_OHMPFNODEPARAMS_H_

#include "Eigen/Dense"

namespace ohmPf
{

/**
 * @brief Structure to hold all parameters at one place.
 */
typedef struct
{
  std::string tfFixedFrame;
  std::string tfBaseFootprintFrame;
  std::string tfOutputFrame;
  std::string tfOdomFrame;
  std::string topOdometry;
  std::string top2dPoseEst;
  std::string topClickedPoint;
  std::string topCeilCam;
  std::string topMap;
  std::string topMapSrv;
  std::vector<std::string> topScans;
  std::string topParticleCloud;
  std::string topProbPose;
  std::string initMode;
  Eigen::Vector3d initPose;
  double initSigmaTrans;
  double initSigmaRot;
  double uncertaintyLaser;
  unsigned int subsamplingLaser;
  int skipParticleForGui;


} OhmPfNodeParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_OHMPFNODEPARAMS_H_ */

