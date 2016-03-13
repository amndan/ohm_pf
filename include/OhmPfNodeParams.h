/*
 * OhmPfNodeParams.h
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_OHMPFNODEPARAMS_H_
#define INCLUDE_OHMPFNODEPARAMS_H_

namespace ohmPf
{

typedef struct
{
  std::string tfFixedFrame;
  std::string tfBaseFootprintFrame;
  std::string topOdometry;
  std::string top2dPoseEst;
  std::string topCeilCam;
  std::string topMap;
  std::string topMapSrv;
  std::string topScan;
} OhmPfNodeParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_OHMPFNODEPARAMS_H_ */
