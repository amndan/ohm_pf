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
  std::string topFixedFrame;
  std::string topOdometry;
  std::string top2dPoseEst;
} OhmPfNodeParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_OHMPFNODEPARAMS_H_ */
