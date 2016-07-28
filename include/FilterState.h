/*
 * FilterState.h
 *
 *  Created on: Apr 26, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_FilterState_t_H_
#define INCLUDE_FilterState_t_H_

namespace ohmPf
{

typedef struct
{
  double probPose;
  double varWeights;
  //double countSamples;
} FilterState_t;

} /* namespace ohmPf */

#endif /* INCLUDE_FilterState_t_H_ */
