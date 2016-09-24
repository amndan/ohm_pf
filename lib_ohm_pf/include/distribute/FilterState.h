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

/**
 * @brief Structure for holding the filters Status.
 * @todo Use this structure for maintaining whole filter state.
 * E.g. variables like prob of pose and variance of weights.
 */
typedef struct
{
  double probPose;
  double stabWeights;
  double adaptiveMeanQuotient;
} FilterState_t;

} /* namespace ohmPf */

#endif /* INCLUDE_FilterState_t_H_ */
