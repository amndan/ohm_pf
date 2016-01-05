/*
 * Particlet.h
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_PARTICLE_H_
#define INCLUDE_PARTICLE_H_

#include "Pose.h"

namespace ohmPf
{

typedef struct
{
  Pose_t pose;
  double weight;
} Particle_t;

} /* namespace ohmPf */

#endif /* INCLUDE_PARTICLE_H_ */
