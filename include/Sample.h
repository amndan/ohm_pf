/*
 * Particlet.h
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_SAMPLE_H_
#define INCLUDE_SAMPLE_H_

#include "Eigen/Dense"

namespace ohmPf
{

typedef struct
{
  Eigen::Vector3d pose;
  double weight;
} Sample_t;

} /* namespace ohmPf */

#endif /* INCLUDE_SAMPLE_H_ */
