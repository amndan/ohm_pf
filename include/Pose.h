/*
 * Pose.h
 *
 *  Created on: Jan 4, 2016
 *      Author: amndan
 */

#ifndef SRC_POSE_H_
#define SRC_POSE_H_

#include "Eigen/Dense"

namespace ohmPf
{

typedef struct
{
  Eigen::Vector3d vector;
} Pose_t;

} /* namespace ohmPf */

#endif /* SRC_POSE_H_ */
