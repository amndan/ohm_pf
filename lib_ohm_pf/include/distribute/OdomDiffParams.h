/*
 * OdomDiffParams.h
 *
 *  Created on: Jan 8, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_OdomDiffParams_t_H_
#define INCLUDE_OdomDiffParams_t_H_

namespace ohmPf
{

/**
 * @brief Param set for differential drive odometry model.
 * see book Probablistic Robotics by S. Thrun et al.
 */

typedef struct
{
  /**
   * @brief rot error from rot motion.
   */
  double a1;

  /**
   * @brief rot error from trans motion.
   */
  double a2;

  /**
   * @brief trans error from trans motion.
   */
  double a3;

  /**
   * @brief trans error from rot motion.
   */
  double a4;
} OdomDiffParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_OdomDiffParams_t_H_ */
