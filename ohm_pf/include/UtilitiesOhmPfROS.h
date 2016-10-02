/*
 * UtilitiesOhmPf.h
 *
 *  Created on: Jan 9, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_UTILITIESOHMPFROS_H_
#define INCLUDE_UTILITIESOHMPFROS_H_


#include "Eigen/Dense"
#include "tf/transform_datatypes.h"

namespace ohmPf
{

/**
 * @brief Converter function for eigen matrix and tf transform
 */
tf::Transform eigenMatrix3x3ToTf(const Eigen::Matrix3d& eigen);

/**
 * @brief Converter function for eigen matrix and tf transform
 */
Eigen::Matrix3d tfToEigenMatrix3x3(const tf::Transform& tf);

}  // ohmPf

#endif /* INCLUDE_UTILITIESOHMPFROS_H_ */
