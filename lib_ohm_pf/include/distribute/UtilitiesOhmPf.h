/*
 * UtilitiesOhmPf.h
 *
 *  Created on: Jan 9, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_UTILITIESOHMPF_H_
#define INCLUDE_UTILITIESOHMPF_H_

#include <cmath>
#include "GaussianPdf.h"
#include "assert.h"
#include "Eigen/Dense"
#include <numeric>
#include <time.h>
#include "Sample.h"
#include <vector>

namespace ohmPf
{

/*
 * PROTOTYPES
 */

/**
 * @return returns the mean angle of a samples vector in range of -pi to pi.
 */
double getMeanOfAngles(const std::vector<Sample_t>& samples);

/**
 * @param corrects the input angle to range of -pi to pi
 */
void correctAngleOverflow(double& angle);

/**
 * @param measurement First Pose.
 * @param sample Second Pose.
 * @param sigmaPos Translational standard deviation.
 * @param sigmaPhi Rotational standard deviation.
 * @return An indicator for the equality of the to poses.
 */
double getProbabilityFrom2Poses(
    const Eigen::Vector3d& measurement,
    const Eigen::Vector3d& sample,
    double sigmaPos = 0.5,
    double sigmaPhi = 0.5 * M_PI);

/**
 * @brief Adds gaussian noise to a sample.
 * @param sample The sample noise should be added.
 * @param sigmaPos Translational standard deviation.
 * @param sigmaPhi Rotational stadard deviation.
 * @bug We are sampling from a square - not from a circle!
 */
void addGaussianRandomness(Sample_t& sample, double sigmaPos = 0.05, double sigmaPhi = 10 / 180 * M_PI);

/**
 * @brief Adds uniform noise to a sample.
 * @param sample The sample noise should be added.
 * @param sigmaPos Translational standard deviation.
 * @param sigmaPhi Rotational stadard deviation.
 * @bug This was the attempt to sample in circular form
 * but is is not uniform!?
 */
void addUniformRandomness(Sample_t& sample, double sigmaPos, double sigmaPhi);

/**
 * @return returns the standard deviation from a double vector.
 */
double getStabw(const std::vector<double>& v);

/**
 * @return returns the standard deviation of a samples vector.
 */
double getStabwOfSamples(const std::vector<Sample_t>& samples);

/**
 * @brief quality of samples is a value between 0 and 1.0.
 * 1.0 means the standard deviation is very low and vice versa
 */
double getQualityOfSamples(const std::vector<Sample_t>& samples);

/**
 * @brief create a 3x3 eigen transformation matrix
 */
void create3x3TransformationMatrix(double& x, double& y, double& theta, Eigen::Matrix3d& tf);

}  // ohmPf

#endif /* INCLUDE_UTILITIESOHMPF_H_ */
