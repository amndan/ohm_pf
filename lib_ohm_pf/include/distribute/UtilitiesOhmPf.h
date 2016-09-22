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
#include "ros/time.h"

namespace ohmPf
{

/*
 * PROTOTYPES
 */

/**
 * @return returns the mean angle of a samples vector in range of -pi to pi.
 */
static double getMeanOfAngles(const std::vector<Sample_t>& samples);

/**
 * @param corrects the input angle to range of -pi to pi
 */
static void correctAngleOverflow(double& angle);

/**
 * @param measurement First Pose.
 * @param sample Second Pose.
 * @param sigmaPos Translational standard deviation.
 * @param sigmaPhi Rotational standard deviation.
 * @return An indicator for the equality of the to poses.
 */
static double getProbabilityFrom2Poses(
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
static void addGaussianRandomness(Sample_t& sample, double sigmaPos = 0.05, double sigmaPhi = 10 / 180 * M_PI);

/**
 * @brief Adds uniform noise to a sample.
 * @param sample The sample noise should be added.
 * @param sigmaPos Translational standard deviation.
 * @param sigmaPhi Rotational stadard deviation.
 * @bug This was the attempt to sample in circular form
 * but is is not uniform!?
 */
static void addUniformRandomness(Sample_t& sample, double sigmaPos, double sigmaPhi);

/**
 * @return returns the standard deviation from a double vector.
 */
static double getStabw(const std::vector<double>& v);

/**
 * @return returns the standard deviation of a samples vector.
 */
static double getStabwOfSamples(const std::vector<Sample_t>& samples);

/**
 * @brief quality of samples is a value between 0 and 1.0.
 * 1.0 means the standard deviation is very low and vice versa
 */
static double getQualityOfSamples(const std::vector<Sample_t>& samples);

/**
 * @brief create a 3x3 eigen transformation matrix
 */
static void create3x3TransformationMatrix(double& x, double& y, double& theta, Eigen::Matrix3d& tf);

/*
 * IMPLEMENTATIONS
 */

double getMeanOfAngles(const std::vector<Sample_t>& samples)
{
  // TODO: assert samples are normalized

  //https://en.wikipedia.org/wiki/Mean_of_circular_quantities
  //mean = atan2(sum(sin(alpha)), sum(cos(alpha)))

  double sumSin = 0;
  double sumCos = 0;

  for(unsigned int i = 0; i < samples.size(); i++)
  {
    //E(X) = int( f(x)*dx )
    sumSin += std::sin(samples.at(i).pose(2)) * samples.at(i).weight;
    sumCos += std::cos(samples.at(i).pose(2)) * samples.at(i).weight;
  }

  return std::atan2(sumSin, sumCos);
}

void correctAngleOverflow(double& angle)
{
  angle = std::fmod(angle, 2 * M_PI);

  if(angle > M_PI)
    angle -= 2 * M_PI;
  else if(angle < -M_PI)
    angle += 2 * M_PI;
}

double getProbabilityFrom2Poses(const Eigen::Vector3d& measurement, const Eigen::Vector3d& sample, double sigmaPos, double sigmaPhi)
{
  double px = GaussianPdf::getProbability(measurement(0), sigmaPos, sample(0));
  double py = GaussianPdf::getProbability(measurement(1), sigmaPos, sample(1));
  double pPhi = GaussianPdf::getProbability(measurement(2), sigmaPhi, sample(2));  //TODO: need angle overflow here?

  return px * py * pPhi;
}

//  static double getProbabilityFrom2Poses(const Eigen::Vector3d& measurement, const Eigen::Vector3d& sample, double sigmaPos = 0.5, double sigmaPhi = 0.5
//      * M_PI, double weightPhi = 0.5)
//  {
//    assert(weightPhi >= 0.0 && weightPhi <= 1.0);
//
//    double distPos = std::sqrt(std::pow(measurement(0) - sample(0), 2) + std::pow(measurement(1) - sample(1), 2));
//
//    double pPhi = GaussianPdf::getProbability(measurement(2), sigmaPhi, sample(2));
//    double pPos = GaussianPdf::getProbability(0.0, sigmaPos, distPos);
//
//    return (1 - weightPhi) * pPos + weightPhi * pPhi;
//  }

void addGaussianRandomness(Sample_t& sample, double sigmaPos, double sigmaPhi)
{
  sample.pose(0) -= GaussianPdf::getRandomValue(0.0, sigmaPos);
  sample.pose(1) -= GaussianPdf::getRandomValue(0.0, sigmaPos);
  sample.pose(2) -= GaussianPdf::getRandomValue(0.0, sigmaPhi);
  correctAngleOverflow(sample.pose(2));
}

void addUniformRandomness(Sample_t& sample, double sigmaPos, double sigmaPhi)
{
  sigmaPhi = std::abs(sigmaPhi);
  sigmaPos = std::abs(sigmaPos);

  double radius = drand48() * sigmaPos;
  double angle = drand48() * 2 * M_PI;

  sample.pose(0) += std::cos(angle) * radius;
  sample.pose(1) += std::sin(angle) * radius;

  sample.pose(2) -= drand48() * sigmaPhi - sigmaPhi / 2.0;
  correctAngleOverflow(sample.pose(2));
}

double getStabw(const std::vector<double>& v)
{
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();
  double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);

  return std::sqrt(sq_sum / v.size() - mean * mean);
}

double getStabwOfSamples(const std::vector<Sample_t>& samples)
{
  std::vector<double> x;
  std::vector<double> y;
  double stabwX;
  double stabwY;

  for(unsigned int i = 0; i < samples.size(); i++)
  {
    x.push_back(samples[i].pose(0));
    y.push_back(samples[i].pose(1));
  }

  stabwX = getStabw(x);
  stabwY = getStabw(y);

  return std::sqrt( pow(stabwX, 2) + pow(stabwY, 2) );
}

double getQualityOfSamples(const std::vector<Sample_t>& samples)
{
  double stabw = getStabwOfSamples(samples);
  return std::pow( M_E, -stabw); // e^-x --> f(0) = 1; f(inf) = 0
}

void create3x3TransformationMatrix(double& x, double& y, double& theta, Eigen::Matrix3d& tf)
{
  tf(0, 0) = std::cos(theta);
  tf(0, 1) = -std::sin(theta);
  tf(0, 2) = x;
  tf(1, 0) = -tf(0, 1);
  tf(1, 1) = tf(0, 0);
  tf(1, 2) = y;
  tf(2, 0) = 0.0;
  tf(2, 1) = 0.0;
  tf(2, 2) = 1.0;
}


}  // ohmPf

#endif /* INCLUDE_UTILITIESOHMPF_H_ */
