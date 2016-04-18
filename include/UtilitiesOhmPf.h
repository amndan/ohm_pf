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
#include "tf/transform_datatypes.h"
#include <time.h>
#include "ros/time.h"

namespace ohmPf
{

  static double getMeanOfAngles(const std::vector<Sample_t>& samples)
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

  static void correctAngleOverflow(double& angle)
  {
    angle = std::fmod(angle, 2 * M_PI);

    if(angle > M_PI)
      angle -= 2 * M_PI;
    else if(angle < -M_PI)
      angle += 2 * M_PI;
  }

  static double getProbabilityFrom2Poses(const Eigen::Vector3d& measurement, const Eigen::Vector3d& sample, double sigmaPos = 0.5, double sigmaPhi = 0.5
      * M_PI)
  {
    double px = GaussianPdf::getProbability(measurement(0), sigmaPos, sample(0));
    double py = GaussianPdf::getProbability(measurement(1), sigmaPos, sample(1));
    double pPhi = GaussianPdf::getProbability(measurement(2), sigmaPhi, sample(2)); //TODO: need angle overflow here?

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

  static void addGaussianRandomness(Sample_t& sample, double sigmaPos = 0.05, double sigmaPhi = 10 / 180 * M_PI)
  {
    sample.pose(0) -= GaussianPdf::getRandomValue(0.0, sigmaPos);
    sample.pose(1) -= GaussianPdf::getRandomValue(0.0, sigmaPos);
    sample.pose(2) -= GaussianPdf::getRandomValue(0.0, sigmaPhi);
    correctAngleOverflow(sample.pose(2));
  }

  static double getStabw(const std::vector<double>& v)
  {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);

    return std::sqrt(sq_sum / v.size() - mean * mean);
  }

  static tf::Transform eigenMatrix3x3ToTf(const Eigen::Matrix3d& eigen)
  {
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(eigen(0, 2), eigen(1, 2), 0.0));
    tf.setRotation(tf::createQuaternionFromYaw(asin(eigen(0, 1))));

    return tf;
  }

  static void create3x3TransformationMatrix(double& x, double& y, double& theta, Eigen::Matrix3d& tf)
  {
    tf(0, 0) = std::cos(theta) + 0.0;
    tf(0, 1) = -std::sin(theta) + 0.0;
    tf(0, 2) = x + 0.0;
    tf(1, 0) = std::sin(theta) + 0.0;
    tf(1, 1) = std::cos(theta) + 0.0;
    tf(1, 2) = y + 0.0;
    tf(2, 0) = 0.0;
    tf(2, 1) = 0.0;
    tf(2, 2) = 1.0;
  }

  static Eigen::Matrix3d tfToEigenMatrix3x3(const tf::Transform& tf)
  {
    Eigen::Matrix3d eigen(3, 3);
    eigen.setIdentity();

    double theta = tf::getYaw(tf.getRotation());
    double x = tf.getOrigin().getX();
    double y = tf.getOrigin().getY();

    // preigenlem with sin() returns -0.0 (avoid with +0.0)
    eigen(0, 0) = cos(theta) + 0.0;
    eigen(0, 1) = -sin(theta) + 0.0;
    eigen(0, 2) = x + 0.0;
    eigen(1, 0) = sin(theta) + 0.0;
    eigen(1, 1) = cos(theta) + 0.0;
    eigen(1, 2) = y + 0.0;

    return eigen;
  }
}  // ohmPf

#endif /* INCLUDE_UTILITIESOHMPF_H_ */
