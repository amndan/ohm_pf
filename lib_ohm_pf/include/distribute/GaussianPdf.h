/*
 * GaussianPdf.h
 *
 *  Created on: 08.01.2016
 *      Author: amndan
 */

#ifndef SRC_GAUSSIANPDF_H_
#define SRC_GAUSSIANPDF_H_

#include <ctime>
#include <assert.h>
#include "Eigen/Dense"
#include "Sample.h"
#include <iostream>

namespace ohmPf
{

/**
 * @brief A helper class for working with gaussian probability density functions.
 */
class GaussianPdf
{
public:
  /**
   * Constructor (empty)
   */
  GaussianPdf(){};

  /**
   * Destructor (empty)
   */
  virtual ~GaussianPdf(){};

  /**
   * @brief initializes the random number generator at first call of getRandomValue().
   */
  void static initializeSeed();

  /**
   * @brief Sample from a gaussian pdf (1D).
   * @param mu Mean value of pdf.
   * @param sigma Standard deviation of pdf.
   * @return The sample as double value.
   */
  double static getRandomValue(double mu, double sigma);

  /**
   * @brief Get the PDFs value of a given sample.
   * @param mu Mean value of pdf.
   * @param sigma Standard deviation of pdf.
   * @param x Input sample.
   * @return PDFs value for the given sample as double.
   */
  double static getProbability(double mu, double sigma, double x);

  /**
   * @brief Sample from a gaussian pdf (3D).
   * @param mu Mean value of pdf for x, y and phi.
   * @param sigmaT Translational standard deviation of pdf.
   * @param sigmaPhi Rotational standard deviation of pdf.
   * @return The sample as Sample_t with weight of 1.
   */
  Sample_t static getRandomSample(Eigen::Vector3d mu, double sigmaT, double sigmaPhi);

private:
  /**
   * @brief Static flag for initializing seed just one times.
   */
  static bool _initialized;
};

} /* namespace ohmPf */

#endif /* SRC_GAUSSIANPDF_H_ */
