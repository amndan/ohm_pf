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

  class GaussianPdf
  {
  public:
    GaussianPdf();
    virtual ~GaussianPdf();

    Sample_t static getRandomSample(Eigen::Vector3d mu, double sigmaT, double sigmaPhi);
    double static getRandomValue(double mu, double sigma);
    void static initializeSeed();

  private:
    static bool _initialized;
  };

} /* namespace ohmPf */

#endif /* SRC_GAUSSIANPDF_H_ */
