/*
 * AdaptiveMean.h
 *
 *  Created on: Sep 24, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_ADAPTIVEMEAN_H_
#define LIB_OHM_PF_SRC_ADAPTIVEMEAN_H_

#include <limits>
#include <iostream>
#include <cmath>

namespace ohmPf
{

class AdaptiveMean
{
public:
  AdaptiveMean(double aFast, double aSlow);
  virtual ~AdaptiveMean(){};

  double getQuotient();

  void addValue(double mean);

private:
  double _fastMean;
  double _slowMean;
  double _aFast;
  double _aSlow;
  bool _initialized;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_ADAPTIVEMEAN_H_ */
