/*
 * AdaptiveMean.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: amndan
 */

#include "AdaptiveMean.h"

namespace ohmPf
{

AdaptiveMean::AdaptiveMean(double aFast, double aSlow)
{
  _initialized = false;
  _aFast = aFast;
  _aSlow = aSlow;
  _fastMean = 0;
  _slowMean = 0;

}

double AdaptiveMean::getQuotient()
{
  if(!_initialized)
  {
    return -1.0;
  }
  else
  {
    if(_slowMean == 0.0)
    {
      return std::numeric_limits<double>::max();
    }
    else
    {
      return std::max(0.0, 1 - _fastMean / _slowMean);
    }
  }
}

void AdaptiveMean::addValue(double value)
{
  if(!_initialized)
  {
    _slowMean = 0;// = value;
    _fastMean = 0;// = value;
    _initialized = true;
  }
  else
  {
    _slowMean = _slowMean + _aSlow * (value - _slowMean);
    _fastMean = _fastMean + _aFast * (value - _fastMean);

   // std::cout << "fast: " << _fastMean << " slow: " << _slowMean<< " mean: " << value<<
   //     " qout: " << _fastMean / _slowMean << " ret: " << std::max(0.0, 1 - _fastMean / _slowMean) <<  std::endl;
  }
}

} /* namespace ohmPf */
