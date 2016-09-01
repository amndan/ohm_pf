/*
 * DefaultResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef INCLUDE_STDRESAMPLER_H_
#define INCLUDE_STDRESAMPLER_H_

#include "interfaces/IResampler.h"

namespace ohmPf
{

  class STDResampler : public IResampler
  {
  public:
    STDResampler(double addNoiseSigmaTrans, double addNoiseSigmaRot);
    virtual ~STDResampler();
    void resample(Filter* filter);
    void setOCSFlagTrue();
  private:
    bool _OCSFlag;
    double _addNoiseSigmaRot;
    double _addNoiseSigmaTrans;
  };

} /* namespace ohmPf */

#endif /* INCLUDE_STDRESAMPLER_H_ */
