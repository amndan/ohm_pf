/*
 * OdomUpdater.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ODOMUPDATER_H_
#define SRC_ODOMUPDATER_H_

#include "FilterUpdater.h"
#include "Filter.h"
#include "OCSClient.h"
#include "IOdomMeasurement.h"
#include "OCSObserver.h"

namespace ohmPf
{

  class OdomUpdater : public FilterUpdater, public OCSClient
  {
  public:
    OdomUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver);
    virtual ~OdomUpdater();
    virtual void calculate() = 0;
    void update();
  protected:
    OCSObserver* _ocsObserver;
    IOdomMeasurement* _measurement;
  };

} /* namespace ohmPf */

#endif /* SRC_ODOMUPDATER_H_ */
