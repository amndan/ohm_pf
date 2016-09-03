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
#include "IOdomQuantifier.h"
#include "IOdomMeasurement.h"
#include "OCSObserver.h"

namespace ohmPf
{

  class OdomUpdater : public FilterUpdater, public OCSClient
  {
  public:
    OdomUpdater(Filter* filter, IOdomQuantifier* quantifier, IOdomMeasurement* measurement, OCSObserver* ocsObserver);
    virtual ~OdomUpdater();
    void update();
  private:
    OCSObserver* _ocsObserver;
    IOdomQuantifier* _quantifier;
    IOdomMeasurement* _measurement;
  };

} /* namespace ohmPf */

#endif /* SRC_ODOMUPDATER_H_ */
