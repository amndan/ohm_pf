/*
 * OdomUpdater.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ODOMUPDATER_H_
#define SRC_ODOMUPDATER_H_

#include "FilterUpdater.h"
#include "IFilter.h"
#include "IOCSClient.h"
#include "IOdomQuantifier.h"
#include "IOdomMeasurement.h"
#include "IUpdateFilterMap.h"

namespace ohmPf
{

  class OdomUpdater : public FilterUpdater, public IOCSClient
  {
  public:
    OdomUpdater(IFilter* filter, IOdomQuantifier* quantifier, IOdomMeasurement* measurement, IUpdateFilterMap* updateFilterMap);
    virtual ~OdomUpdater();
    void update();
    void setOCSFlagTrue();
  private:
    IOdomQuantifier* _quantifier;
    IOdomMeasurement* _measurement;
    IUpdateFilterMap* _updateFilterMap;
    bool _OCSFlag;
  };

} /* namespace ohmPf */

#endif /* SRC_ODOMUPDATER_H_ */
