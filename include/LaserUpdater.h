/*
 * RosLaserPM.h
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LASERUPDATER_H_
#define INCLUDE_LASERUPDATER_H_

#include "FilterUpdater.h"
#include "IMap.h"
#include "ILaserMeasurement.h"
#include "ILaserQuantifier.h"
#include "IOCSClient.h"
#include "MapUpdater.h"


namespace ohmPf
{

  class LaserUpdater : public FilterUpdater, public IOCSClient
  {
  public:
    LaserUpdater(Filter* filter, IMap* map, ILaserMeasurement* measurement, ILaserQuantifier* quantifier, MapUpdater* updateFilterMap);
    virtual ~LaserUpdater();
    void update();
    void setOCSFlagTrue();

  private:
    IMap* _map;
    ILaserMeasurement* _measurement;
    ILaserQuantifier* _quantifier;
    MapUpdater* _updateFilterMap;
    bool _OCSFlag;
  };

} /* namespace ohmPf */

#endif /* INCLUDE_LASERUPDATER_H_ */
