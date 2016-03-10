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
#include "ILaserQuanifier.h"


namespace ohmPf
{

  class LaserUpdater : public FilterUpdater
  {
  public:
    LaserUpdater();
    virtual ~LaserUpdater();
    void update();
    void initFilter();

  private:
    IMap* _map;
    ILaserMeasurement* _measurement;
    ILaserQuantifier _quantifier;
  };

} /* namespace ohmPf */

#endif /* INCLUDE_LASERUPDATER_H_ */
