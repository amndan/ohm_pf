/*
 * MapUpdater.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_MAPUPDATER_H_
#define SRC_MAPUPDATER_H_

#include "FilterUpdater.h"
#include "IFilter.h"
#include "IMap.h"

namespace ohmPf
{

  class MapUpdater : public FilterUpdater
  {
  public:
    MapUpdater(IFilter* filter, IMap* map);
    virtual ~MapUpdater();
    void update();
    void updateForce();
  private:
    IMap* _map;

  };

} /* namespace ohmPf */

#endif /* SRC_MAPUPDATER_H_ */
