/*
 * MapUpdater.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_MAPUPDATER_H_
#define SRC_MAPUPDATER_H_

#include "FilterUpdater.h"
#include "Filter.h"
#include "IMap.h"
#include "UtilitiesOhmPf.h"
#include "Eigen/Dense"
#include "IUpdateFilterMap.h"

namespace ohmPf
{

  class MapUpdater : public FilterUpdater, public IUpdateFilterMap
  {
  public:
    MapUpdater(Filter* filter, IMap* map);
    virtual ~MapUpdater();
    void initFilter();
    void update();
    void updateForce();
  private:
    IMap* _map;
    void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax);

  };

} /* namespace ohmPf */

#endif /* SRC_MAPUPDATER_H_ */
