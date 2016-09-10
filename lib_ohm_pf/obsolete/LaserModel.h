/*
 * LaserModel.h
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LASERMODEL_H_
#define INCLUDE_LASERMODEL_H_


namespace ohmPf
{
  class Filter; // forward decl

  class LaserModel
  {
  public:
    LaserModel(){};
    virtual ~LaserModel(){};
    
    virtual void updateFilter(Filter& filter) = 0;
  };

} /* namespace ohmPf */

#endif /* INCLUDE_LASERMODEL_H_ */
