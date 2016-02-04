/*
 * Map.h
 *
 *  Created on: 13.01.2016
 *      Author: amndan
 */

#ifndef SRC_MAP_H_
#define SRC_MAP_H_

namespace ohmPf
{
  class Map
  {
  public:
    Map(){}
    virtual ~Map(){}
    virtual bool isOccupied(double x, double y, bool isInMapOriginFrame = false) = 0; //x, y in m
    virtual double getWith() = 0;
    virtual double getHeigh() = 0;
  };
} /* namespace ohmPf */

#endif /* SRC_MAP_H_ */
