/*
 * IUpdateFilterMap.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_IUPDATEFILTERMAP_H_
#define SRC_IUPDATEFILTERMAP_H_

namespace ohmPf
{

  class IUpdateFilterMap
  {
  public:
    IUpdateFilterMap(){};
    virtual ~IUpdateFilterMap(){};
    virtual void update() = 0;
    virtual void updateForce() = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_IUPDATEFILTERMAP_H_ */
