/*
 * Measurement.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_MEASUREMENT_H_
#define SRC_MEASUREMENT_H_

namespace ohmPf
{

  class Measurement
  {
  public:
    Measurement();
    virtual ~Measurement();
    void getStamp(); // todo: timestamp as returnv
  private:
    int _stamp; // todo: time as stamp
  };

} /* namespace ohmPf */

#endif /* SRC_MEASUREMENT_H_ */
