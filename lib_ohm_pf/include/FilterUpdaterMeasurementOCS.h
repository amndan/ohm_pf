/*
 * FilterUpdaterMeasurementOCS.h
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENTOCS_H_
#define LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENTOCS_H_

#include "FilterUpdaterMeasurement.h"
#include "OCSClient.h"

namespace ohmPf
{

class FilterUpdaterMeasurementOCS : public FilterUpdaterMeasurement, public OCSClient
{
public:
  FilterUpdaterMeasurementOCS(IMeasurement* measurement, Filter* filter);
  virtual ~FilterUpdaterMeasurementOCS(){};

  virtual void tryToUpdate();

protected:
  void activateOCS();
  void deactivateOCS();

private:
  bool _OCSactive;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENTOCS_H_ */
