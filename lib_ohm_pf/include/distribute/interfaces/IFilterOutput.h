/*
 * IFilterOutput.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_IFILTEROUTPUT_H_
#define SRC_IFILTEROUTPUT_H_

#include "Sample.h"
#include <vector>
#include "Eigen/Dense"
#include "FilterState.h"
#include <evocortex/core/time/Time.h>

namespace ohmPf
{

/**
 * @brief An abstract class for providing the user a generalized
 * Interface for accessing the filters output values. E.g. for
 * incorporating the map --> base_footprint tf into his application.
 */
class IFilterOutput
{
public:
  IFilterOutput(){};
  virtual ~IFilterOutput(){};

  /**
   * @brief Gets called when the filters output pose has changed.
   * @bug For now ALL this functions get called when the user requests
   * a filter update via filter updater. These functions should be called
   * automatically with a defined rate. Perhaps Filter updater could
   * be a ocs client too?
   * @param pose The filter output pose: x y yaw
   */
  virtual void onOutputPoseChanged(Eigen::Vector3d pose, evo::Time stamp) = 0;

  /**
   * @brief Gets called if the filters state struct has changed.
   * @param state Filter state struct.
   */
  virtual void onFilterStateChanged(FilterState_t state) = 0;

  /**
   * @brief Gets called if the sample set of the filter has changed.
   * @param samples Sample set of the filter.
   */
  virtual void onSampleSetChanged(const std::vector<Sample_t>& samples) = 0;
};

} /* namespace ohmPf */

#endif /* SRC_IFILTEROUTPUT_H_ */
