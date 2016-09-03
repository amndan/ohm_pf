/*
 * FilterOutputUpdater.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_FILTEROUTPUTUPDATER_H_
#define SRC_FILTEROUTPUTUPDATER_H_

#include "FilterUpdater.h"
#include "Filter.h"
#include "IFilterOutput.h"
#include "Eigen/Dense"
#include "assert.h"
#include "UtilitiesOhmPf.h"


namespace ohmPf
{

/**
 * @brief Updates the filters output
 */
class FilterOutputUpdater : public FilterUpdater
{
public:
  /**
   * @brief Constructor
   * @param filterOutput Pointer to an IFilterOutput - E.g. ROSFilterOutput.
   * The user must connect his IFilterOutput with FilterController::setFilterOutput()
   * function to the filter.
   * @todo The filter has no Main loop or spin function for
   * updating its output variables. If the user calls FilterController::updateOutput()
   * The filters variables got updated and the FilterOutput callbacks get called.
   * Perhaps there is a better design for doing that.
   * @param filter Pointer to the Filter.
   */
  FilterOutputUpdater(IFilterOutput* filterOutput, Filter* filter);

  /**
   * Destructor (empty)
   */
  virtual ~FilterOutputUpdater(){};

  /**
   * @brief Updates The filter output.
   * In particular it calls the callbacks from IFilterOutput to let
   * the user visualize the filters output. Furthermore it actualizes
   * the filters pose with updateTf().
   */
  void update();

private:

  /**
   * @brief Updates the filters Pose
   * It creates a mean value of all Samples.
   * @todo Implement a better update function here with
   * e.g. Clustering of Samples
   * @return Returns the actual assumption of the robots pose.
   */
  Eigen::Vector3d updateTf();

  IFilterOutput* _filterOutput;
};

} /* namespace ohmPf */

#endif /* SRC_FILTEROUTPUTUPDATER_H_ */
