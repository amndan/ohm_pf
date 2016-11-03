/*
 * ROSFilterOutput.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_ROSFILTEROUTPUT_H_
#define SRC_ROSFILTEROUTPUT_H_

#include "IFilterOutput.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float32.h"
#include "Sample.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include "Eigen/Dense"
#include "OhmPfNodeParams.h"
#include "FilterState.h"

namespace ohmPf
{

/**
 * @brief A measurement container for ROS ceil cam measurements.
 * It implements IFilterOutput to provide the user of the filter
 * a generalized interface for getting access to the filters output.
 * E.g. to vizualize the output.
 */
class ROSFilterOutput : public IFilterOutput
{
public:
  /**
   * @brief Constructor
   * @param paramSet ROS node param set for initialization.
   * @todo For calculating the right filter output we need
   * the actual filter time --> requires timing
   */
  ROSFilterOutput(OhmPfNodeParams_t paramSet);

  /**
   * @brief Destructor (empty)
   */
  virtual ~ROSFilterOutput(){};

  /**
   * @brief Calculate the output tfs for ROS and publish them.
   * @param pose Filter output pose in map frame.
   * @param auctual filter timestamp. Odom Message pushes time ahead.
   */
  void onOutputPoseChanged(Eigen::Vector3d pose, evo::Time stamp);

  /**
   * @brief Create and publish a PoseArray for displaying particle cloud in rviz.
   * @param samples Particle cloud.
   */
  void onSampleSetChanged(const std::vector<Sample_t>& samples); // TODO: saples must be const here

  /**
   * @brief Actualize the filter state if this method gets called.
   * @param state filter state struct.
   */
  void onFilterStateChanged(FilterState_t state);

  void publishMapOdom();

private:
  ros::Publisher _pubPoseArray;
  ros::Publisher _pubPose;
  ros::Publisher _pubProbPose;
  ros::Publisher _pubStabWeights;
  ros::Publisher _pubAdaptiveMeanQuotient;
  OhmPfNodeParams_t _paramSet;
  tf::TransformBroadcaster _tfBroadcaster;
  tf::TransformListener _tfListener;
  tf::Transform _map_odom;
  int _skipParticleForGui;
};

} /* namespace ohmPf */

#endif /* SRC_ROSFILTEROUTPUT_H_ */
