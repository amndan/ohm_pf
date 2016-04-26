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

  class ROSFilterOutput : public IFilterOutput
  {
  public:
    ROSFilterOutput(OhmPfNodeParams_t paramSet);
    virtual ~ROSFilterOutput();
    void actualizeTF(Eigen::Vector3d pose);
    void printSampleSet(std::vector<Sample_t>& samples); // TODO: saples must be const here
    void actualizeState(FilterState_t state);
  private:
    ros::Publisher _pubPoseArray;
    ros::Publisher _pubProbPose;
    OhmPfNodeParams_t _paramSet;
    tf::TransformBroadcaster _tfBroadcaster;
    tf::TransformListener _tfListener;
  };

} /* namespace ohmPf */

#endif /* SRC_ROSFILTEROUTPUT_H_ */
