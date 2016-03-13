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
#include "geometry_msgs/PoseArray.h"
#include "Sample.h"
#include "tf/transform_datatypes.h"
#include <vector>
#include <string>

namespace ohmPf
{

  class ROSFilterOutput : public IFilterOutput
  {
  public:
    ROSFilterOutput(std::string fixedFrame);
    virtual ~ROSFilterOutput();
    void actualizeTF();
    void printSampleSet(std::vector<Sample_t>& samples); // TODO: saples must be const here
  private:
    ros::Publisher _pubPoseArray;
    std::string _fixedFrame;
  };

} /* namespace ohmPf */

#endif /* SRC_ROSFILTEROUTPUT_H_ */
