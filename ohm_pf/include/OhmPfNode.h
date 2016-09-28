/*
 * OhmPfNode.h
 *
 *  Created on: Jan 5, 2016
 *      Author: amndan
 */

#ifndef SRC_OHMPFNODE_H_
#define SRC_OHMPFNODE_H_

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "assert.h"
#include "OhmPfNodeParams.h"
#include "Eigen/Dense"
#include "IFilterController.h"
#include "FilterParams.h"
#include "ROSOdomMeasurement.h"
#include "ROSMap.h"
#include "ROSLaserMeasurement.h"
#include "ROSFilterOutput.h"
#include "ROSCeilCamMeasurement.h"

namespace ohmPf
{

/**
 * @brief This class is used to get the ohmPf running together with ROS.
 * The initialization of the particle filter is done here.
 * ROS callbacks are implemented and pushed through to ohmPf Implementation.
 * Measurement containers are instantiated and updated in the respective callbacks.
 */
class OhmPfNode
{
public:
  /**
   * @brief Standard ros node constructor. Parameter parsing is done here.
   */
  OhmPfNode();

  /**
   * @brief Deconstructor (empty)
   */
  virtual ~OhmPfNode(){};

  /**
   * @brief ROS spin function.
   */
  void spin();


private:
  /**
   * @brief Callback for odometry messages.
   * @param msg ROS odometry message.
   */
  void calOdom(const nav_msgs::OdometryConstPtr& msg);

  /**
   * @brief Callback for a pose estimate for use with rviz.
   * If somebody sends a pose estimate with rviz the particle filter
   * will reinitialize its particle at this pose.
   * @param msg rvizs pose estimate message.
   */
  void cal2dPoseEst(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  /**
   * @brief Callback for a clicked point msg from rviz.
   * If this callback gets called the particle filter will reinitialize with a global
   * localization step.
   * @param msg Point message (is not used)
   */
  void calClickedPoint(const geometry_msgs::PointStampedConstPtr& msg);

  /**
   * @brief Callback for managing ceil cam measurements.
   * @param msg Pose array from CeilCam node.
   */
  void calCeilCam(const geometry_msgs::PoseArrayConstPtr& msg);

  /**
   * @param Modified scan callback. If multible scanners are activated, this callback
   * provides the topic of the scan message to distinguish between different lasers.
   * @param msg ROS laser scanner message.
   * @param topic The topic the scan comes from.
   */
  void calScan(const sensor_msgs::LaserScanConstPtr& msg, const std::string topic);

  /**
   * @brief Helper function for instantiating filter itself
   * and the ROS measurement containers.
   */
  void spawnFilter();

  /**
   * @brief helper function for waiting for the ros map service and to create
   * the ROS map for the filter.
   */
  void waitForMap();

  /**
   * @brief helper function to parse multible laser topics if multible lasers
   * are requested via launch file parameter.
   * @param topic The raw launch file parameter of laser topics. If multible lasers
   * are requested the topics must be separated with ";"
   */
  void parseLaserTopics(std::string topic);

  ros::Publisher _pubSampleSet;
  ros::Publisher _pubProbMap;
  ros::Subscriber _subOdometry;
  ros::Subscriber _subCeilCam;
  std::vector<ros::Subscriber> _subScans;
  ros::Subscriber _sub2dPoseEst;
  ros::Subscriber _subClickedPoint;
  ros::ServiceClient _cliMapSrv;
  ros::NodeHandle _nh;
  ros::NodeHandle _prvNh;
  ros::WallRate* _loopRate;
  OhmPfNodeParams_t _paramSet;
  OdomParams_t _odomParams;
  FilterParams_t _filterParams;
  bool _odomInitialized;
  bool _ceilCamInitialized;
  IFilterController* _filterController;
  ROSOdomMeasurement* _odomMeasurement;
  ROSMap* _map;
  std::vector<ROSLaserMeasurement*> _laserMeasurements;
  ROSFilterOutput* _filterOutput;
  ROSCeilCamMeasurement* _ceilCamMeasurement;
};

} /* namespace ohmPf */

#endif /* SRC_OHMPFNODE_H_ */
