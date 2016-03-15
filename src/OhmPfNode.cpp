/*
 * OhmPfNode.cpp
 *
 *  Created on: Jan 5, 2016
 *      Author: amndan
 */

#include "OhmPfNode.h"

namespace ohmPf
{

OhmPfNode::OhmPfNode() :
    _nh(), _prvNh("~"), _loopRate(25)
{
  _prvNh.param<std::string>("tfFixedFrame", _paramSet.tfFixedFrame, "map");
  _prvNh.param<std::string>("tfBaseFootprintFrame", _paramSet.tfBaseFootprintFrame, "base_footprint");
  _prvNh.param<std::string>("topOdometry", _paramSet.topOdometry, "robot0/odom");
  _prvNh.param<std::string>("top2dPoseEst", _paramSet.top2dPoseEst, "initialpose");
  _prvNh.param<std::string>("topCeilCam", _paramSet.topCeilCam, "ceilCamPoseArray");
  _prvNh.param<std::string>("topMap", _paramSet.topMap, "map");
  _prvNh.param<std::string>("topMapSrv", _paramSet.topMapSrv, "static_map");
  _prvNh.param<std::string>("topScan", _paramSet.topScan, "robot0/laser_0");
  int tmp;
  _prvNh.param<int>("maxDistanceProbMap", tmp, 10);
  assert(tmp > 0);
  _maxDistanceProbMap = (unsigned int) tmp;
  _prvNh.param<int>("subsamplingRateLaser", tmp, 3);
  assert(tmp > 0);
  _rosLaserPMParams.subsamplingRate = (unsigned int) tmp;
  _prvNh.param<int>("samplesMax", tmp, 5000);
  _filterParams.samplesMax = (unsigned int) std::abs(tmp);
  _prvNh.param<int>("samplesMin", tmp, 50);
  _filterParams.samplesMin = (unsigned int) std::abs(tmp);
  _prvNh.param<double>("resamplingIntervallFilter", _filterParams.resamplingIntervall, 0.5);
  double dtmp;
  _prvNh.param<double>("uncertaintyLaser",dtmp,  0.5);
  assert(dtmp >= 0 && dtmp < 1.0);
  _rosLaserPMParams.uncertainty = dtmp;

  _pubSampleSet = _nh.advertise<geometry_msgs::PoseArray>("particleCloud", 1, true);
  _pubProbMap = _nh.advertise<nav_msgs::OccupancyGrid>("probMap", 1, true);
  _subOdometry = _nh.subscribe(_paramSet.topOdometry, 1, &OhmPfNode::calOdom, this);
  _subScan = _nh.subscribe(_paramSet.topScan, 1, &OhmPfNode::calScan, this);
  _sub2dPoseEst = _nh.subscribe(_paramSet.top2dPoseEst, 1, &OhmPfNode::cal2dPoseEst, this);
  _subCeilCam = _nh.subscribe(_paramSet.topCeilCam, 1, &OhmPfNode::calCeilCam, this);
  _cliMapSrv = _nh.serviceClient<nav_msgs::GetMap>(_paramSet.topMapSrv);


  _rosLaserPMParams.tfBaseFooprintFrame = _paramSet.tfBaseFootprintFrame;

  _resampleTimer = _nh.createTimer(ros::Duration(_filterParams.resamplingIntervall), &OhmPfNode::calResampleTimer, this); 

  spawnFilter();

  _laserInitialized = false;
  _odomInitialized = false;

  _cumSumRot = 0.0;
  _cumSumtrans = 0.0;
}

OhmPfNode::~OhmPfNode()
{

}

void OhmPfNode::spin()
{
  ros::spin();
}

void OhmPfNode::spinOnce()
{
  if(ros::ok())
  {
    _loopRate.sleep();
    ros::spinOnce();
  }
  else
  {
    exit(EXIT_FAILURE);
  }
}


void OhmPfNode::calOdom(const nav_msgs::OdometryConstPtr& msg)
{
  Eigen::Vector3d measurement;
  measurement(0) = msg->pose.pose.position.x;
  measurement(1) = msg->pose.pose.position.y;
  measurement(2) = tf::getYaw(msg->pose.pose.orientation);

  if(!_odomInitialized)
  {
    ROS_INFO_STREAM("Received first odom message - initializing odom...");
    //todo: get odom Params from Launchfile
    _odomDiffParams.a1 = 0.005;
    _odomDiffParams.a2 = 0.0;
    _odomDiffParams.a3 = 0.01;
    _odomDiffParams.a4 = 0.0;

    _odomMeasurement = new ROSOdomMeasurement();
    _odomMeasurement->setMeasurement(msg);

    if(_filterController->setOdomMeasurement(_odomMeasurement, _odomDiffParams))
    {
      _odomInitialized = true;
      // DEBUG
      _lastOdomPose = measurement;
      _odomInitialized = true;
      ROS_INFO_STREAM("odom initialized");
      return;
    }
    else
    {
      exit(EXIT_FAILURE);
    }
  }

  _odomMeasurement->setMeasurement(msg);
  _filterController->updateOdom();
  _filterController->updateOutput();

  Eigen::Vector3d diff = measurement - _lastOdomPose;
  _cumSumtrans += std::sqrt(std::pow(diff(0) ,2) + std::pow(diff(1) ,2));
  _cumSumRot += std::abs(diff(2));

  if(_cumSumRot > 0.01 || _cumSumtrans > 0.01)
  {
    //_filter->triggerOdomChangedSignificantly();
    _cumSumRot = 0;
    _cumSumtrans = 0;
  }

  _lastOdomPose = measurement;

}

void OhmPfNode::cal2dPoseEst(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // todo: integrate into gl service
  nav_msgs::GetMap srv_map;

  if (_cliMapSrv.call(srv_map))
  {
    ROS_INFO("Map service called successfully");
    const nav_msgs::OccupancyGrid& map(srv_map.response.map);



    delete _map;

    _map = new ROSMap(map, _maxDistanceProbMap);
    _filterController->setMap(_map);
    _filterController->initFilterMap();

//    nav_msgs::OccupancyGrid probMapMsg;
//    probMapMsg.header = map.header;
//    probMapMsg.info = map.info;
//    ((MapUpdater&) _filter->getSensor(MAP)).getProbMap(probMapMsg);
//    //rosMap->getProbMap(probMapMsg);
//    _pubProbMap.publish(probMapMsg);

  }
  else
  {
    ROS_ERROR("Failed to call map service");
    return;
  }
}


void OhmPfNode::spawnFilter()
{
  _filterController = IFilterController::createFilter(_filterParams);

  _filterOutput = new ROSFilterOutput(_paramSet.tfFixedFrame);
  assert(_filterController->setFilterOutput(_filterOutput));

}

void OhmPfNode::calCeilCam(const geometry_msgs::PoseArrayConstPtr& msg)
{
//  if (_filter->isInitialized())
//  {
//    std::vector<Eigen::Vector3d> measurement;
//
//    // todo: why do i need this conversation?
//    std::vector<geometry_msgs::Pose> vec = msg->poses;
//
//    for(std::vector<geometry_msgs::Pose>::iterator it = vec.begin(); it != vec.end(); ++it)
//    {
//      Eigen::Vector3d pose;
//      pose(0) = it->position.x;
//      pose(1) = it->position.y;
//      pose(2) = tf::getYaw(it->orientation);
//      measurement.push_back(pose);
//    }
//
//    ((CeilCamUpdater&) _filter->getSensor(CEILCAM)).setMeasurement(measurement);
//    _filter->updateWithSensor(CEILCAM);
//  }
}

  void OhmPfNode::calScan(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if(!_laserInitialized)
    {
      _laserMeasurement = new ROSLaserMeasurement();
      _laserMeasurement->initWithMeasurement(msg, _rosLaserPMParams.tfBaseFooprintFrame);
      if( _filterController->setLaserMeasurement(_laserMeasurement) )
      {
        _laserInitialized = true;
        return;
      }
      return;
    }
    else
    {
      _laserMeasurement->setMeasurement(msg);
      _filterController->updateLaser();
    }
  }

  void OhmPfNode::calResampleTimer(const ros::TimerEvent& event)
  {
    //TODO: we need a resampling method without LVS because after map
    //update no particle should be on occupied cells; or map updates
    //not weights but whole particle set and seeds random particles
    _filterController->resample();
    _filterController->updateOutput();

  }  

} /* namespace ohmPf */
