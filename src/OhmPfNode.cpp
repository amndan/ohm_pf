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
    _prvNh.param<std::string>("tfFixedFrame", _paramSet.tfFixedFrame, "/map");
    _prvNh.param<std::string>("tfBaseFootprintFrame", _paramSet.tfBaseFootprintFrame, "base_footprint_ekf");
    _prvNh.param<std::string>("tfOutputFrame", _paramSet.tfOutputFrame, "ohm_pf_output");
    _prvNh.param<std::string>("tfOdomFrame", _paramSet.tfOdomFrame, "odom");
    _prvNh.param<std::string>("topOdometry", _paramSet.topOdometry, "wheelodom");
    _prvNh.param<std::string>("top2dPoseEst", _paramSet.top2dPoseEst, "/initialpose");
    _prvNh.param<std::string>("topClickedPoint", _paramSet.topClickedPoint, "/clicked_point");
    _prvNh.param<std::string>("topParticleCloud", _paramSet.topParticleCloud, "particlecloud");
    _prvNh.param<std::string>("topProbPose", _paramSet.topProbPose, "/probPose");
    _prvNh.param<std::string>("topCeilCam", _paramSet.topCeilCam, "ceilCamPoseArray");
    _prvNh.param<std::string>("topMap", _paramSet.topMap, "/map");
    _prvNh.param<std::string>("topMapSrv", _paramSet.topMapSrv, "/static_map");
    _prvNh.param<std::string>("topScan", _paramSet.topScan, "filtered_scan");
    _prvNh.param<std::string>("resamplingMethod", _filterParams.resamplingMethod, "STD");
    int tmp;
    _prvNh.param<int>("maxDistanceProbMap", tmp, 10);
    assert(tmp > 0);
    _maxDistanceProbMap = (unsigned int)tmp;
    _prvNh.param<int>("subsamplingRateLaser", tmp, 3);
    assert(tmp > 0);
    _rosLaserPMParams.subsamplingRate = (unsigned int)tmp;
    _prvNh.param<int>("samplesMax", tmp, 5000);
    _filterParams.samplesMax = (unsigned int)std::abs(tmp);
    _prvNh.param<int>("samplesMin", tmp, 50);
    _filterParams.samplesMin = (unsigned int)std::abs(tmp);
    _prvNh.param<double>("resamplingIntervallFilter", _filterParams.resamplingIntervall, 0.5);
    double dtmp;
    _prvNh.param<double>("uncertaintyLaser", dtmp, 0.5);
    assert(dtmp >= 0 && dtmp < 1.0);
    _rosLaserPMParams.uncertainty = dtmp;

    _prvNh.param<double>("initX", _paramSet.initPose(0), 0.0);
    _prvNh.param<double>("initY", _paramSet.initPose(1), 0.0);
    _prvNh.param<double>("initPhi", _paramSet.initPose(2), 0.0);

    _prvNh.param<int>("skipParticleForGui", _paramSet.skipParticleForGui, 0);

    _prvNh.param<double>("OCSThresholdLaser", _filterParams.OCSThresholdLaser, 0.2);
    _prvNh.param<double>("OCSThresholdOdom", _filterParams.OCSThresholdOdom, 0.001);
    _prvNh.param<double>("OCSThresholdResampler", _filterParams.OCSThresholdResampler, 0.2);

    _prvNh.param<double>("initSigmaTrans", _paramSet.initSigmaTrans, 0.5);
    _prvNh.param<double>("initSigmaRot", _paramSet.initSigmaRot, 180 / M_PI * 10);

    _prvNh.param<std::string>("initMode", _paramSet.initMode, "GL");

    //_pubSampleSet = _nh.advertise<geometry_msgs::PoseArray>(_paramSet.topParticleCloud, 1, true); // tob: published in ROSFilterOutput
    _pubProbMap = _nh.advertise<nav_msgs::OccupancyGrid>("probMap", 1, true);
    _subOdometry = _nh.subscribe(_paramSet.topOdometry, 1, &OhmPfNode::calOdom, this);
    _subScan = _nh.subscribe(_paramSet.topScan, 1, &OhmPfNode::calScan, this);
    _sub2dPoseEst = _nh.subscribe(_paramSet.top2dPoseEst, 1, &OhmPfNode::cal2dPoseEst, this);
    _subClickedPoint = _nh.subscribe(_paramSet.topClickedPoint, 1, &OhmPfNode::calClickedPoint, this);
    _subCeilCam = _nh.subscribe(_paramSet.topCeilCam, 1, &OhmPfNode::calCeilCam, this);
    _cliMapSrv = _nh.serviceClient<nav_msgs::GetMap>(_paramSet.topMapSrv);

    _rosLaserPMParams.tfBaseFooprintFrame = _paramSet.tfBaseFootprintFrame;

    _resampleTimer = _nh.createTimer(ros::Duration(_filterParams.resamplingIntervall), &OhmPfNode::calResampleTimer, this);

    spawnFilter();

    _laserInitialized = false;
    _odomInitialized = false;

    _map = NULL;

    waitForMap();

    if(_paramSet.initMode == "POSE")
    {
      _filterController->initFilterPose(_paramSet.initPose, _paramSet.initSigmaTrans, _paramSet.initSigmaRot);
    }
  }

  OhmPfNode::~OhmPfNode()
  {

  }

  void OhmPfNode::waitForMap()
  {
    // todo: integrate into gl service
    nav_msgs::GetMap srv_map;

    ROS_INFO("Try to call map service...");

    while(!_cliMapSrv.call(srv_map))
    {
      ros::Duration(0.5).sleep();
      ROS_INFO("%s", _paramSet.topMapSrv.c_str());
      ROS_INFO("No map available --> keep waiting for map...");
    }

    ROS_INFO("Map service called successfully");
    const nav_msgs::OccupancyGrid& map(srv_map.response.map);

    assert(_map == NULL);

    _map = new ROSMap(map, _maxDistanceProbMap);
    _filterController->setMap(_map);

    if(_paramSet.initMode == "GL")
    {
      _filterController->initFilterMap();
    }

    nav_msgs::OccupancyGrid probMapMsg;
    probMapMsg.header = map.header;
    probMapMsg.info = map.info;
    _map->getProbMap(probMapMsg);
    _pubProbMap.publish(probMapMsg);
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
    if(!_odomInitialized)
    {
      ROS_INFO_STREAM("Received first odom message - initializing odom...");
      _odomMeasurement->setMeasurement(msg);

      if(_filterController->setOdomMeasurement(_odomMeasurement, _odomDiffParams))
      {
        _odomInitialized = true;
        ROS_INFO_STREAM("odom initialized");
        return;
      }
      else
      {
        return;
      }
    }

    _odomMeasurement->setMeasurement(msg);
    _filterController->updateOdom();
    _filterController->updateOutput();
  }

  void OhmPfNode::cal2dPoseEst(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
  {
    Eigen::Vector3d pose;
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = tf::getYaw(msg->pose.pose.orientation);

    _filterController->initFilterPose(pose, 1.0, M_PI / 180 * 10);
  }

  void OhmPfNode::calClickedPoint(const geometry_msgs::PointStampedConstPtr& msg)
  {
    _filterController->initFilterMap();
  }

  void OhmPfNode::spawnFilter()
  {
    _filterController = IFilterController::createFilter(_filterParams);

    _filterOutput = new ROSFilterOutput(_paramSet);
    assert(_filterController->setFilterOutput(_filterOutput));

    _ceilCamMeasurement = new ROSCeilCamMeasurement();

    //todo: get odom Params from Launchfile
    _odomDiffParams.a1 = 0.005;  // rot error from rot motion
    _odomDiffParams.a2 = 20;  // rot error from trans motion
    _odomDiffParams.a3 = 0.01;  // trans error from trans motion
    _odomDiffParams.a4 = 0.0;  // trans error from rot motion
    _odomMeasurement = new ROSOdomMeasurement();

    _laserMeasurement = new ROSLaserMeasurement(_rosLaserPMParams.uncertainty);

  }

  void OhmPfNode::calCeilCam(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    if(!_ceilCamInitialized)
    {
      if(_filterController->setCeilCamMeasurement(_ceilCamMeasurement))
      {
        _ceilCamInitialized = true;
      }
      else
      {
        return;
      }
    }
    _ceilCamMeasurement->setMeasurement(msg);
    _filterController->updateCeilCam();
  }

  void OhmPfNode::calScan(const sensor_msgs::LaserScanConstPtr& msg)
  {

    if(!_laserInitialized)
    {
      _laserMeasurement->initWithMeasurement(msg, _rosLaserPMParams.tfBaseFooprintFrame);
      if(_filterController->setLaserMeasurement(_laserMeasurement))
      {
        _laserInitialized = true;
        return;
      }
      else
      {
        return;
      }
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
