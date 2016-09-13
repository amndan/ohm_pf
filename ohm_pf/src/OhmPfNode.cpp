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
      _nh(), _prvNh("~"), _loopRate(50)
  {
    std::string rawLaserTopicString;
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
    _prvNh.param<std::string>("topScan", rawLaserTopicString, "filtered_scan");

    int itmp;
    double dtmp;

    _prvNh.param<std::string>("resamplingMethod", _filterParams.resamplingMethod, "STD");
    _prvNh.param<int>("maxDistanceProbMap", itmp, 10);
    assert(itmp > 0);
    _filterParams.maxDistanceProbMap = (unsigned int)itmp;
    _prvNh.param<int>("samplesMax", itmp, 5000);
    _filterParams.samplesMax = (unsigned int)std::abs(itmp);
    _prvNh.param<int>("samplesMin", itmp, 50);
    _filterParams.samplesMin = (unsigned int)std::abs(itmp);
    _prvNh.param<double>("resamplingIntervallFilter", _filterParams.resamplingIntervall, 0.5);
    _prvNh.param<double>("outputIntervallFilter", _filterParams.outputIntervall, 1.0);
    _prvNh.param<double>("uncertaintyLaser", _paramSet.uncertaintyLaser, 0.5);
    _prvNh.param<double>("minimumValidScanRaysFactor", _filterParams.minValidScanRaysFactor, 0.5);
    _prvNh.param<double>("additionalTranslationalNoise", _filterParams.resamplerAdditionalTranslationalNoise, 0.05);
    _prvNh.param<double>("additionalRotationalNoise", _filterParams.resamplerAdditionalRotationalNoise, 10.0 / 180.0 * M_PI);

    _prvNh.param<int>("lowVarianceFactor", itmp, 3);
    _filterParams.resamplerLowVarianceFactor = (unsigned int) itmp;
    _prvNh.param<int>("subsamplingRateLaser", itmp, 3);
    _paramSet.subsamplingLaser = (unsigned int) itmp;
    _prvNh.param<int>("countLasers", itmp, 1);
    assert(itmp > 0);
    _filterParams.countLasers = itmp;

    _prvNh.param<double>("initX", _paramSet.initPose(0), 0.0);
    _prvNh.param<double>("initY", _paramSet.initPose(1), 0.0);
    _prvNh.param<double>("initPhi", _paramSet.initPose(2), 0.0);

    _prvNh.param<int>("skipParticleForGui", _paramSet.skipParticleForGui, 0);

    _prvNh.param<double>("OCSThresholdLaser", _filterParams.OCSThresholdLaser, 0.2);
    _prvNh.param<double>("OCSThresholdOdom", _filterParams.OCSThresholdOdom, 0.001);
    _prvNh.param<double>("OCSThresholdResampler", _filterParams.OCSThresholdResampler, 0.2);
    _prvNh.param<double>("OCSRotToTransFactor", _filterParams.OCSRotToTransFactor, 8.0);

    _prvNh.param<double>("initSigmaTrans", _paramSet.initSigmaTrans, 0.5);
    _prvNh.param<double>("initSigmaRot", _paramSet.initSigmaRot, 180 / M_PI * 10);

    _prvNh.param<std::string>("initMode", _paramSet.initMode, "GL");

    //_pubSampleSet = _nh.advertise<geometry_msgs::PoseArray>(_paramSet.topParticleCloud, 1, true); // tob: published in ROSFilterOutput
    _pubProbMap = _nh.advertise<nav_msgs::OccupancyGrid>("probMap", 1, true);
    _subOdometry = _nh.subscribe(_paramSet.topOdometry, 1, &OhmPfNode::calOdom, this);
    _sub2dPoseEst = _nh.subscribe(_paramSet.top2dPoseEst, 1, &OhmPfNode::cal2dPoseEst, this);
    _subClickedPoint = _nh.subscribe(_paramSet.topClickedPoint, 1, &OhmPfNode::calClickedPoint, this);
    _subCeilCam = _nh.subscribe(_paramSet.topCeilCam, 1, &OhmPfNode::calCeilCam, this);
    _cliMapSrv = _nh.serviceClient<nav_msgs::GetMap>(_paramSet.topMapSrv);


    parseLaserTopics(rawLaserTopicString);
    spawnFilter();

    _odomInitialized = false;

    _map = NULL;

    waitForMap();

    if(_paramSet.initMode == "POSE")
    {
      _filterController->initFilterPose(_paramSet.initPose, _paramSet.initSigmaTrans, _paramSet.initSigmaRot);
    }
  }

  void OhmPfNode::parseLaserTopics(std::string topic)
  {
    std::istringstream ss(topic);
    std::string token;
    unsigned int nScanners = 0;

    while(std::getline(ss, token, ';'))
    {
      _paramSet.topScans.push_back(token);
      nScanners++;
    }

    if(_filterParams.countLasers != nScanners)
    {
      ROS_ERROR("_filterParams.countLasers != nScanners; number of laser topics and lasers must be the same! --> exit");
      exit(EXIT_FAILURE);
    }

    for(unsigned int i = 0; i < nScanners; i++)
    {
      _subScans.push_back(_nh.subscribe<sensor_msgs::LaserScan>(_paramSet.topScans.at(i), 1, boost::bind(&OhmPfNode::calScan, this, _1, _paramSet.topScans.at(i))));

      std::cout << "scannerNr: " << i << std::endl;

      _laserMeasurements.push_back(NULL);
    }
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

    _map = new ROSMap(map);

    _filterController->setMap(_map);

    if(_paramSet.initMode == "GL")
    {
      _filterController->initFilterMap();
    }

    // Should not use map messages information because prob map could be new format!
    nav_msgs::OccupancyGrid probMapMsg;
    probMapMsg.header = map.header;
    Eigen::Matrix3d originTf;
    probMapMsg.info = map.info;

    _filterController->requestProbMap(
        probMapMsg.info.width,
        probMapMsg.info.height,
        probMapMsg.info.resolution,
        originTf,
        probMapMsg.data);

    tf::Transform tmp = eigenMatrix3x3ToTf(originTf);
    tf::quaternionTFToMsg(tmp.getRotation(), probMapMsg.info.origin.orientation);
    probMapMsg.info.origin.position.x = tmp.getOrigin().getX();
    probMapMsg.info.origin.position.y = tmp.getOrigin().getY();
    probMapMsg.info.origin.position.z = 0.0;

    _pubProbMap.publish(probMapMsg);

  }

  void OhmPfNode::spin()
  {
    while(ros::ok())
    {
      ros::spinOnce();  // update measurements
      _filterController->filterSpinOnce();  // integrate measurements
      _filterOutput->publishMapOdom();  // publish map->odom
      _loopRate.sleep();  // sleep
    }
  }

  void OhmPfNode::calOdom(const nav_msgs::OdometryConstPtr& msg)
  {
    if(!_odomInitialized)
    {
      ROS_INFO_STREAM("Received first odom message - initializing odom...");
      _odomMeasurement->setMeasurement(msg);

      if(_filterController->connectOdomMeasurement(_odomMeasurement, _odomDiffParams))
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
    // call factory method from lib
    _filterController = IFilterController::createFilter(_filterParams);

    _filterOutput = new ROSFilterOutput(_paramSet);
    assert(_filterController->connectFilterOutput(_filterOutput));

    _ceilCamMeasurement = new ROSCeilCamMeasurement();

    //todo: get odom Params from Launchfile
    _odomDiffParams.a1 = 0.01;  // rot error from rot motion
    _odomDiffParams.a2 = 10.0;  // rot error from trans motion
    _odomDiffParams.a3 = 0.01;  // trans error from trans motion
    _odomDiffParams.a4 = 0.001;  // trans error from rot motion
    _odomMeasurement = new ROSOdomMeasurement();

  }

  void OhmPfNode::calCeilCam(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    if(!_ceilCamInitialized)
    {
      if(_filterController->connectCeilCamMeasurement(_ceilCamMeasurement))
      {
        _ceilCamInitialized = true;
      }
      else
      {
        return;
      }
    }
    _ceilCamMeasurement->setMeasurement(msg);
  }

  void OhmPfNode::calScan(const sensor_msgs::LaserScanConstPtr& msg, const std::string topic)
  {
    unsigned int i = 0;

    // determine laserId from topic
    while(i < _paramSet.topScans.size())
    {
      if(!topic.compare(_paramSet.topScans.at(i)))
      {
        break;
      }
      i++;
    }

    if(_laserMeasurements.at(i) == NULL)
    {
      _laserMeasurements.at(i) = new ROSLaserMeasurement(msg,
                                                         _paramSet.tfBaseFootprintFrame,
                                                         _paramSet.subsamplingLaser,
                                                         _paramSet.uncertaintyLaser);

      if(_filterController->connectLaserMeasurement(_laserMeasurements.at(i), i))
      {
        return;
      }
      else
      {
        delete _laserMeasurements.at(i);
        _laserMeasurements.at(i) = NULL;
        return;
      }
    }
    else
    {
      _laserMeasurements.at(i)->setMeasurement(msg);
    }
  }

} /* namespace ohmPf */
