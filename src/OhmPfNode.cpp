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

  _odomInitialized = false;

  _resampleTimer = _nh.createTimer(ros::Duration(_filterParams.resamplingIntervall), &OhmPfNode::calResampleTimer, this); 

  spawnOdom();
  spawnFilter();

  _filter->setSensor(CEILCAM, (Sensor*) new CeilCam());
  _filter->setSensor(LASER, (Sensor*) new RosLaserPM(_rosLaserPMParams));
  _filter->setSensor(RESAMPLER, (Sensor*) new Resampler());

  _cumSumRot = 0.0;
  _cumSumtrans = 0.0;
}

OhmPfNode::~OhmPfNode()
{
  delete _odomDiff;
  delete _filter;
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

void OhmPfNode::printSampleSet(SampleSet* sampleSet){
  std::vector<Sample_t> samples;
  samples = *(sampleSet->getSamples());

  geometry_msgs::PoseArray poseArray;
  geometry_msgs::Pose pose;

  poseArray.header.frame_id = _paramSet.tfFixedFrame;

  for(unsigned int i = 0; i < samples.size(); i++)
  {
    pose.position.x = samples[i].pose(0);
    pose.position.y = samples[i].pose(1);
    pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw( samples[i].pose(2) ), pose.orientation);
    poseArray.poses.push_back(pose);
  }
  _pubSampleSet.publish(poseArray);
}

void OhmPfNode::calOdom(const nav_msgs::OdometryConstPtr& msg)
{
  //#######################

//  odomCounter++;
//  if (odomCounter > 10)
//  {
//    odomCounter = 0;
//    double dist;
//
//    std::vector<Sample_t>* samples = _filter->getSampleSet()->getSamples();
//
//    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
//    {
//      dist = std::sqrt( std::pow(it->pose(0) - msg->pose.pose.position.x,2) + //...
//          std::pow(it->pose(1) - msg->pose.pose.position.y,2)) +
//          std::pow(it->pose(2) - tf::getYaw(msg->pose.pose.orientation),2);
//
//      it->weight = it->weight * GaussianPdf::getProbability(0.0, 2, dist);
//    }
//
//    _filter->getSampleSet()->normalize();
//    _filter->getSampleSet()->resample();
//    printSampleSet(_filter->getSampleSet());
//
//    ROS_INFO_STREAM("resampled with odom!");
//
//  }

  //#######################


  Eigen::Vector3d measurement;
  measurement(0) = msg->pose.pose.position.x;
  measurement(1) = msg->pose.pose.position.y;
  measurement(2) = tf::getYaw(msg->pose.pose.orientation);

  if(!_odomInitialized)
  {
    ROS_INFO_STREAM("Received first odom message - initializing odom...");
    _odomDiff->addSingleMeasurement(measurement);
    _lastOdomPose = measurement;
    _odomInitialized = true;
    ROS_INFO_STREAM("odom initialized");

    //todo remove init from here
    _filter->initWithPose(measurement);
    printSampleSet(_filter->getSampleSet());

    return;
  }

  _odomDiff->addSingleMeasurement(measurement);
  _odomDiff->updateFilter(*_filter);
  printSampleSet(_filter->getSampleSet());

  Eigen::Vector3d diff = measurement - _lastOdomPose;
  _cumSumtrans += std::sqrt(std::pow(diff(0) ,2) + std::pow(diff(1) ,2));
  _cumSumRot += std::abs(diff(2));

  if(_cumSumRot > 0.01 || _cumSumtrans > 0.01)
  {
    _filter->triggerOdomChangedSignificantly();
    _cumSumRot = 0;
    _cumSumtrans = 0;
  }

  _lastOdomPose = measurement;

}

void OhmPfNode::cal2dPoseEst(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
//  Eigen::Vector3d measurement;
//  measurement(0) = msg->pose.pose.position.x;
//  measurement(1) = msg->pose.pose.position.y;
//  measurement(2) = tf::getYaw(msg->pose.pose.orientation);
//
//  //_filter->initWithPose(measurement);
//  _filter->initWithMap();
//  printSampleSet(_filter->getSampleSet());

  // todo: integrate into gl service
  nav_msgs::GetMap srv_map;

  if (_cliMapSrv.call(srv_map))
  {
    ROS_INFO("Map service called successfully");
    const nav_msgs::OccupancyGrid& map(srv_map.response.map);

    static RosMap* rosMap;
    delete rosMap;
    rosMap = new RosMap(map, _maxDistanceProbMap);



    _filter->setSensor(MAP, rosMap);

    nav_msgs::OccupancyGrid probMapMsg;
    probMapMsg.header = map.header;
    probMapMsg.info = map.info;
    ((RosMap&) _filter->getSensor(MAP)).getProbMap(probMapMsg);
    //rosMap->getProbMap(probMapMsg);
    _pubProbMap.publish(probMapMsg);

    _filter->initWithSensor(MAP);


  }
  else
  {
    ROS_ERROR("Failed to call map service");
    return;
  }
}

void OhmPfNode::spawnOdom()
{
  //todo: get odom Params from Launchfile
  _odomDiffParams.a1 = 0.005;
  _odomDiffParams.a2 = 0.0;
  _odomDiffParams.a3 = 0.01;
  _odomDiffParams.a4 = 0.0;

  _odomDiff = new ohmPf::OdomDiff(_odomDiffParams);
}

void OhmPfNode::spawnFilter()
{
  _filter = new ohmPf::Filter(_filterParams);
}

void OhmPfNode::calCeilCam(const geometry_msgs::PoseArrayConstPtr& msg)
{
  if (_filter->isInitialized())
  {
    std::vector<Eigen::Vector3d> measurement;

    // todo: why do i need this conversation?
    std::vector<geometry_msgs::Pose> vec = msg->poses;

    for(std::vector<geometry_msgs::Pose>::iterator it = vec.begin(); it != vec.end(); ++it)
    {
      Eigen::Vector3d pose;
      pose(0) = it->position.x;
      pose(1) = it->position.y;
      pose(2) = tf::getYaw(it->orientation);
      measurement.push_back(pose);
    }

    ((CeilCam&) _filter->getSensor(CEILCAM)).setMeasurement(measurement);
    _filter->updateWithSensor(CEILCAM);
  }
}

  void OhmPfNode::calScan(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if(&_filter->getSensor(MAP) != NULL && &_filter->getSensor(LASER) != NULL)
    {
      RosLaserPM& laser = (RosLaserPM&)_filter->getSensor(LASER);
      laser.setMeasurement(msg);
      _filter->updateWithSensor(LASER);
      //laser.updateFilter(*_filter); // todo: das sollte nur der filter können!?
      printSampleSet(_filter->getSampleSet());
    }
  }

  void OhmPfNode::calResampleTimer(const ros::TimerEvent& event)
  {
    if(_filter->isInitialized())
    {
      if(&_filter->getSensor(RESAMPLER) != NULL)
      {
        _filter->updateWithSensor(MAP); 
        //TODO: we need a resampling method without LVS because after map
        //update no particle should be on occupied cells; or map updates 
        //not weights but whole particle set and seeds random particles
        _filter->updateWithSensor(RESAMPLER);
        printSampleSet(_filter->getSampleSet());
      }
    }
  }  

} /* namespace ohmPf */
