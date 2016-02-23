/*
 * RosLaserPM.cpp
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#include "RosLaserPM.h"

namespace ohmPf
{

  RosLaserPM::RosLaserPM(std::string tfBaseFooprintFrame)
  {
    _initialized = false;

    _paramSet.tfBaseFooprintFrame = tfBaseFooprintFrame;
  }

  RosLaserPM::~RosLaserPM()
  {
    // TODO Auto-generated destructor stub
  }

  void RosLaserPM::updateFilter(Filter& filter)
  {
    if (!_initialized)
    {
      initWithMeasurement();
    }


    Eigen::Matrix3Xd coords = rangesToCoordinates(_actualScan.ranges);

    std::vector<Sample_t>* samples = filter.getSampleSet()->getSamples();

    std::cout << "samples count: " << samples->size() << std::endl;

    Eigen::Matrix3Xd coordsTf;
    Eigen::Matrix3d tf;

    ros::Time t0;
    ros::Time t1;
    t0 = ros::Time::now();

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it) // each sample
    {
      // transform scan to position of particle
      create3x3TransformationMatrix(it->pose(0), it->pose(1), it->pose(2), tf); // todo: dont forget laser tf
      coordsTf = tf * _tfBaseFootprintToLaser * coords;

      // lookup probs
      it->weight = ( (MapModel&) filter.getSensor(MAP) ).getProbability(coordsTf);
    }

    t1 = ros::Time::now();
    ros::Duration dur = t1 - t0;
    std::cout << "calScan Duration: " << dur << std::endl;

    filter.getSampleSet()->boostWeights();
    filter.updateWithSensor(MAP);
    //filter.getSampleSet()->normalize();
    //filter.getSampleSet()->resample(); // todo: should we do that here??

    return;

  }

  void RosLaserPM::setMeasurement(const sensor_msgs::LaserScanConstPtr& scanMsg)
  {
    _actualScan = *scanMsg;
  }

  void RosLaserPM::initWithMeasurement()
  {
    _paramSet.angleIncrement = (double) _actualScan.angle_increment;
    _paramSet.angleMax = (double) _actualScan.angle_max;
    _paramSet.angleMin = (double) _actualScan.angle_min;
    _paramSet.rangeMax = (double) _actualScan.range_max;
    _paramSet.rangeMin = (double) _actualScan.range_min;

    _paramSet.tfLaserFrame = _actualScan.header.frame_id;

    double angleRange = _paramSet.angleMax - _paramSet.angleMin;

    assert(angleRange > 0.0);
    assert(_paramSet.angleIncrement != 0);

    int count = (int) (angleRange / _paramSet.angleIncrement); // todo: angle inc and ranges.size() dont equals!

    //assert(_actualScan.ranges.size() == count);

    _paramSet.count = _actualScan.ranges.size();
    _paramSet.angleIncrement = std::abs(angleRange) / _paramSet.count;

    tf::Transform tf;
    tf::StampedTransform tmp;
    tf::TransformListener tfListener;

    tfListener.waitForTransform(_paramSet.tfBaseFooprintFrame, _paramSet.tfLaserFrame, ros::Time(0), ros::Duration(3.0));
    assert(tfListener.canTransform(_paramSet.tfBaseFooprintFrame, _paramSet.tfLaserFrame, ros::Time(0)));
    tfListener.lookupTransform(_paramSet.tfBaseFooprintFrame, _paramSet.tfLaserFrame, ros::Time(0), tmp);

    tf = tmp; // stamped to not stamped

    _tfBaseFootprintToLaser = tfToEigenMatrix3x3(tf);

    _initialized = true;
  }

  Eigen::Matrix3Xd RosLaserPM::rangesToCoordinates(std::vector<float>& ranges)
  {
    assert(ranges.size() == _paramSet.count);

    int subSampFact = 3; // todo: use parameter --> subsampling in laserFilter?

    int iter = (int) std::floor(_paramSet.count / subSampFact);

    Eigen::Matrix3Xd scanCoord(3,iter+1); // abs(5 / 2) = 2 --> 0 2 4 --> 2 + 1 = 3

    for(unsigned int i = 0, j = 0; i < _paramSet.count; i = i + subSampFact, j++)
    {
      if( ranges[i] <= _paramSet.rangeMax && ranges[i] >= _paramSet.rangeMin &&  !std::isinf(ranges[i]) ) //todo: laserfilter
      {
        scanCoord(0,j) = ranges[i] * std::cos(_paramSet.angleMin + i * _paramSet.angleIncrement);
        scanCoord(1,j) = ranges[i] * std::sin(_paramSet.angleMin + i * _paramSet.angleIncrement);
        scanCoord(2,j) = 1;
      }
      else
      {
        scanCoord(0,j) = 0.0; // todo: perhaps use a mask here
        scanCoord(1,j) = 0.0;
        scanCoord(2,j) = 0.0;
      }

    }
    return scanCoord;
  }

  void RosLaserPM::initFilter(Filter& filter)
  {
    std::cout << "RosLaserPM can't init filter..." << std::endl;
  }

} /* namespace ohmPf */


