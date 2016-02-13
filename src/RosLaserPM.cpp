/*
 * RosLaserPM.cpp
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#include "RosLaserPM.h"

namespace ohmPf
{

  RosLaserPM::RosLaserPM()
  {
    _initialized = false;
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

    ros::Time t0;
    ros::Time t1;
    t0 = ros::Time::now();

    t1 = ros::Time::now();
    ros::Duration dur = t1 - t0;
    std::cout << "calScan Duration: " << dur << std::endl;

    std::vector<Sample_t>* samples = filter.getSampleSet()->getSamples();

    std::cout << "samples count: " << samples->size() << std::endl;

    double probOfSample = 1.0;
    double prob = 1.0;

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it) // each sample
    {
      // transform scan to position of particle
      Eigen::Matrix3d tf = create3x3TransformationMatrix(it->pose(0), it->pose(1), it->pose(2)); // todo: dont forget laser tf
      Eigen::Matrix3Xd coordsTf = tf * coords;

      // lookup probs

      probOfSample = 1.0;

      for(unsigned int i = 0; i < _paramSet.count; i++) // whole scan
      {
        if(coordsTf(2,i) != 0)
        {
          prob = filter.getMap()->getProbability(coordsTf(0,i), coordsTf(1,i));
          prob = 0.2 * prob + 0.8; // todo: magic numbers
          probOfSample *= prob;
        }
      }
      it->weight = probOfSample;
    }


    filter.getSampleSet()->normalize(); // todo: when to normalize?

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

    double angleRange = _paramSet.angleMax - _paramSet.angleMin;

    assert(angleRange > 0.0);
    assert(_paramSet.angleIncrement != 0);

    int count = (int) (angleRange / _paramSet.angleIncrement); // todo: angle inc and ranges.size() dont equals!

    //assert(_actualScan.ranges.size() == count);

    _paramSet.count = _actualScan.ranges.size();
    _paramSet.angleIncrement = std::abs(angleRange) / _paramSet.count;

    _initialized = true;
  }

  Eigen::Matrix3Xd RosLaserPM::rangesToCoordinates(std::vector<float>& ranges)
  {
    assert(ranges.size() == _paramSet.count);

    Eigen::Matrix3Xd scanCoord(3,_paramSet.count);

    for(unsigned int i = 0; i < _paramSet.count; i++)
    {
      if( ranges[i] <= _paramSet.rangeMax && ranges[i] >= _paramSet.rangeMin &&  !std::isinf(ranges[i]) ) //todo: laserfilter
      {
        scanCoord(0,i) = ranges[i] * std::cos(_paramSet.angleMin + i * _paramSet.angleIncrement);
        scanCoord(1,i) = ranges[i] * std::sin(_paramSet.angleMin + i * _paramSet.angleIncrement);
        scanCoord(2,i) = 1;
      }
      else
      {
        scanCoord(0,i) = 0.0; // todo: perhaps use a mask here
        scanCoord(1,i) = 0.0;
        scanCoord(2,i) = 0.0;
      }
    }
    return scanCoord;
  }

} /* namespace ohmPf */
