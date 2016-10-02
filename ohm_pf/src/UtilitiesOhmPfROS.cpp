/*
 * UtilitiesOhmPfROS.cpp
 *
 *  Created on: Oct 2, 2016
 *      Author: amndan
 */

#include "UtilitiesOhmPfROS.h"

namespace ohmPf
{

tf::Transform eigenMatrix3x3ToTf(const Eigen::Matrix3d& eigen)
{
  tf::Transform tf;
  tf.setOrigin(tf::Vector3(eigen(0, 2), eigen(1, 2), 0.0));
  tf.setRotation(tf::createQuaternionFromYaw(asin(eigen(0, 1))));

  return tf;
}

Eigen::Matrix3d tfToEigenMatrix3x3(const tf::Transform& tf)
{
  Eigen::Matrix3d eigen(3, 3);
  eigen.setIdentity();

  double theta = tf::getYaw(tf.getRotation());
  double x = tf.getOrigin().getX();
  double y = tf.getOrigin().getY();

  // preigenlem with sin() returns -0.0 (avoid with +0.0)
  eigen(0, 0) = cos(theta) + 0.0;
  eigen(0, 1) = -sin(theta) + 0.0;
  eigen(0, 2) = x + 0.0;
  eigen(1, 0) = sin(theta) + 0.0;
  eigen(1, 1) = cos(theta) + 0.0;
  eigen(1, 2) = y + 0.0;

  return eigen;
}

} //~ohmpf
