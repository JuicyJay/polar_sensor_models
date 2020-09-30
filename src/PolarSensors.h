/*
 * PolarSensors.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Jasmin Ziegler
 */

#include "obvision/reconstruct/space/TsdSpace.h"
#include "obvision/reconstruct/space/VelodyneVLP16.h"
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class PolarSensors
{
public:
  PolarSensors(const float dimX, const float dimY, const float dimZ, const float cellSize);
  virtual ~PolarSensors();
  void callbackPointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

private:
  void init(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  std::unique_ptr<obvious::TsdSpace> _space;
  //   std::unique_ptr<obvious::VelodyneVLP16> _sensor; // nicht als member machen sondern übergeben in den Methoden, damit ich über launchfile sensortype
  //   bestimmen kann

  ros::Subscriber _subPointcloud;

  ros::NodeHandle _nh;
  ros::NodeHandle _prvnh;

  std::string _sensorType;
  std::string _laserDataTopic;
  double      _inclMin;
  double      _inclMax;
  double      _inclRes;
  double      _azimMin;
  double      _azimMax;
  double      _azimRes;

  std::vector<int> _firingSeq;

  float _dimX;
  float _dimY;
  float _dimZ;
  float _cellSize;
  float _cellsX;
  float _cellsY;
  float _cellsZ;
}