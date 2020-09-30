/*
 * PolarSensors.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Jasmin Ziegler
 */

#include "PolarSensors.h"

PolarSensors::PolarSensors(const float dimX, const float dimY, const float dimZ, const float cellSize)
    : _nh(), _prvnh("~"), _dimX(dimX), _dimY(dimY), _dimZ(dimZ), _cellSize(cellSize), /**_listener(std::make_unique<tf::TransformListener>()),**/ _cellsX(0),
      _cellsY(0), _cellsZ(0)
{
  // launchfile parameters, default vals for VLP16
  _prvnh.param<std::string>("sensorType", _sensorType, "VLP16");
  _prvnh.param<std::string>("laserDataTopic", _laserDataTopic, "puck_rear/velodyne_points");
  _prvnh.param<double>("inclMin", _inclMin, -0.26180); //-15°
  _prvnh.param<double>("inclMax", _inclMax, 0.26180);  //+15°
  _prvnh.param<double>("inclRes", _inclRes, 0.034907); // 2°
  _prvnh.param<double>("azimMin", _azimMin, 0.0);
  _prvnh.param<double>("azimMax", _azimMax, 6.28319);    // 360°
  _prvnh.param<double>("azimRes", _azimRes, 0.00349066); // 0.2°

  _subPointcloud = _nh.subscribe(_laserDataTopic, 1, &PolarSensors::callbackPointcloud, this);
}
virtual PolarSensors::~PolarSensors() {}

void PolarSensors::callbackPointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {}