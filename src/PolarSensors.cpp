/*
 * PolarSensors.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Jasmin Ziegler
 */

#include "PolarSensors.h"
#include "VelodyneVLP"

PolarSensors::PolarSensors(const float dimX, const float dimY, const float dimZ, const float cellSize)
    : _nh(), _prvnh("~"), _dimX(dimX), _dimY(dimY), _dimZ(dimZ), _cellSize(cellSize), /**_listener(std::make_unique<tf::TransformListener>()),**/ _cellsX(0),
      _cellsY(0), _cellsZ(0)
{
  // launchfile parameters, default vals for VLP16
  _prvnh.param<std::string>("sensorType", _sensorType, "VLP16");
  _prvnh.param<std::string>("laserDataTopic", _laserDataTopic, "puck_rear/velodyne_points");
  _prvnh.param<int>("raysIncl", _raysIncl, 16);
  _prvnh.param<double>("inclMin", _inclMin, -0.26180); //-15°
  _prvnh.param<double>("inclMax", _inclMax, 0.26180);  //+15°
  _prvnh.param<double>("inclRes", _inclRes, 0.034907); // 2°
  _prvnh.param<double>("azimMin", _azimMin, 0.0);
  _prvnh.param<double>("azimMax", _azimMax, 6.28319);    // 360°
  _prvnh.param<double>("azimRes", _azimRes, 0.00349066); // 0.2°

  std::cout << __PRETTY_FUNCTION__ << "launch check" << std::endl;
  std::cout << "raysIncl = " << _raysIncl << std::endl;
  std::cout << "inclMin = " << _inclMin << std::endl;
  std::cout << "inclMax = " << _inclMax << std::endl;
  std::cout << "inclRes = " << _inclRes << std::endl;
  std::cout << "azimMin = " << _azimMin << std::endl;
  std::cout << "azimMax = " << _azimMax << std::endl;
  std::cout << "azimRes = " << _azimRes << std::endl;

  _subPointcloud = _nh.subscribe(_laserDataTopic, 1, &PolarSensors::callbackPointcloud, this);
}
virtual PolarSensors::~PolarSensors() {}

void PolarSensors::init(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if(_sensor || _space)
    return;

  // 1. INITIALIZE SPACE
  unsigned int cellsX = static_cast<unsigned int>(std::round(_dimX / _cellSize));
  unsigned int cellsY = static_cast<unsigned int>(std::round(_dimY / _cellSize));
  unsigned int cellsZ = static_cast<unsigned int>(std::round(_dimZ / _cellSize));

  const unsigned int xFac = cellsX / 64;
  const unsigned int yFac = cellsY / 64;
  const unsigned int zFac = cellsZ / 64;

  _cellsX = xFac * 64;
  _cellsY = yFac * 64;
  _cellsZ = zFac * 64;

  _space = std::make_unique<obvious::TsdSpace>(_cellSize, obvious::LAYOUT_64x64x64, _cellsX, _cellsY, _cellsZ);
  _space->setMaxTruncation(3.0 * _cellSize);
  // initial sensor pose: middle of space
  double tr[3];
  _space->getCentroid(tr);
  double          tf[16] = {1, 0, 0, tr[0], 0, 1, 0, tr[1], 0, 0, 1, tr[2], 0, 0, 0, 1};
  obvious::Matrix Tinit(4, 4);
  Tinit.setIdentity();
  Tinit.setData(tf);

  // 2. INITIALIZE SENSOR
  if(_sensorType == 'VLP16')
  {
    _firingSeq = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15}; // todo: do this via launchfile & pushback into vec here. better: yaml file
    _sensor    = std::make_unique<obvious::SensorPolar>(_raysIncl, _inclMin, _inclMax, _inclRes, _azimMin, _azimMax, _azimRes, _firingSeq);
  }
  else if(_sensorType == 'HDL32E')
  {
    _firingSeq = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};
    _sensor    = std::make_unique<obvious::SensorPolar>(_raysIncl, _inclMin, _inclMax, _inclRes, _azimMin, _azimMax, _azimRes, _firingSeq);
  }
  else if((_sensorType == 'OUSTEROS0') || (_sensorType == 'TILT3D') || (_sensorType == 'TILT4D'))
  {
    _sensor = std::make_unique<obvious::SensorPolar>(_raysIncl, _inclMin, _inclMax, _inclRes, _azimMin, _azimMax, _azimRes);
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__
              << "invalid sensorType. Please check launchfile. Wanna add a new sensor? pls add it to polarsensors.launch and initialize it here."
              << std::endl;
  }

  _sensor->setTransformation(Tinit);
  std::cout << "Sensor pose after transforming to the middle of TSD space: " << std::endl;
  _sensor->getTransformation().print();
}

void PolarSensors::callbackPointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  // if(!_sensor || !_space) // if neither sensor nor space have been initialized yet -> do so in init routine
  if(!_space) // if space hasn't been initialized yet -> do so in init routine
    this->init(cloud);
}