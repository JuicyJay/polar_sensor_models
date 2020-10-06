/*
 * PolarSensors.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Jasmin Ziegler
 */

#include "PolarSensors.h"
#include "obvision/reconstruct/space/RayCast3D.h"
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"

PolarSensors::PolarSensors(const float dimX, const float dimY, const float dimZ, const float cellSize)
    : _nh(), _prvnh("~"), _dimX(dimX), _dimY(dimY), _dimZ(dimZ), _cellSize(cellSize), /**_listener(std::make_unique<tf::TransformListener>()),**/ _cellsX(0),
      _cellsY(0), _cellsZ(0)
{
  // launchfile parameters, default vals for VLP16
  _prvnh.param<std::string>("sensorType", _sensorType, "VLP16");
  _prvnh.param<bool>("artificialData", _artificialData, "true");
  _prvnh.param<std::string>("laserDataTopic", _laserDataTopic, "puck_rear/velodyne_points");
  _prvnh.param<std::string>("tfMyFrameID", _tfMyFrameID, "map");
  _prvnh.param<std::string>("tfCallbackFrameID", _tfCallbackFrameID, "puck_rear");
  _prvnh.param<int>("raysIncl", _raysIncl, 16);
  _prvnh.param<double>("inclMin", _inclMin, -0.26180); //-15°
  _prvnh.param<double>("inclMax", _inclMax, 0.26180);  //+15°
  _prvnh.param<double>("inclRes", _inclRes, 0.034907); // 2°
  _prvnh.param<double>("azimMin", _azimMin, 0.0);
  _prvnh.param<double>("azimMax", _azimMax, 6.28319);    // 360°
  _prvnh.param<double>("azimRes", _azimRes, 0.00349066); // 0.2°

  std::cout << __PRETTY_FUNCTION__ << " LAUNCH CHECK " << std::endl;
  std::cout << "sensorType = " << _sensorType << std::endl;
  std::cout << "artificialData = " << _artificialData << std::endl;
  std::cout << "laserDataTopic = " << _laserDataTopic << std::endl;
  std::cout << "tfMyFrameID = " << _tfMyFrameID << std::endl;
  std::cout << "tfCallbackFrameID = " << _tfCallbackFrameID << std::endl;
  std::cout << "raysIncl = " << _raysIncl << std::endl;
  std::cout << "inclMin = " << _inclMin << " = " << RAD2DEG(_inclMin) << "°" << std::endl;
  std::cout << "inclMax = " << _inclMax << " = " << RAD2DEG(_inclMax) << "°" << std::endl;
  std::cout << "inclRes = " << _inclRes << " = " << RAD2DEG(_inclRes) << "°" << std::endl;
  std::cout << "azimMin = " << _azimMin << " = " << RAD2DEG(_azimMin) << "°" << std::endl;
  std::cout << "azimMax = " << _azimMax << " = " << RAD2DEG(_azimMax) << "°" << std::endl;
  std::cout << "azimRes = " << _azimRes << " = " << RAD2DEG(_azimRes) << "°" << std::endl;

  _subPointcloud         = _nh.subscribe(_laserDataTopic, 1, &PolarSensors::callbackPointcloud, this);
  _pubAxisAlignedCloud   = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal> >("axisAlignedCloud", 1);
  _pubRedBlueRendered    = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("redBlueRendered_space", 1);
  _pubSensorRaycastCloud = _nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("sensorRaycastCloud", 1);
  _tfActive              = false;
  _virginPush            = false;
}

PolarSensors::~PolarSensors() {}

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
  _cellsX                 = xFac * 64;
  _cellsY                 = yFac * 64;
  _cellsZ                 = zFac * 64;

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
  if(_sensorType == "VLP16")
  {
    _firingSeq = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};
    _sensor    = std::make_unique<obvious::SensorPolar>(_raysIncl, _inclMin, _inclMax, _inclRes, _azimMin, _azimMax, _azimRes, _firingSeq);
  }
  else if(_sensorType == "HDL32E")
  {
    _firingSeq = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};
    _sensor    = std::make_unique<obvious::SensorPolar>(_raysIncl, _inclMin, _inclMax, _inclRes, _azimMin, _azimMax, _azimRes, _firingSeq);
  }
  else if((_sensorType == "OUSTEROS0") || (_sensorType == "TILT3D") || (_sensorType == "TILT4D"))
  {
    _sensor = std::make_unique<obvious::SensorPolar>(_raysIncl, _inclMin, _inclMax, _inclRes, _azimMin, _azimMax, _azimRes);
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__
              << "invalid sensorType. Please check launchfile. Wanna add a new sensor? pls add it to polarsensors.launch and "
                 "initialize it here. More detailed info on how to add a new sensor in polarsensors.launch"
              << std::endl;
  }

  _sensor->setTransformation(Tinit);
  std::cout << "Sensor pose after transforming to the middle of TSD space: " << std::endl;
  _sensor->getTransformation().print();
}

void PolarSensors::sortPointcloudTilt4d(const pcl::PointCloud<pcl::PointXYZ>* cloud, pcl::PointCloud<pcl::PointXYZ>* sortedCloud)
{
  // sort incoming incl rays in vector of vectors (buckets) - one bucket for each inclination angle
  // store new pointcloud in sortedCloud so it can be accessed from callbackPointcloud for calculating depth values and setting vector depthData
}

void PolarSensors::pubAxisAlignedRaycaster(void)
{
  static obvious::obfloat*      coords  = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static obvious::obfloat*      normals = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned char*         rgb     = new unsigned char[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned int           seq     = 0;
  obvious::RayCastAxisAligned3D raycasterAxisAligned;
  unsigned int                  cnt = 0;

  raycasterAxisAligned.calcCoords(_space.get(), coords, normals, rgb, &cnt);

  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  obvious::obfloat                        tr[3];
  _space->getCentroid(tr);

  for(unsigned int i = 0; i < cnt; i += 3)
  {
    pcl::PointXYZRGBNormal p;
    p.x = coords[i] - tr[0];
    p.y = coords[i + 1] - tr[1];
    p.z = coords[i + 2] - tr[2];

    p.normal_x = normals[i] - tr[0];
    p.normal_y = normals[i + 1] - tr[1];
    p.normal_z = normals[i + 2] - tr[2];

    p.r = rgb[i];
    p.g = rgb[i + 1];
    p.b = rgb[i + 2];

    cloud.push_back(p);
  }

  cloud.header.frame_id = _tfMyFrameID;
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
  _pubAxisAlignedCloud.publish(cloud);
}

void PolarSensors::redBlueRenderSpace(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  static unsigned int seq = 0;
  struct Color
  {
    Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
    Color() : r(0), g(0), b(0) {}
    void    red(uint8_t val) { r = val; }
    void    blue(uint8_t val) { b = val; }
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  obvious::Matrix*   cellCoordsHom = obvious::TsdSpacePartition::getCellCoordsHom();
  obvious::Matrix*   partCoords    = obvious::TsdSpacePartition::getPartitionCoords();
  unsigned int       partSize      = (_space->getPartitions())[0][0][0]->getSize();
  stdVecEig3d        centers;
  std::vector<Color> colors;
  obvious::obfloat   tr[3];
  _space->getCentroid(tr);
  for(unsigned int pz = 0; pz < _space->getPartitionsInZ(); pz++)
  {
    for(unsigned int py = 0; py < _space->getPartitionsInY(); py++)
    {
      for(unsigned int px = 0; px < _space->getPartitionsInX(); px++)
      {
        obvious::TsdSpacePartition* part = _space->getPartitions()[pz][py][px];
        if(part->isInitialized() && !part->isEmpty())
        {
          obvious::obfloat t[3];
          part->getCellCoordsOffset(t);
          for(unsigned int c = 0; c < partSize; c++)
          {
            Eigen::Vector3d center;
            center(0) = (*cellCoordsHom)(c, 0) + t[0];
            center(1) = (*cellCoordsHom)(c, 1) + t[1];
            center(2) = (*cellCoordsHom)(c, 2) + t[2];
            // if(center(1) > _space->getMaxY() / 2.0)
            //   continue;
            obvious::obfloat tsd = part->getTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2));
            if((isnan(tsd)) || (tsd > std::abs(1.1)))
              continue;

            Color tsdColor; //(0, 0, 0);
            if(tsd < 0.0)   // red
              tsdColor.red(static_cast<unsigned char>(-1.0 * tsd * 255.0));
            else
              tsdColor.blue(static_cast<unsigned char>(tsd * 255.0));

            centers.push_back(center);
            colors.push_back(tsdColor);
          }
        }
      }
    }
  }
  if((centers.size() == colors.size()) && (centers.size() != 0))
  {
    for(unsigned int i = 0; i < centers.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x = centers[i].x() - tr[0];
      p.y = centers[i].y() - tr[1];
      p.z = centers[i].z() - tr[2];
      p.r = colors[i].r;
      p.g = colors[i].g;
      p.b = colors[i].b;
      cloud.push_back(p);
    }
  }
  else
    std::cout << __PRETTY_FUNCTION__ << "nuttingham found " << centers.size() << " " << colors.size() << std::endl;
  cloud.header.frame_id = _tfMyFrameID;
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
}

void PolarSensors::pubSensorRaycast(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  static obvious::obfloat* coords  = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static obvious::obfloat* normals = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned char*    rgb     = new unsigned char[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned int      seq     = 0;

  unsigned int width  = _sensor->getWidth();
  unsigned int height = _sensor->getHeight();
  unsigned int size   = 0;

  obvious::RayCast3D raycasterSensor;
  raycasterSensor.calcCoordsFromCurrentPose(_space.get(), _sensor.get(), coords, normals, rgb, &size);
  obvious::obfloat tr[3];
  _space->getCentroid(tr);

  for(unsigned int i = 0; i < size; i += 3)
  {
    pcl::PointXYZ p;
    p.x = coords[i] - tr[0];
    p.y = coords[i + 1] - tr[1];
    p.z = coords[i + 2] - tr[2];
    cloud.push_back(p);
  }

  cloud.header.frame_id = _tfMyFrameID;
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
}

void PolarSensors::callbackPointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if(!_sensor || !_space) // if neither sensor nor space have been initialized yet -> do so in init routine
                          // if(!_space) // if space hasn't been initialized yet -> do so in init routine
    this->init(cloud);

  if(_tfActive) // transform sensor
  {
    tf::StampedTransform tfSensor;
    // look up transform from pointcloud frame_id to _tfMyFrameID=map for my sensor to transform sensor!
    try
    {
      _listener->waitForTransform(_tfCallbackFrameID, _tfMyFrameID, ros::Time(0), ros::Duration(10.0));
      _listener->lookupTransform(_tfCallbackFrameID, _tfMyFrameID, ros::Time(0), tfSensor);
    }
    catch(const tf::TransformException& e)
    {
      std::cout << __PRETTY_FUNCTION__ << "e: " << e.what() << std::endl;
      return;
    }
    std::cout << __PRETTY_FUNCTION__ << "tfSensorBefore:  translation: " << tfSensor.getOrigin().getX() << " , " << tfSensor.getOrigin().getY() << " , "
              << tfSensor.getOrigin().getZ() << std::endl;

    tf::Vector3    tfVec = tfSensor.getOrigin();
    tf::Quaternion quat  = tfSensor.getRotation();
    tf::Matrix3x3  RotMat(quat);
    double         roll, pitch, yaw = 0.0;
    RotMat.getRPY(roll, pitch, yaw);

    // transform sensor
    obvious::Matrix TransMat(4, 4);
    TransMat.setIdentity();
    obvious::obfloat center[3];
    _space->getCentroid(center);
    TransMat =
        obvious::MatrixFactory::TransformationMatrix44(yaw, pitch, roll, center[0] + tfVec.getX(), center[1] + tfVec.getY(), center[2] + tfVec.getZ());
    _sensor->setTransformation(TransMat);
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::cout << "Current Transformation of sensor (listened to tf + transformed): " << std::endl;
    _sensor->getTransformation().print();
  }

  if(!_virginPush) // is set to false in constructor. will be set to true after first push
  {
    std::vector<double> depthData(cloud.height * cloud.width, 0.0);
    bool*               mask = new bool[cloud.height * cloud.width];

    std::cout << "Pointcloud height = " << cloud.height << " , width = " << cloud.width << std::endl;

    unsigned int valid = 0;

    for(unsigned int i = 0; i < cloud.width; i++)
    {
      for(unsigned int j = 0; j < cloud.height; j++)
      {
        const unsigned int idx = i * cloud.height + j;

        Eigen::Vector3f point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);
        double          abs;

        if(_artificialData)
        {
          abs = static_cast<double>(point.norm());
        }
        else // real laser data
        {
          if(_sensorType == "TILT3D")
          {
            // probably not necessary since tilt3d software looks like scans are already collected in the correct order
            // since tilt3d works with SICK TIM571 which is a 2D LIDAR, there won't be multiple measurements per inclination ray
            // just delete this statement so that for TILT3D also the "else" condition applies
            // if sorting is needed: add a method for sorting like in tilt4d
          }
          else if(_sensorType == "TILT4D")
          {
            // problem here: tilt4d tilts the 16 vertical rays of VLP16 and this results in more measurements per inclination position/ray
            // implement a sort function that sorts each measurement in a bucket, compute average of each bucket so the result is one single measurement
            // per inclination ray. The total number of inclination depth measurements must not exceed the number of inclination rays which can be determined
            // by calulating |inclMin - inclMax| / inclRes

            pcl::PointCloud<pcl::PointXYZ> sortedCloud;
            this->sortPointcloudTilt4d(&cloud, &sortedCloud);
            Eigen::Vector3f sortedPoint(sortedCloud.points[idx].x, sortedCloud.points[idx].y, sortedCloud.points[idx].z);
            abs = static_cast<double>(sortedPoint.norm());
          }
          // else if(_sensorType == "OUSTEROS0")
          // {
          //   // call sort function if needed. Ouster tech support said that each measurement carries a ring parameter which specifies the vertical index
          //   // check out in which order data is sorted and then push measurement rays in order from inclMin = -45° to inclMax = 45° into vector depthData
          //   // in this way, no lookupIndex function is needed
          // }
          else // fill in depthData in order of scan pointcloud. always done for artificial data. this can also always be done for VLP16 and HDL32E because
               // SensorPolar implements a lookupIndex function for these two which takes care of the sorting with help of the firing sequence specified in
               // _firingSeq
          {
            abs = static_cast<double>(point.norm());
          }
        }

        // continue for all cases
        if(abs > 0.0)
        {
          depthData[idx] = abs;
          // depthData[idx] = 2.0;

          mask[idx]      = true;
          valid++;
        }
        else
        {
          mask[idx] = false;
        }
      }
    }

    if(!valid)
    {
      std::cout << __PRETTY_FUNCTION__ << " no valid points with depth value > 0.0 in data " << std::endl;
      return;
    }
    std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points " << std::endl;
    _sensor->setRealMeasurementData(depthData.data());
    std::cout << __PRETTY_FUNCTION__ << " ouch " << std::endl;

    _sensor->setRealMeasurementMask(mask);
    std::cout << __PRETTY_FUNCTION__ << " uhh " << std::endl;

    delete mask;
    _space->push(_sensor.get());
    std::cout << __PRETTY_FUNCTION__ << " ayayay " << std::endl;

    _virginPush = true;
  }
  else
  {
    // CALL RAYCAST METHODS HERE
    this->pubAxisAlignedRaycaster();

    pcl::PointCloud<pcl::PointXYZRGB> redBlueRenderedCloud;
    this->redBlueRenderSpace(redBlueRenderedCloud);
    _pubRedBlueRendered.publish(redBlueRenderedCloud);

    pcl::PointCloud<pcl::PointXYZ> sensorRaycastCloud;
    this->pubSensorRaycast(sensorRaycastCloud);
    _pubSensorRaycastCloud.publish(sensorRaycastCloud);
  }
}