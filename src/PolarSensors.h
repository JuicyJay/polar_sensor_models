/*
 * PolarSensors.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Jasmin Ziegler
 */

#include "obvision/reconstruct/space/SensorPolar.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > stdVecEig3d;
class PolarSensors
{
public:
  PolarSensors(const float dimX, const float dimY, const float dimZ, const float cellSize);
  virtual ~PolarSensors();
  void callbackPointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

private:
  void init(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void sortPointcloudTilt4d(const pcl::PointCloud<pcl::PointXYZ>* cloud, pcl::PointCloud<pcl::PointXYZ>* sortedCloud);
  void pubAxisAlignedRaycaster(void);
  void redBlueRenderSpace(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
  void pubSensorRaycast(pcl::PointCloud<pcl::PointXYZ>& cloud);

  std::unique_ptr<obvious::TsdSpace>    _space;
  std::unique_ptr<obvious::SensorPolar> _sensor;

  ros::Subscriber _subPointcloud;
  ros::Publisher  _pubAxisAlignedCloud;
  ros::Publisher  _pubRedBlueRendered;
  ros::Publisher  _pubSensorRaycastCloud;

  ros::NodeHandle                        _nh;
  ros::NodeHandle                        _prvnh;
  std::unique_ptr<tf::TransformListener> _listener;
  std::string                            _tfMyFrameID;
  std::string                            _tfCallbackFrameID;

  int              _raysIncl;
  std::string      _sensorType;
  bool             _artificialData;
  std::string      _laserDataTopic;
  double           _inclMin;
  double           _inclMax;
  double           _inclRes;
  double           _azimMin;
  double           _azimMax;
  double           _azimRes;
  std::vector<int> _firingSeq; // firingSeq order in which vertical lasers are fired (see SensorPolar class Constructor+lookupIndex methods for more
  // details), is by default empty. If not empty, it will trigger a call to lookupIndex in function backProject() of C SensorPolar
  // firingSeq for Velodyne VLP16: firingSeq={0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15}; (check VLP16 User Manual p. 54ff)
  // note: since the firing sequence (scan order) of the VLP16 does not increment evenly from inclMin to inclMax in inclRes steps, the lookupIndex
  // method is necessary to match the scan order to the raycast order
  // firingSeq for Velodyne HDL-32E: firingSeq={0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,  1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25,
  // 27, 29, 31};

  float        _dimX;
  float        _dimY;
  float        _dimZ;
  float        _cellSize;
  unsigned int _cellsX;
  unsigned int _cellsY;
  unsigned int _cellsZ;
  bool         _tfActive;   // deactivate tf frame shifts for debugging reasons
  bool         _virginPush; // only pushes first pointcloud that arrives in callback and performs raycasts afterwards. rm this for entire SLAM
};