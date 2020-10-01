/*
 * main.cpp
 *
 *  Created on: Seo 29, 2020
 *      Author: Jasmin Ziegler
 */

#include "PolarSensors.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "polar_sensor_models_node");
  PolarSensors model(10.0, 10.0, 10.0, 0.025);
  ros::spin();
}