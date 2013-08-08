/***************************************************************************
 *  laserSensor.cpp - Provides laser sensor data
 *
 *  Created: Wed Aug 07 18:22:45 2013
 *  Copyright  2013  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <math.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "laserSensor.h"

using namespace gazebo;

LaserSensor::LaserSensor(physics::ModelPtr model, transport::NodePtr node, sensors::SensorPtr sensorPtr)
 : SimDevice(model, node)
{
  this->parentSensor = boost::dynamic_pointer_cast<sensors::RaySensor>(sensorPtr);
}
LaserSensor::~LaserSensor()
{
}

void LaserSensor::init()
{
  printf("Initialize LaserSensor \n");

  //Register OnNewLaserScans function
  this->newLaserScansConnection = this->parentSensor->GetLaserShape()->ConnectNewLaserScans(boost::bind(&LaserSensor::OnNewLaserScans, this));
}

void LaserSensor::createPublishers()
{
  this->laserPub = this->node->Advertise<msgs::Vector3d>("~/RobotinoSim/LaserSensor/");
}

void LaserSensor::createSubscribers()
{
}

void LaserSensor::update()
{
}

void LaserSensor::OnNewLaserScans()
{
  printf("Got Laser Data :D\n");
}
