/***************************************************************************
 *  robotino.cpp - Main Plugin file for controlling the robotino model
 *
 *  Created: Mon Jul 29 17:33:31 2013
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
#include <transport/transport.hh>
#include <math.h>
#include <sensors/sensors.hh>
#include <sensors/SensorTypes.hh>
#include <sensors/RaySensor.hh>

#include "robotino.h"
#include "simDevice.h"
#include "messageDisplay.h"
#include "gyro.h"
#include "motor.h"
#include "gps.h"
#include "laserSensor.h"
#include "machineVision.h"
#include "puck_detection.h"
#include "infraredPuckSensor.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Robotino)

Robotino::Robotino()
{
}

Robotino::~Robotino()
{
  printf("Destructing Robotino Plugin!\n");
  //Destruct all simulated devices
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    delete *it;
  }
}

void Robotino::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Robotino::OnUpdate, this, _1));

  //Init the communication Node
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init();
  
  //creating simulated devices
  devices_list_.push_back((SimDevice*) new MessageDisplay(model_, node_));
  devices_list_.push_back((SimDevice*) new Gyro(model_, node_));
  devices_list_.push_back((SimDevice*) new Motor(model_, node_));
  devices_list_.push_back((SimDevice*) new Gps(model_, node_));
  devices_list_.push_back((SimDevice*) new LaserSensor(model_, node_, sensors::get_sensor("laser")));
  devices_list_.push_back((SimDevice*) new MachineVision(model_, node_));
  devices_list_.push_back((SimDevice*) new PuckDetection(model_, node_));
  devices_list_.push_back((SimDevice*) new InfraredPuckSensor(model_, node_, sensors::get_sensor("infrared_puck_sensor")));

  //initialize and publish messages of devices (before subscribing to avoid deadlocks)
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    (*it)->init();
    (*it)->create_publishers();
  }

  //suscribe messages of devices
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    (*it)->create_subscribers();
  }

  printf("Robotino-Plugin sucessfully loaded \n");
}

// Called by the world update start event
void Robotino::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //update devices
  for (std::list<SimDevice*>::iterator it = devices_list_.begin(); it != devices_list_.end(); it++)
  {
    (*it)->update();
  }
}

void Robotino::Reset()
{
}
