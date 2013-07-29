/***************************************************************************
 *  modelControl.cpp - Main Plugin file for controlling the robotino model
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
#include "modelControl.h"
#include "simDevice.h"
#include "messageDisplay.h"
#include "gyro.h"
#include "motor.h"
#include "gps.h"


using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelControl)

ModelControl::ModelControl()
{
}

ModelControl::~ModelControl()
{
  printf("Destructing ModelControl Plugin!\n");
  //Destruct all simulated devices
  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    delete *it;
  }
}

void ModelControl::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelControl::OnUpdate, this, _1));

  //Init the communication Node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  
  //creating simulated devices
  devices_list.push_back((SimDevice*) new MessageDisplay(model, node));
  devices_list.push_back((SimDevice*) new Gyro(model, node));
  devices_list.push_back((SimDevice*) new Motor(model, node));
  devices_list.push_back((SimDevice*) new Gps(model, node));
  
  //initialize and publish messages of devices (before subscribing to avoid deadlocks)
  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->init();
    (*it)->createPublishers();
  }

  //suscribe messages of devices
  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->createSubscribers();
  }

  printf("ModelControl-Plugin sucessfully loaded \n");
}

// Called by the world update start event
void ModelControl::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //update devices
  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->update();
  }
}

void ModelControl::Reset()
{
}
