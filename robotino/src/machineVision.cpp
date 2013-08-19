/***************************************************************************
 *  machineVision.cpp - provides ground truth about 
 *                    the nearest machine light signals
 *
 *  Created: Sat Aug 17 23:24:45 2013
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
#include "simDevice.h"
#include "machineVision.h"
#include "../../llsf/src/data_table.h"


using namespace gazebo;

MachineVision::MachineVision(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
MachineVision::~MachineVision()
{
}

void MachineVision::init()
{
  printf("Initialize MachineVision \n");
  table_ = LlsfDataTable::get_table();
}

void MachineVision::create_publishers()
{
  this->light_signal_pub_ = this->node->Advertise<msgs::Vector3d>("~/RobotinoSim/MachineVision/");
}

void MachineVision::create_subscribers()
{
  //no subscribers
}

void MachineVision::update()
{
  send_light_results();
}

/*void MachineVision::send_light_results()
{
  //just send ground truth machine light signals with position to fawkes
  //the fawkes plugin takes care of the selection of the right machine in front of the robotino
  
  
  }*/
