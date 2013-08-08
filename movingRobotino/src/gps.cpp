/***************************************************************************
 *  gps.cpp - Provides ground Truth position
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
#include <math.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "gps.h"
#include "config.h"

using namespace gazebo;

Gps::Gps(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
Gps::~Gps()
{
}

void Gps::init()
{
  printf("Initialize Gps \n");
  last_sent_time_ = model->GetWorld()->GetSimTime().Double();
}

void Gps::createPublishers()
{
  this->gpsPub = this->node->Advertise<msgs::Vector3d>("~/RobotinoSim/Gps/");
}

void Gps::createSubscribers()
{
}

void Gps::update()
{
  //Send position information to Fawkes
  double time = model->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > (1.0 / 5.0))
  {
    last_sent_time_ = time;
    sendPosition();
  }
}

void Gps::sendPosition()
{
  if(gpsPub->HasConnections())
  {
    //Read position and orientation from simulation
    float x = this->model->GetWorldPose().pos.x;
    float y = this->model->GetWorldPose().pos.y;
    float ori = this->model->GetWorldPose().rot.GetAsEuler().z;

    //build message
    msgs::Vector3d posMsg;
    //x,y,ori
    posMsg.set_x(x);
    posMsg.set_y(y);
    posMsg.set_z(ori);

    //send
    gpsPub->Publish(posMsg);
  }
}
