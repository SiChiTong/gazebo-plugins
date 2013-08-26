/***************************************************************************
 *  light_control.cpp - Module to control the Machine Lights in the visualization
 *
 *  Created: Mon Aug 26 11:28:42 2013
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
#include <math.h>
#include <transport/transport.hh>
#include <transport/Node.hh>
#include <msgs/msgs.hh>
#include <stdio.h>
#include <common/common.hh>

#include "light_control.h"


using namespace gazebo;

LightControl::LightControl(physics::WorldPtr world)
{
  //create node and initialize it (init is mandatory here!)
  node_ = transport::NodePtr(new transport::Node());
  world_ = world;
  node_->Init(world_->GetName().c_str());
  visPub_ = this->node_->Advertise<msgs::Visual>("~/visual", 10);
}
LightControl::~LightControl()
{
  node_->Fini();
  visPub_.reset();
}

void LightControl::update()
{  
  /* Visuals in Gazebo can be modified by publishing a Visual msg
     on the ~/visual topic (node has to be inited with the worlds name) 
     However the messages do not override visuals defined in the sdf. */
  double time = world_->GetSimTime().Double();
  if(time - last_sent_time_ < 0.5)
  {
    return;
  }
  
  last_sent_time_ = time;
    
  Machine m1 = LlsfDataTable::get_table()->get_machine(M1);
  Machine m2 = LlsfDataTable::get_table()->get_machine(M2);
  Machine m3 = LlsfDataTable::get_table()->get_machine(M3);
  Machine m4 = LlsfDataTable::get_table()->get_machine(M4);
    
  if(!visPub_->HasConnections())
  {
    printf("light_control: visual publisher not connected!\n");
  }
  else
  {
    visPub_->Publish(create_vis_msg(m1.name_as_string, RED, ON));
    visPub_->Publish(create_vis_msg(m2.name_as_string, YELLOW, ON));
    visPub_->Publish(create_vis_msg(m3.name_as_string, GREEN, ON));
    visPub_->Publish(create_vis_msg(m4.name_as_string, RED, OFF));
  }
}

//creates all needed visual messages
msgs::Visual LightControl::create_vis_msg(std::string machine_name, Color color, LightState state)
{
  //create message to return
  msgs::Visual msg;
  //resolve BLINK
  
  //common parameters
  msg.set_parent_name(machine_name.c_str());
  msgs::Geometry *geomMsg = msg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::CYLINDER);
  geomMsg->mutable_cylinder()->set_radius(0.02);
  geomMsg->mutable_cylinder()->set_length(0.034);
  msg.set_cast_shadows(false);

  //parameters dependent of color and state
  if(state == ON)
  {
    msg.set_transparency(0.0);
    msg.set_visible(true);
  }
  else
  {
    msg.set_visible(false);
  }

  switch(color)
  {
  case RED:
    {
      msg.set_name(machine_name.append("red").c_str());
      msgs::Set(msg.mutable_pose(), math::Pose(0.02, 0.0, 0.253, 0, 0, 0));
	msgs::Set(msg.mutable_material()->mutable_diffuse(), common::Color(0.8, 0, 0, 0.8));
	msgs::Set(msg.mutable_material()->mutable_emissive(), common::Color(1.0, 0.3, 0.3, 1.0));
      break;
    }
  case YELLOW:
    {
      msg.set_name(machine_name.append("yellow").c_str());
      msgs::Set(msg.mutable_pose(), math::Pose(0.02, 0.0, 0.219, 0, 0, 0));
	msgs::Set(msg.mutable_material()->mutable_diffuse(), common::Color(0.9, 0.7, 0, 0.8));
	msgs::Set(msg.mutable_material()->mutable_emissive(), common::Color(1.0, 0.9, 0.3, 1.0));
      break;
    }
  case GREEN:
    {
      msg.set_name(machine_name.append("green").c_str());
      msgs::Set(msg.mutable_pose(), math::Pose(0.02, 0.0, 0.185, 0, 0, 0));
	msgs::Set(msg.mutable_material()->mutable_diffuse(), common::Color(0, 0.8, 0, 0.8));
	msgs::Set(msg.mutable_material()->mutable_emissive(), common::Color(0.3, 1.0, 0.3, 1.0));
      break;
    }
    }
  return msg;
}
