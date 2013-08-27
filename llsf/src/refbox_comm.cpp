/***************************************************************************
 *  refbox_comm.cpp - responsable for the communication with the refbox
 *                  reads/writes the data in the table
 *
 *  Created: Wed Aug 14 21:25:34 2013
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
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>

#include "refbox_comm.h"
#include "data_table.h"

#include "../../libs/msgs/MachineCommands.pb.h"
#include "../../libs/msgs/MachineInfo.pb.h"

using namespace gazebo;

RefboxComm::RefboxComm(LlsfDataTable *table, transport::NodePtr gazebo_node)
{
  table_ = table;
  gazebo_node_ = gazebo_node;

  //create publisher
  this->place_puck_under_machine_pub_ = gazebo_node_->Advertise<llsf_msgs::PlacePuckUnderMachine>("~/LLSFRbSim/PlacePuckUnderMachine/");
  this->remove_puck_from_machine_pub_ = gazebo_node_->Advertise<llsf_msgs::RemovePuckFromMachine>("~/LLSFRbSim/RemovePuckFromMachine/");

  //create subscriber
  this->machine_info_sub_ = gazebo_node_->Subscribe(std::string("~/LLSFRbSim/MachineInfo/"), &RefboxComm::on_machine_info_msg, this);
}

RefboxComm::~RefboxComm()
{
}

void RefboxComm::send_puck_placed_under_rfid(int puck, Machine & machine)
{
  //create the message
  llsf_msgs::PlacePuckUnderMachine ppum;
  ppum.set_puck_id(puck);
  ppum.set_machine_name(machine.name_as_string);

  //publish
  place_puck_under_machine_pub_->Publish(ppum);
}

void RefboxComm::send_remove_puck_from_machine(int puck, Machine & machine)
{
  //create the message
  llsf_msgs::RemovePuckFromMachine rpfm;
  rpfm.set_puck_id(puck);
  rpfm.set_machine_name(machine.name_as_string);

  //publish
  remove_puck_from_machine_pub_->Publish(rpfm);
}

void RefboxComm::on_machine_info_msg(ConstMachineInfoPtr &msg)
{
  printf("Got MachineInfo :D\n");
  
  //read all machines and set light signals
  for(int i = 0; i < msg->machines_size(); i++)
  {
    llsf_msgs::Machine machine = msg->machines(i);
    LightState red, yellow, green;
    for(int j = 0; j < machine.lights_size(); j++)
    {
      llsf_msgs::LightSpec light_spec = machine.lights(j);
      LightState state = OFF;
      switch(light_spec.state())
      {
      case llsf_msgs::OFF: state = OFF; break;
      case llsf_msgs::ON: state = ON; break;
      case llsf_msgs::BLINK: state = BLINK; break;
      }
      switch(light_spec.color())
      {
      case llsf_msgs::RED: red = state; break;
      case llsf_msgs::YELLOW: yellow = state; break;
      case llsf_msgs::GREEN: green = state; break;
      }
      table_->set_light_state(machine.name(), red, yellow, green);
    }
  }
}
