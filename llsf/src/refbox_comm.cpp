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

#include "msgs/MachineCommands.pb.h"
#include "msgs/MachineInfo.pb.h"

using namespace gazebo;

RefboxComm::RefboxComm(LlsfDataTable *table, transport::NodePtr gazebo_node)
{
  table_ = table;
  gazebo_node_ = gazebo_node;

  //create publisher
  this->place_puck_under_machine_pub_ = gazebo_node_->Advertise<llsf_msgs::PlacePuckUnderMachine>("~/LLSF-RefboxComm/PlacePuckUnderMachine/");

  //create subscriber
  //this->machine_info_sub_ = gazebo_node_->Subscribe(std::string("~/LLSF-RefboxComm/MachineInfo/"), &RefboxComm::on_machine_info_msg, this);
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

/*void RefboxComm::on_machine_info_msg(ConstMachineInfoPtr &msg)
{

}*/
