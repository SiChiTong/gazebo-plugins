/***************************************************************************
 *  data_table.cpp - This module stores all logical information
 *    about the llsf simulation (e.g. machine orientations,
 *    light signals, puck locations)
 *
 *  Created: Fri Aug 09 12:33:22 2013
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
#include <stddef.h>

#include "data_table.h"
#include "refbox_comm.h"


using namespace gazebo;

LlsfDataTable::LlsfDataTable(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  world_ = world;

  //read out machine positions and orientation from world
  init_table();

  //initialize refbox communication
  refbox_comm_ = new RefboxComm(this, gazebo_node);
}

LlsfDataTable::~LlsfDataTable()
{
}

LlsfDataTable* LlsfDataTable::table_ = NULL;

void LlsfDataTable::init(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  if(!table_)
  {
    table_ = new LlsfDataTable(world, gazebo_node);
  }
}

LlsfDataTable* LlsfDataTable::get_table()
{
  return table_;
}

void LlsfDataTable::finalize()
{
  if(table_)
  {
    delete table_;
  }
}

Machine LlsfDataTable::get_machine(MachineName name)
{
  return machines_[name];
}

Puck LlsfDataTable::get_puck(int number)
{
  return pucks_[number]; //assuming the pucks numbers start at 0
}

void LlsfDataTable::set_light_state(MachineName machine, LightState red, 
				    LightState yellow, LightState green)
{
  machines_[machine].red = red;
  machines_[machine].yellow = yellow;
  machines_[machine].green = green;
}

void LlsfDataTable::set_puck_pos(int puck, double x, double y)
{
  pucks_[puck].x = x;
  pucks_[puck].y = y;
}

void LlsfDataTable::set_puck_under_rfid(int puck, MachineName machine)
{
  pucks_[puck].under_rfid = machine;
  //inform refbox
  refbox_comm_->send_puck_placed_under_rfid(puck, machines_[machine]);
}

void LlsfDataTable::set_puck_in_machine_area(int puck, MachineName machine)
{
  pucks_[puck].in_machine_area = machine;
  //TODO: inform refbox if a puck leaves a machine area
}

void LlsfDataTable::init_table()
{
  init_machine(M1, "llsf_field::M1::machine_link");
  init_machine(M2, "llsf_field::M2::machine_link");
  init_machine(M3, "llsf_field::M3::machine_link");
  init_machine(M4, "llsf_field::M4::machine_link");
  init_machine(M5, "llsf_field::M5::machine_link");
  init_machine(M6, "llsf_field::M6::machine_link");
  init_machine(M7, "llsf_field::M7::machine_link");
  init_machine(M8, "llsf_field::M8::machine_link");
  init_machine(M9, "llsf_field::M9::machine_link");
  init_machine(M10, "llsf_field::M10::machine_link");
  init_machine(D1, "llsf_field::D1::machine_link");
  init_machine(D2, "llsf_field::D2::machine_link");
  init_machine(D3, "llsf_field::D3::machine_link");
  init_machine(R1, "llsf_field::R1::machine_link");
  init_machine(R2, "llsf_field::R2::machine_link");
  init_machine(T, "llsf_field::T::machine_link");

  init_puck(0, "llsf_field::Puck0::cylinder");
  init_puck(1, "llsf_field::Puck1::cylinder");
  init_puck(2, "llsf_field::Puck2::cylinder");
  init_puck(3, "llsf_field::Puck3::cylinder");
  init_puck(4, "llsf_field::Puck4::cylinder");
  init_puck(5, "llsf_field::Puck5::cylinder");
  init_puck(6, "llsf_field::Puck6::cylinder");
  init_puck(7, "llsf_field::Puck7::cylinder");
  init_puck(8, "llsf_field::Puck8::cylinder");
  init_puck(9, "llsf_field::Puck9::cylinder");
  /*
    CURRENTLY NOT IN THE SIMULATION
  init_puck(10, "llsf_field::Puck10::cylinder");
  init_puck(11, "llsf_field::Puck11::cylinder");
  init_puck(12, "llsf_field::Puck12::cylinder");
  init_puck(13, "llsf_field::Puck13::cylinder");
  init_puck(14, "llsf_field::Puck14::cylinder");
  init_puck(15, "llsf_field::Puck15::cylinder");
  init_puck(16, "llsf_field::Puck16::cylinder");
  init_puck(17, "llsf_field::Puck17::cylinder");
  init_puck(18, "llsf_field::Puck18::cylinder");
  init_puck(19, "llsf_field::Puck19::cylinder");*/
}

void LlsfDataTable::init_machine(MachineName number, std::string name)
{
  machines_[number].name = number;
  machines_[number].name_as_string = name;
  machines_[number].x = world_->GetEntity(name)->GetWorldPose().pos.x;
  machines_[number].y = world_->GetEntity(name)->GetWorldPose().pos.y;
  machines_[number].ori = world_->GetEntity(name)->GetWorldPose().rot.GetAsEuler().z;
  machines_[number].red = OFF;
  machines_[number].yellow = OFF;
  machines_[number].green = OFF;
}

void LlsfDataTable::init_puck(int number, std::string name)
{
  pucks_[number].number = number;
  pucks_[number].x = world_->GetEntity(name)->GetWorldPose().pos.x;
  pucks_[number].y = world_->GetEntity(name)->GetWorldPose().pos.y;
  pucks_[number].under_rfid = NONE;
  pucks_[number].in_machine_area = NONE;
}
