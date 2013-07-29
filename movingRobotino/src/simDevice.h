/***************************************************************************
 *  simDevice.h - general superclass for simulated devices
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

#ifndef __GAZEBO_SIM_DEVICE_H_
#define __GAZEBO_SIM_DEVICE_H_


#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>

namespace gazebo
{
   
  class SimDevice
  {
  public:
    //Constructor
    SimDevice(physics::ModelPtr model, transport::NodePtr node);
    //Destructor
    virtual ~SimDevice();
    //common functions
    virtual void init() = 0;
    virtual void createPublishers() = 0;
    virtual void createSubscribers() = 0;
    virtual void update() = 0;


  protected:
    //Gazebo-objects needed by every device
    // Pointer to the model
    physics::ModelPtr model;
    //Node for communication
    transport::NodePtr node;
  };
}
#endif
