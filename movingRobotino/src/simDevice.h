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
