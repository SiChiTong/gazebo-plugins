#ifndef __GAZEBO_DEVICE_H_
#define __GAZEBO_DEVICE_H_


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
    SimDevice(physics::ModelPtr, transport::NodePtr);

    //common functions
    virtual void init();
    virtual void createPublishers();
    virtual void createSubscribers();
    virtual void update();


  protected:
    //Gazebo-objects needed by every device
    // Pointer to the model
    physics::ModelPtr model;
    //Node for communication
    transport::NodePtr node;
  };
}
#endif
