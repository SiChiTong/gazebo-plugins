#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include "simDevice.h"

namespace gazebo
{
/*
   This class displayes messages form Fawkes in the Gazebo console
 */
  class Gyro: public SimDevice
  {
  public:

    //Constructor
    Gyro(physics::ModelPtr, transport::NodePtr);
    //Destructor
    ~Gyro();
  
    virtual void init();
    virtual void createPublishers();
    virtual void createSubscribers();
    virtual void update();

  private:

    //Functions for sending ionformation to fawkes:
    void sendGyro();

    //Publisher for GyroAngle
    transport::PublisherPtr gyroPub;  
  };
}
