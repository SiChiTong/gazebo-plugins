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
   This class provides ground-truth information about the robots position and orientation
 */
  class Gps: public SimDevice
  {
  public:

    //Constructor
    Gps(physics::ModelPtr, transport::NodePtr);
    //Destructor
    ~Gps();
  
    virtual void init();
    virtual void createPublishers();
    virtual void createSubscribers();
    virtual void update();

  private:

    //Functions for sending ionformation to fawkes:
    void sendPosition();

    //Publisher for GyroAngle
    transport::PublisherPtr gpsPub;

    //Helper Variables
    float x, y, ori;
  };
}
