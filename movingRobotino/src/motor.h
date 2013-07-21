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
  class Motor: public SimDevice
  {
  public:

    //Constructor
    Motor(physics::ModelPtr, transport::NodePtr);
    //Destructor
    ~Motor();
  
    virtual void init();
    virtual void createPublishers();
    virtual void createSubscribers();
    virtual void update();

  private:
        
    //Functions for recieving Messages (registerd via suscribers)
    void OnMotorMoveMsg(ConstVector3dPtr &msg);

    //Suscriber for MotorMove Interfaces from Fawkes
    transport::SubscriberPtr motorMoveSub;


    //current movement commands:
    float vx, vy, vomega;  
  };
}
