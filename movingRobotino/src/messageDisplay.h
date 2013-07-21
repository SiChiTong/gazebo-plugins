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
  class MessageDisplay: public SimDevice
  {
  public:

    //Constructor
    MessageDisplay(physics::ModelPtr, transport::NodePtr);
    //Destructor
    ~MessageDisplay();

    virtual void init();
    virtual void createPublishers();
    virtual void createSubscribers();
    virtual void update();

  private:
    //Suscriber for Messages from Fawkes
    transport::SubscriberPtr stringSub;
    
    //Functions for recieving Messages (registerd via suscribers)
    void onStringMsg(ConstHeaderPtr &msg);  
  };
}
