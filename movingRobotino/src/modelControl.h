#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include <list>
#include "device.h"

namespace gazebo
{   
  class ModelControl : public ModelPlugin
  {
  public:
    //Constructor
    ModelControl();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  private:

    //simulated devices
    std::list<SimDevice*> devices_list;
    
    //Functions for recieving Messages (registerd via suscribers)
    void OnMotorMoveMsg(ConstVector3dPtr &msg);

    //Functions for sending ionformation to fawkes:
    void sendGyro();

    // Pointer to the model
    physics::ModelPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
    //Node for communication
    transport::NodePtr node;
    //Suscriber for MotorMove Interfaces from Fawkes
    transport::SubscriberPtr motorMoveSub;
    //Publisher for GyroAngle
    transport::PublisherPtr gyroPub;
    

    //current movement commands:
    float vx, vy, vomega;

  };
}