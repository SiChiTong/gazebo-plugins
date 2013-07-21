#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include <math.h>
#include "modelControl.h"
#include "simDevice.h"
#include "messageDisplay.h"
#include "gyro.h"
#include "motor.h"


using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelControl)

ModelControl::ModelControl()
{
  //printf("Constructor ModelControl \n");
}

void ModelControl::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  //printf("Loading ModelControl \n");
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelControl::OnUpdate, this, _1));

  //Init the communication Node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  
  //creating devices
  devices_list.push_back((SimDevice*) new MessageDisplay(model, node));
  devices_list.push_back((SimDevice*) new Gyro(model, node));
  devices_list.push_back((SimDevice*) new Motor(model, node));
  
  //initialize and publish messages of devices (before subscribing to avoid deadlocks)
  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->init();
    (*it)->createPublishers();
  }

  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->createSubscribers();
  }

  printf("ModelControl-Plugin sucessfully loaded \n");
}

// Called by the world update start event
void ModelControl::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //update devices
  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->update();
  }
}

void ModelControl::Reset()
{
}
