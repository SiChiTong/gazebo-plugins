#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include "modelControl.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelControl)

ModelControl::ModelControl()
{
}

void ModelControl::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelControl::OnUpdate, this, _1));

  //Init the communication Node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  printf("Try suscribing \n");
  this->headerSub = this->node->Subscribe(std::string("~/RobotinoSim/Message/"), &ModelControl::OnHeaderMsg, this);


  printf("ModelControl-Plugin sucessfully loaded \n");
}

// Called by the world update start event
void ModelControl::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  // Apply a small linear velocity to the model.
  this->model->SetLinearVel(math::Vector3(.1, 0, 0));
  this->model->SetAngularVel(math::Vector3(0, 0, 0.1));
}

void ModelControl::Reset()
{
}

void ModelControl::OnHeaderMsg(ConstHeaderPtr &msg)
{
  printf("Got Header Msg!!! ");
  printf(msg->str_id().c_str());
  printf("\n");
}
