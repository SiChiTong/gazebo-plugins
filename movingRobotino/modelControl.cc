#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include <math.h>
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
  this->stringSub = this->node->Subscribe(std::string("~/RobotinoSim/String/"), &ModelControl::OnStringMsg, this);
  this->motorMoveSub = this->node->Subscribe(std::string("~/RobotinoSim/MotorMove/"), &ModelControl::OnMotorMoveMsg, this);


  printf("ModelControl-Plugin sucessfully loaded \n");
}

// Called by the world update start event
void ModelControl::OnUpdate(const common::UpdateInfo & /*_info*/)
{
}

void ModelControl::Reset()
{
}

void ModelControl::OnStringMsg(ConstHeaderPtr &msg)
{
  printf("Got String Msg!!! ");
  printf(msg->str_id().c_str());
  printf("\n");
}

void ModelControl::OnMotorMoveMsg(ConstVector3dPtr &msg)
{
  printf("Got MotorMove Msg!!! %f %f %f\n", msg->x(), msg->y(), msg->z());
  //Transform relative motion into ablosulte motion
  float x, y, omega;
  omega = msg->z();
  float yaw = this->model->GetWorldPose().rot.GetAsEuler().z;
  //foward part
  x = cos(yaw) * msg->x();
  y = sin(yaw) * msg->x();
  //sideways part
  x += cos(yaw + 3.1415926f / 2) * msg->y();
  y += sin(yaw + 3.1415926f / 2) * msg->y();
  // Apply velocity to the model.
  this->model->SetLinearVel(math::Vector3(x * 10, y * 10, 0));
  this->model->SetAngularVel(math::Vector3(0, 0, omega));
}
