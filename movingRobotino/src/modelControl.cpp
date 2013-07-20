#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include <math.h>
#include "modelControl.h"
#include "device.h"
#include "messageDisplay.h"

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
  
  //creating devices
  devices_list.push_back((SimDevice*) new MessageDisplay(model, node));
  
  //initialize and publish messages of devices (before subscribing to avoid deadlocks)
  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->init();
    (*it)->createPublishers();
  }
  printf("Try Publishing \n");
  this->gyroPub = this->node->Advertise<msgs::Vector3d>("~/RobotinoSim/Gyro/");

  for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->createSubscribers();
  }

  printf("Try suscribing \n");
  this->motorMoveSub = this->node->Subscribe(std::string("~/RobotinoSim/MotorMove/"), &ModelControl::OnMotorMoveMsg, this);

  printf("ModelControl-Plugin sucessfully loaded \n");

  //initialize movement commands:
  vx = 0.0;
  vy = 0.0;
  vomega = 0.0;
}

// Called by the world update start event
void ModelControl::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //update devices
    for (std::list<SimDevice*>::iterator it = devices_list.begin(); it != devices_list.end(); it++)
  {
    (*it)->update();
  }

  //Send information to Fawkes
  sendGyro();

  //Apply movement command
  float x,y;
  float yaw = this->model->GetWorldPose().rot.GetAsEuler().z;
  //foward part
  x = cos(yaw) * vx;
  y = sin(yaw) * vx;
  //sideways part
  x += cos(yaw + 3.1415926f / 2) * vy;
  y += sin(yaw + 3.1415926f / 2) * vy;
  // Apply velocity to the model.
  this->model->SetLinearVel(math::Vector3(x, y, 0));
  this->model->SetAngularVel(math::Vector3(0, 0, vomega));
}

void ModelControl::Reset()
{
}

void ModelControl::OnMotorMoveMsg(ConstVector3dPtr &msg)
{
  printf("Got MotorMove Msg!!! %f %f %f\n", msg->x(), msg->y(), msg->z());
  //Transform relative motion into ablosulte motion
  vx = msg->x();
  vy = msg->y();
  vomega = msg->z();
}

void ModelControl::sendGyro()
{
  if(gyroPub->HasConnections())
  {
    //Read gyro from simulation
    float roll = this->model->GetWorldPose().rot.GetAsEuler().x;
    float pitch = this->model->GetWorldPose().rot.GetAsEuler().y;
    float yaw = this->model->GetWorldPose().rot.GetAsEuler().z;

    //build message
    msgs::Vector3d gyroMsg;
    gyroMsg.set_x(roll);
    gyroMsg.set_y(pitch);
    gyroMsg.set_z(yaw);

    //send
    gyroPub->Publish(gyroMsg);
  }
}
