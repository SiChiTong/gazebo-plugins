#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "motor.h"

using namespace gazebo;

Motor::Motor(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}

void Motor::init()
{
  printf("Initialize Motor\n");
  //initialize movement commands:
  vx = 0.0;
  vy = 0.0;
  vomega = 0.0;
}

void Motor::createPublishers()
{
}

void Motor::createSubscribers()
{
  this->motorMoveSub = this->node->Subscribe(std::string("~/RobotinoSim/MotorMove/"), &Motor::OnMotorMoveMsg, this);
}

void Motor::update()
{
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

void Motor::OnMotorMoveMsg(ConstVector3dPtr &msg)
{
  printf("Got MotorMove Msg!!! %f %f %f\n", msg->x(), msg->y(), msg->z());
  //Transform relative motion into ablosulte motion
  vx = msg->x();
  vy = msg->y();
  vomega = msg->z();
}
