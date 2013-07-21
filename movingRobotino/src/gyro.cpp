#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "gyro.h"

using namespace gazebo;

Gyro::Gyro(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}

void Gyro::init()
{
  printf("Initialize Gyro \n");
}

void Gyro::createPublishers()
{
  this->gyroPub = this->node->Advertise<msgs::Vector3d>("~/RobotinoSim/Gyro/");
}

void Gyro::createSubscribers()
{

}

void Gyro::update()
{
  //Send gyro information to Fawkes
  sendGyro();
}

void Gyro::sendGyro()
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
