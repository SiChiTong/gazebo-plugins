#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "gps.h"

using namespace gazebo;

Gps::Gps(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}
Gps::~Gps()
{
}

void Gps::init()
{
  printf("Initialize Gps \n");
}

void Gps::createPublishers()
{
  this->gpsPub = this->node->Advertise<msgs::Vector3d>("~/RobotinoSim/Gps/");
}

void Gps::createSubscribers()
{

}

void Gps::update()
{
  //Send position information to Fawkes
  sendPosition();
}

void Gps::sendPosition()
{
  if(gpsPub->HasConnections())
  {
    //Read position and orientation from simulation
    float x = this->model->GetWorldPose().pos.x;
    float y = this->model->GetWorldPose().pos.y;
    float ori = this->model->GetWorldPose().rot.GetAsEuler().z;

    //build message
    msgs::Vector3d posMsg;
    //x,y,ori
    posMsg.set_x(x);
    posMsg.set_y(y);
    posMsg.set_z(ori);

    printf("%f, %f, %f\n", x, y, ori);

    //send
    gpsPub->Publish(posMsg);
  }
}
