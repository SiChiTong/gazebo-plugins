#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <math.h>
#include <transport/transport.hh>
#include "simDevice.h"
#include "gps.h"
#include "config.h"

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
  x = 0.0;
  y = 0.0;
  ori = 0.0;
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
    float new_x = this->model->GetWorldPose().pos.x;
    float new_y = this->model->GetWorldPose().pos.y;
    float new_ori = this->model->GetWorldPose().rot.GetAsEuler().z;

    //send message only if the position has changed
    float diffX = fabs(new_x-x);
    float diffY = fabs(new_y-y);
    float diffOri = fabs(new_ori-ori);
    if(diffX > POSITION_SEND_TOLLERANCE || diffY > POSITION_SEND_TOLLERANCE || diffOri > POSITION_SEND_TOLLERANCE)
    {
      //update values
      x = new_x;
      y = new_y;
      ori = new_ori;

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
}
