#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>

#include "simDevice.h"

using namespace gazebo;

SimDevice::SimDevice(physics::ModelPtr model, transport::NodePtr node)
{
  //Store the pointer to the model
  this->model = model;

  //communication Node
  this->node = node;
}
