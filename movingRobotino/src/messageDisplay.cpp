#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include "messageDisplay.h"

using namespace gazebo;

MessageDisplay::MessageDisplay(physics::ModelPtr model, transport::NodePtr node)
 : SimDevice(model, node)
{
}

void MessageDisplay::initialize()
{
  printf("Initialize MessageDisplay Device \n");
}

void MessageDisplay::createPublishers()
{
}

void MessageDisplay::createSubscribers()
{
  this->stringSub = this->node->Subscribe(std::string("~/RobotinoSim/String/"), &MessageDisplay::onStringMsg, this);
}

void MessageDisplay::update()
{
}

void MessageDisplay::onStringMsg(ConstHeaderPtr &msg)
{
  printf("Msg from Fawkes: ");
  printf(msg->str_id().c_str());
  printf("\n");
}