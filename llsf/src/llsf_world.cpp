#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include "data_table.h"
#include "llsf_world.h"

using namespace gazebo;

LlsfWorldPlugin::LlsfWorldPlugin() : WorldPlugin() 
{
}

LlsfWorldPlugin::~LlsfWorldPlugin() 
{
}

void LlsfWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  //Init the communication Node
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init();
  
  world_ = _world;
  
  //init simulation data
  LlsfDataTable::init(_world, node_);
  table_ = LlsfDataTable::get_table();

  printf("LLSF-World-Plugin loaded!\n");
}
