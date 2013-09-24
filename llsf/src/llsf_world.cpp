#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include "data_table.h"
#include "llsf_world.h"
#include <string.h>

using namespace gazebo;

LlsfWorldPlugin::LlsfWorldPlugin() : WorldPlugin() 
{
  //Init the communication Node
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init("LLSF");
  puck_update_frequency_ = 3.0;
}

LlsfWorldPlugin::~LlsfWorldPlugin() 
{
  delete light_control_;
  delete puck_localization_;
  delete rfid_sensors_;
}

void LlsfWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  
  //init simulation data
  LlsfDataTable::init(_world, node_);
  table_ = LlsfDataTable::get_table();

  //has to be created after the table
  light_control_ = new LightControl(world_);
  puck_localization_ = new PuckLocalization(world_);
  rfid_sensors_ = new RfidSensors();

  //connect update function
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LlsfWorldPlugin::Update, this));
  printf("LLSF-World-Plugin loaded!\n");
}

void LlsfWorldPlugin::Update()
{
  double time = world_->GetSimTime().Double();
  light_control_->update();
  if((time - last_puck_update_) > (1.0 / puck_update_frequency_))
  {
    last_puck_update_ = time;
    puck_localization_->update();
    rfid_sensors_->update();
  }
}
