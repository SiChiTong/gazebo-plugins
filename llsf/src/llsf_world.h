/***************************************************************************
 *  llsf_world.h - The main plugin for the llsf field
 *
 *  Created: Sun Aug 18 14:55:33 2013
 *  Copyright  2013  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */


#include <gazebo/gazebo.hh>

#include "data_table.h"


namespace gazebo
{
  class LlsfWorldPlugin : public WorldPlugin
  {
  public:
    //Constructor
    LlsfWorldPlugin();
    //Destructor
    ~LlsfWorldPlugin();

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);


  private:
    //Table with simulation data
    LlsfDataTable *table_;

    //Node for communication
    transport::NodePtr node_;
    
    physics::WorldPtr world_;
  };
  GZ_REGISTER_WORLD_PLUGIN(LlsfWorldPlugin)
}