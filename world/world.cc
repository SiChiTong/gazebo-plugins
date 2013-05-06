#include <gazebo/gazebo.hh>

namespace gazebo
{
  class FirstWorldPlugin : public WorldPlugin
  {
    public: FirstWorldPlugin() : WorldPlugin() 
            {
              printf("World-Plugin loaded!\n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }

  };
  GZ_REGISTER_WORLD_PLUGIN(FirstWorldPlugin)
}
