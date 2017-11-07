#include "plugin_stop_physics.h"

namespace gazebo {

  // Constructor
  StopPhysicsPlugin::StopPhysicsPlugin():
	WorldPlugin()
  {
	ROS_ERROR("On the constructor");
  }

  // Destructor
  StopPhysicsPlugin::~StopPhysicsPlugin() {

  }

  void StopPhysicsPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
	ROS_ERROR("On the load");
	if(_world->GetEnablePhysicsEngine())
	{
		ROS_ERROR("In the if");
		_world->EnablePhysicsEngine(false);
	}
  }

} //end of namespace gazebo
