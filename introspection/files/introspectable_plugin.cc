#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/IntrospectionManager.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class ModelPush : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      // Introspection callback.
      auto fCounterValue = [this]()
      {
        return this->counter;
      };

      // Register the counter element.
      gazebo::util::IntrospectionManager::Instance()->Register
      <int>("data://my_plugin/counter", fCounterValue);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ++this->counter;
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // A counter for testing the introspection capabilites.
    private: int counter = 0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ModelPush)
}
