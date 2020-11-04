#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#ifndef GAZEBO_TUTORIALS_SET_VELOCITY_ODE_PERFECT_ANGULAR_HH_
#define GAZEBO_TUTORIALS_SET_VELOCITY_ODE_PERFECT_ANGULAR_HH_

using namespace gazebo;

namespace gazebo
{
  //////////////////////////////////////////////////
  /// \brief uses joint motors to perfectly control velocity (ODE only)
  class OdePerfectAngularVelController
  {
    public: OdePerfectAngularVelController()
      {
      }

    public: ~OdePerfectAngularVelController()
      {
        this->Stop();
      }

    /// \brief start controlling the angular velocity of an object
    /// \param[in] _link the link to move at the given velocity
    /// \param[in] _vel world velocity to move the link at
    /// \param[in] _maxTorque Maximum torque to apply each timestep
    public: void Start(
                physics::LinkPtr _link, math::Vector3 _vel, double _maxTorque)
      {
        this->Stop();
        // create revolute joint with the world as a parent
        physics::ModelPtr model = _link->GetModel();
        physics::WorldPtr world = physics::get_world("default");
        physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
        this->joint = engine->CreateJoint("revolute");
        this->joint->SetName(model->GetName() + "__perfect_ang_joint__");
        physics::LinkPtr worldLink =
          boost::dynamic_pointer_cast<physics::Link>(world->GetByName("world"));
        math::Pose jointOrigin;
        this->joint->Load(worldLink, _link, jointOrigin);
        this->joint->Init();
        double magnitude = _vel.GetLength();
        this->joint->SetAxis(0, _vel.Normalize());
        this->joint->SetParam("fmax", 0, _maxTorque);
        this->joint->SetParam("vel", 0, magnitude);
      }

    public: void Stop()
      {
        //remove the joint from the world
        if (this->joint)
        {
          this->joint->SetParam("fmax", 0, 0.0);
          this->joint->Detach();
          this->joint->Fini();
          this->joint.reset();
        }
      }
  
    private: physics::JointPtr joint;
  };
}
#endif
