#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#ifndef GAZEBO_TUTORIALS_SET_VELOCITY_ODE_PERFECT_VELOCITY_HH_
#define GAZEBO_TUTORIALS_SET_VELOCITY_ODE_PERFECT_VELOCITY_HH_

namespace gazebo
{
  //////////////////////////////////////////////////
  /// \brief uses joint motors to perfectly control velocity (ODE only)
  class OdePerfectVelocityController
  {
    public: OdePerfectVelocityController()
      {
      }

    public: ~OdePerfectVelocityController()
      {
        this->Stop();
      }

  public: void Start(physics::LinkPtr _link, math::Vector3 _linearVel,
              math::Vector3 _angularVel, double _maxForce, double _maxTorque)
      {
        this->Stop();
        physics::ModelPtr model = _link->GetModel();
        physics::WorldPtr world = physics::get_world("default");
        physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();

        // Create a phantom link
        this->phantomLink = model->CreateLink("__perfect_phantom_link__");
        sdf::ElementPtr config(new sdf::Element);
        config->Copy(_link->GetSDF()->Clone());
        config->GetAttribute("name")->Set("__perfect_phantom_link__");
        // Remove visuals/collisions/inertials
        while (config->HasElement("visual"))
        {
          config->RemoveChild(config->GetElement("visual"));
        }
        while (config->HasElement("collision"))
        {
          config->RemoveChild(config->GetElement("collision"));
        }
        while (config->HasElement("inertial"))
        {
          config->RemoveChild(config->GetElement("inertial"));
        }
        this->phantomLink->Load(config);
        this->phantomLink->Init();

        // create prismatic joint with parent "world" and child phantomLink
        this->prismaticJoint = engine->CreateJoint("prismatic");
        this->prismaticJoint->SetName(
            model->GetName() + "__perfect_vel_lin_joint__");
        physics::LinkPtr worldLink = boost::dynamic_pointer_cast<physics::Link>(
            world->GetByName("world"));
        math::Pose prismaticJointOrigin;
        this->prismaticJoint->Load(worldLink, this->phantomLink,
            prismaticJointOrigin);
        this->prismaticJoint->Init();
        double linearMagnitude = _linearVel.GetLength();
        this->prismaticJoint->SetAxis(0, _linearVel.Normalize());
        this->prismaticJoint->SetParam("fmax", 0, _maxForce);
        this->prismaticJoint->SetParam("vel", 0, linearMagnitude);

        // create revolute joint with parent phantomLink and child _link
        this->revoluteJoint = engine->CreateJoint("revolute");
        this->revoluteJoint->SetName(
            model->GetName() + "__perfect_vel_ang_joint__");
        math::Pose revoluteJointOrigin;
        this->revoluteJoint->Load(this->phantomLink, _link,
            revoluteJointOrigin);
        this->revoluteJoint->Init();
        double angularMagnitude = _angularVel.GetLength();
        this->revoluteJoint->SetAxis(0, _angularVel.Normalize());
        this->revoluteJoint->SetParam("fmax", 0, _maxTorque);
        this->revoluteJoint->SetParam("vel", 0, angularMagnitude);
      }

    public: void Stop()
      {
        //remove everything from the world
        if (this->prismaticJoint)
        {
          this->prismaticJoint->SetParam("fmax", 0, 0.0);
          prismaticJoint->Detach();
          this->prismaticJoint->Fini();
          this->prismaticJoint.reset();
        }
        if (this->revoluteJoint)
        {
          this->revoluteJoint->SetParam("fmax", 0, 0.0);
          revoluteJoint->Detach();
          this->revoluteJoint->Fini();
          this->revoluteJoint.reset();
        }
        if (this->phantomLink)
        {
          this->phantomLink->Fini();
          this->phantomLink.reset();
        }
      }

    private: physics::JointPtr prismaticJoint;
    private: physics::JointPtr revoluteJoint;
    private: physics::LinkPtr phantomLink;
  };
}
#endif

