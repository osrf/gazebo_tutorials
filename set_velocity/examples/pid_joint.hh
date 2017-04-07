#include <gazebo/gazebo.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>

using namespace gazebo;

//////////////////////////////////////////////////
// \brief use a PID controller to apply forces to achieve a target velocity
class PIDJointVelocityController
{
  public: PIDJointVelocityController()
    {
    }

  public: ~PIDJointVelocityController()
    {
      this->Stop();
    }

  public: void Start(physics::JointPtr _joint, int _axis, double _vel,
              double _maxForce)
    {
      this->Stop();
      this->joint = _joint;
      this->targetVel = _vel;
      this->axis = _axis;

      // Hard coded gains. Tune these for your own application!
      double p = 100.0;
      double i = 0.0;
      double d = 0.0;
      double imax = 123456789.0;

      this->controller.Init(p, i, d, imax, -imax, _maxForce, -_maxForce);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PIDJointVelocityController::Update, this,
          std::placeholders::_1));
    }

  public: void Stop()
    {
      this->lastSimTime.Set(0.0);
      this->joint.reset();
      this->updateConnection.reset();
    }

  public: void Update(const common::UpdateInfo &_info)
  {
    common::Time curTime = _info.simTime;
    if (this->lastSimTime.Double() < 0.000001)
    {
      // First update, need a second one to get change time
    }
    if (curTime < this->lastSimTime)
    {
      // Time moved backwards (World reset?)
      this->Stop();
    }
    else
    {
      // Get change in time between updates
      double dt = (curTime - lastSimTime).Double();

      // Calculate the error between actual and target velocity
      double curVel = this->joint->GetVelocity(this->axis);
      double error = curVel - this->targetVel;

      // Get forces to apply from controller
      double force = this->controller.Update(error, dt);

      // Set the force on the joint
      this->joint->SetForce(this->axis, force);
    }
    lastSimTime = curTime;
  }

  private: physics::JointPtr joint;
  private: common::PID controller;
  private: event::ConnectionPtr updateConnection;
  private: double targetVel;
  private: int axis;
  private: common::Time lastSimTime;
};

// ...

// Create the controller in your plugin so it lives for multiple time steps
PIDJointVelocityController controller;

// ...

// Start the controller with a target velocity on one of the joint axis
controller.Start(myJoint, 0, 1, 100.0);

// ...
// Wait at least one time step before stopping the controller or no
// force/torque will be applied!
controller.Stop();
