#include <gazebo/gazebo.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>

#ifndef GAZEBO_TUTORIALS_SET_VELOCITY_PID_LINK_HH_
#define GAZEBO_TUTORIALS_SET_VELOCITY_PID_LINK_HH_

using namespace gazebo;

//////////////////////////////////////////////////
// \brief use a PID controller to apply forces to achieve a target velocity
class PIDLinkVelocityController
{
  public: PIDLinkVelocityController()
    {
    }

  public: ~PIDLinkVelocityController()
    {
      this->Stop();
    }

  public: void Start(physics::LinkPtr _link, math::Vector3 _linearVel,
              math::Vector3 _angularVel, double _maxForce, double _maxTorque)
    {
      this->Stop();
      this->link = _link;
      this->targetLinearVel = _linearVel;
      this->targetAngularVel = _angularVel;

      // Hard coded gains. Tune these for your own application!
      double linear_p = 100.0;
      double linear_i = 0.0;
      double linear_d = 0.0;
      double linear_imax = 123456789.0;
      double angular_p = 100.0;
      double angular_i = 0.0;
      double angular_d = 0.0;
      double angular_imax = 123456789.0;

      // Add a PID controller for each DoF
      for (int i = 0; i < 3; i++)
      {
        common::PID controller_translation(linear_p, linear_i, linear_d,
            linear_imax, -linear_imax, _maxForce, -_maxForce);
        common::PID controller_rotation(angular_p, angular_i, angular_d,
            angular_imax, -angular_imax, _maxTorque, -_maxTorque);
        this->controllers.push_back(controller_translation);
        this->controllers.push_back(controller_rotation);
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PIDLinkVelocityController::Update, this,
          std::placeholders::_1));
    }

  public: void Stop()
    {
      this->lastSimTime.Set(0.0);
      this->link.reset();
      this->controllers.clear();
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
      math::Vector3 curLinearVel = this->link->GetWorldLinearVel();
      math::Vector3 curAngularVel = this->link->GetWorldAngularVel();
      math::Vector3 linearError = curLinearVel - this->targetLinearVel;
      math::Vector3 angularError = curAngularVel - this->targetAngularVel;

      // Get forces to apply from controllers
      math::Vector3 worldForce;
      math::Vector3 worldTorque;
      worldForce.x = this->controllers[0].Update(linearError.x, dt);
      worldTorque.x = this->controllers[1].Update(angularError.x, dt);
      worldForce.y = this->controllers[2].Update(linearError.y, dt);
      worldTorque.y = this->controllers[3].Update(angularError.y, dt);
      worldForce.z = this->controllers[4].Update(linearError.z, dt);
      worldTorque.z = this->controllers[5].Update(angularError.z, dt);

      // Add those forces to the body
      this->link->AddForce(worldForce);
      this->link->AddTorque(worldTorque);
    }
    lastSimTime = curTime;
  }

  private: physics::LinkPtr link;
  private: std::vector<common::PID> controllers;
  private: event::ConnectionPtr updateConnection;
  private: math::Vector3 targetLinearVel;
  private: math::Vector3 targetAngularVel;
  private: common::Time lastSimTime;
};

#endif
