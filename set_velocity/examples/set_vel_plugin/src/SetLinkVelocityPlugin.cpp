/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/gazebo.hh>
#include "ode_perfect_linear.hh"
#include "ode_perfect_angular.hh"
#include "ode_perfect_velocity.hh"
#include "pid_link.hh"


namespace gazebo
{
  //////////////////////////////////////////////////
  /// \brief Sets velocity on a link or joint
  class SetLinkVelocityPlugin : public ModelPlugin
  {
    public: OdePerfectVelocityController odePerfectVelocity;
    public: OdePerfectLinearVelController odePerfectLinear;
    public: OdePerfectAngularVelController odePerfectAngular;
    public: PIDLinkVelocityController pidLinkController0;
    public: PIDLinkVelocityController pidLinkController1;
    public: PIDLinkVelocityController pidLinkController2;


    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));
      }

    public: void Update(const common::UpdateInfo &_info)
      {
        if (update_num == 0)
        {
          // Link velocity instantaneously without applying forces
          model->GetLink("white_link_0")->SetLinearVel({0, 1, 0});
          model->GetLink("white_link_1")->SetLinearVel({0, 1, 0});
          model->GetLink("white_link_1")->SetAngularVel({1, 0, 0});
          model->GetLink("white_link_2")->SetAngularVel({1, 0, 0});

          // Link velocity using joint motors
          odePerfectLinear.Start(model->GetLink("red_link_0"), {0, 1, 0}, 100);
          odePerfectVelocity.Start(model->GetLink("red_link_1"), {0, 1, 0}, {1, 0, 0}, 100, 100);
          odePerfectAngular.Start(model->GetLink("red_link_2"), {1, 0, 0}, 100);

          // Link velocity using PID controllers
          pidLinkController1.Start(
              model->GetLink("green_link_0"), {0, 1, 0}, {0, 0, 0}, 100, 0);
          pidLinkController0.Start(
              model->GetLink("green_link_1"), {0, 1, 0}, {1, 0, 0}, 100, 100);
          pidLinkController2.Start(
              model->GetLink("green_link_2"), {0, 0, 0}, {1, 0, 0}, 0, 100);
        }
        else if (update_num == 200)
        {
          odePerfectVelocity.Stop();
          odePerfectLinear.Stop();
          odePerfectAngular.Stop();
          pidLinkController0.Stop();
          pidLinkController1.Stop();
          pidLinkController2.Stop();
        }
        update_num++;
      }

    /// \brief a pointer to the model this plugin was loaded by
    public: physics::ModelPtr model;
    /// \brief object for callback connection
    public: event::ConnectionPtr updateConnection;
    /// \brief number of updates received
    public: int update_num = 0;
  };

  GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
}
