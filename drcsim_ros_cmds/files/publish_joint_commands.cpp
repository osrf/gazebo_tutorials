/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();
  {
    // for testing round trip time
    jointcommands.header.stamp = _js->header.stamp;

    // assign sinusoidal joint angle targets
    for (unsigned int i = 0; i < jointcommands.name.size(); i++)
      jointcommands.position[i] =
        3.2* sin((ros::Time::now() - startTime).toSec());

    pub_joint_commands_.publish(jointcommands);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_joint_command_test");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

  // Waits for simulation time update.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  // must match those inside AtlasPlugin
  jointcommands.name.push_back("atlas::back_lbz");
  jointcommands.name.push_back("atlas::back_mby");
  jointcommands.name.push_back("atlas::back_ubx");
  jointcommands.name.push_back("atlas::neck_ay");
  jointcommands.name.push_back("atlas::l_leg_uhz");
  jointcommands.name.push_back("atlas::l_leg_mhx");
  jointcommands.name.push_back("atlas::l_leg_lhy");
  jointcommands.name.push_back("atlas::l_leg_kny");
  jointcommands.name.push_back("atlas::l_leg_uay");
  jointcommands.name.push_back("atlas::l_leg_lax");
  jointcommands.name.push_back("atlas::r_leg_uhz");
  jointcommands.name.push_back("atlas::r_leg_mhx");
  jointcommands.name.push_back("atlas::r_leg_lhy");
  jointcommands.name.push_back("atlas::r_leg_kny");
  jointcommands.name.push_back("atlas::r_leg_uay");
  jointcommands.name.push_back("atlas::r_leg_lax");
  jointcommands.name.push_back("atlas::l_arm_usy");
  jointcommands.name.push_back("atlas::l_arm_shx");
  jointcommands.name.push_back("atlas::l_arm_ely");
  jointcommands.name.push_back("atlas::l_arm_elx");
  jointcommands.name.push_back("atlas::l_arm_uwy");
  jointcommands.name.push_back("atlas::l_arm_mwx");
  jointcommands.name.push_back("atlas::r_arm_usy");
  jointcommands.name.push_back("atlas::r_arm_shx");
  jointcommands.name.push_back("atlas::r_arm_ely");
  jointcommands.name.push_back("atlas::r_arm_elx");
  jointcommands.name.push_back("atlas::r_arm_uwy");
  jointcommands.name.push_back("atlas::r_arm_mwx");

  unsigned int n = jointcommands.name.size();
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
      jointcommands.kp_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
      jointcommands.ki_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
      jointcommands.kd_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_min[i]);
    jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_max[i]);

    jointcommands.velocity[i]     = 0;
    jointcommands.effort[i]       = 0;
    jointcommands.kp_velocity[i]  = 0;
  }

  // ros topic subscriptions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  ros::spin();

  return 0;
}
