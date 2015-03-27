/*
 * Copyright 2013 Open Source Robotics Foundation
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

/*
 * Many thanks to the original author: Kevin Knoedler
 */

// Stubbed library AtlasRobotInterface that talks to Gazebo

// System includes
#include <stdio.h>
#include <math.h>
#include <boost/thread/mutex.hpp>

// From the AtlasRobotInterface SDK
#include <AtlasInterface.h>

// ROS stuff
#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <atlas_msgs/ControllerStatistics.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>

// KDL, installed via ROS
#include <kdl/frames_io.hpp>

static boost::mutex G_atlasstate_lock;
static atlas_msgs::AtlasState G_atlasstate_msg;
static boost::mutex G_atlassiminterface_lock;
static atlas_msgs::AtlasSimInterfaceState G_atlassiminterface_msg;
static int G_dataAvailable = 0;
static boost::mutex G_dataavailable_lock;

static ros::Publisher pub_atlas_command_;
static ros::Publisher pub_bdi1;
static int G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
static AtlasRobotBehavior G_atlas_behavior;

static void atlasstateCallback(const atlas_msgs::AtlasState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(G_atlasstate_lock);
  G_atlasstate_msg = *msg;
  G_dataAvailable++;
}

static void atlasSimInterfaceStateCallback(const atlas_msgs::AtlasSimInterfaceState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(G_atlassiminterface_lock);
  G_atlassiminterface_msg = *msg;
}

AtlasInterface::AtlasInterface()
{
  // Here I'm assuming that the user never creates more than one of these
  // objects in the same process.  That may be a bad assumption.
  const char* argv = "atlas_interface";
  int argc = 1;
  ros::init(argc, (char**)&argv, "atlas_interface");
  ros::NodeHandle* rosnode = new ros::NodeHandle();

  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
    {
      wait = false;
    }
  }

  // ros topic subscribtions

  // /atlas/atlas_state
  ros::SubscribeOptions atlasstateSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasState>(
     "/atlas/atlas_state", 1, atlasstateCallback,
     ros::VoidPtr(), rosnode->getCallbackQueue());
  atlasstateSo.transport_hints = ros::TransportHints().unreliable();
  static ros::Subscriber subAtlasState = rosnode->subscribe(atlasstateSo);

  // AtlasSimInterfaceState
  ros::SubscribeOptions simstateSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasSimInterfaceState>(
     "/atlas/atlas_sim_interface_state", 1, atlasSimInterfaceStateCallback,
     ros::VoidPtr(), rosnode->getCallbackQueue());
  simstateSo.transport_hints = ros::TransportHints().unreliable();
  static ros::Subscriber subSimState = rosnode->subscribe(simstateSo);

  pub_atlas_command_ =
    rosnode->advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1);
  pub_bdi1 =
  rosnode->advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command", 1);
}

AtlasInterface::~AtlasInterface()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

AtlasErrorCode AtlasInterface::open_net_connection_to_robot(std::string robot_hostname,
              int send_port,
              int recv_port)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return NO_ERRORS;
}

AtlasErrorCode AtlasInterface::close_net_connection_to_robot()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return NO_ERRORS;
}

bool AtlasInterface::net_connection_open()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return true;
}

#if ATLAS_VERSION_MAJOR >= 2 && ATLAS_VERSION_MINOR >= 4
AtlasErrorCode AtlasInterface::get_robot_ip_address(std::string& robot_ip_address)
{
  robot_ip_address = "10.66.171.30";
  return NO_ERRORS;
}
#endif

AtlasErrorCode AtlasInterface::start(AtlasHydraulicPressureSetting desired_pressure,
         int64_t* packet_seq_id)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return NO_ERRORS;
}

AtlasErrorCode AtlasInterface::stop(int64_t* packet_seq_id)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return NO_ERRORS;
}

/*
AtlasErrorCode AtlasInterface::suspend(int64_t* packet_seq_id)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return NO_ERRORS;
}
*/
bool AtlasInterface::command_in_progress()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return false;
}

AtlasErrorCode AtlasInterface::set_desired_behavior(AtlasRobotBehavior behavior,
            int64_t* packet_seq_id)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  G_atlas_behavior = behavior;
  if (behavior == BEHAVIOR_NONE)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::FREEZE; // FREEZE
  }
  else if (behavior == BEHAVIOR_FREEZE)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::FREEZE;
  }
  else if (behavior == BEHAVIOR_STAND_PREP)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND_PREP;
  }
  else if (behavior == BEHAVIOR_STAND)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
  }
  else if (behavior == BEHAVIOR_WALK)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::WALK;
  }
  else if (behavior == BEHAVIOR_STEP)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::STEP;
  }
  else if (behavior == BEHAVIOR_MANIPULATE)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE;
  }
  else if (behavior == BEHAVIOR_USER)
  {
    G_sim_behavior = atlas_msgs::AtlasSimInterfaceCommand::USER;
  }

  return NO_ERRORS;
}

AtlasErrorCode AtlasInterface::get_desired_behavior(AtlasRobotBehavior& desired_behavior)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return NO_ERRORS;
}

static void initSimInterfaceCommand(atlas_msgs::AtlasSimInterfaceCommand *msg)
{
  msg->k_effort.resize(Atlas::NUM_JOINTS);
  for (int i = 0; i < Atlas::NUM_JOINTS ; i++)
  {
    msg->k_effort[i] = 0;
  }
}

// Correctly sizes the arrays in an AtlasCommand
static void resizeAtlasCommand(atlas_msgs::AtlasCommand *ac)
{
  const static unsigned int n = Atlas::NUM_JOINTS;
  ac->position.resize(n);
  ac->velocity.resize(n);
  ac->effort.resize(n);
  ac->kp_position.resize(n);
  ac->ki_position.resize(n);
  ac->kd_position.resize(n);
  ac->kp_velocity.resize(n);
  ac->i_effort_min.resize(n);
  ac->i_effort_max.resize(n);
  ac->k_effort.resize(n);
}


static void zeroAtlasCommand(atlas_msgs::AtlasCommand *ac)
{
  for (int i = 0; i<Atlas::NUM_JOINTS; i++)
  {
    ac->position[i] = 0.0;
    ac->velocity[i] = 0.0;
    ac->effort[i] = 0.0;
    ac->kp_position[i] = 0.0;
    ac->ki_position[i] = 0.0;
    ac->kd_position[i] = 0.0;
    ac->kp_velocity[i] = 0.0;
    ac->i_effort_min[i] = 0.0;
    ac->i_effort_max[i] = 0.0;
    ac->k_effort[i] = 0;
  }
}

AtlasErrorCode AtlasInterface::send_control_data_to_robot(AtlasControlDataToRobot& data_to_robot,
            int64_t* packet_seq_id)
{
  //std::cout << __PRETTY_FUNCTION__ << std::endl;

  atlas_msgs::AtlasSimInterfaceCommand asic;
  atlas_msgs::AtlasCommand ac;
  initSimInterfaceCommand(&asic);
  resizeAtlasCommand(&ac);
  zeroAtlasCommand(&ac);

  for (int i = 0; i < Atlas::NUM_JOINTS; i++)
  {
    ac.position[i] = data_to_robot.j[i].q_d;
    ac.velocity[i] = data_to_robot.j[i].qd_d;
    ac.effort[i] = data_to_robot.j[i].f_d;

    ac.kp_position[i] = data_to_robot.jparams[i].k_q_p;
    ac.ki_position[i] = data_to_robot.jparams[i].k_q_i;
    ac.kd_position[i] = data_to_robot.jparams[i].k_qd_p;
    ac.kp_velocity[i] = 0;
    ac.k_effort[i] = 0;
    ac.i_effort_min[i] =  0.0;
    ac.i_effort_max[i] =  0.0;
    if (fabs(ac.ki_position[i] > 0.01))
    {
      ac.i_effort_min[i] = -10000;
      ac.i_effort_max[i] =  10000;
    }
    asic.k_effort[i] = 0;
  }


  // k_effort is gone from the real robot
  // set it to match the real robot behavior
  if (G_atlas_behavior == BEHAVIOR_MANIPULATE)
  {
    for (int i = 0; i < 4; i++)
    {
      ac.k_effort[i] = 255;
      asic.k_effort[i] = 255;
    }
    for (int i = 16; i < Atlas::NUM_JOINTS; i++)
    {
      ac.k_effort[i] = 255;
      asic.k_effort[i] = 255;
    }
  }
  else if (G_atlas_behavior == BEHAVIOR_USER)
  {
    for (int i = 0; i < Atlas::NUM_JOINTS; i++)
    {
      ac.k_effort[i] = 255;
      asic.k_effort[i] = 255;
    }
  }

  double roll, pitch, yaw, x, y, z, w;
  KDL::Rotation r1;
  asic.behavior = G_sim_behavior;

  asic.walk_params.use_demo_walk = data_to_robot.walk_params.use_demo_walk;

  for (int i = 0; i < 4; i++)
  {
    asic.walk_params.step_queue[i].step_index = data_to_robot.walk_params.step_queue[i].step_index;
    asic.walk_params.step_queue[i].foot_index = data_to_robot.walk_params.step_queue[i].foot_index;
    asic.walk_params.step_queue[i].duration   = data_to_robot.walk_params.step_queue[i].duration;
    asic.walk_params.step_queue[i].swing_height = data_to_robot.walk_params.step_queue[i].swing_height;
    asic.walk_params.step_queue[i].pose.position.x = data_to_robot.walk_params.step_queue[i].position.x();
    asic.walk_params.step_queue[i].pose.position.y = data_to_robot.walk_params.step_queue[i].position.y();
    asic.walk_params.step_queue[i].pose.position.z = data_to_robot.walk_params.step_queue[i].position.z();
    roll = asin(data_to_robot.walk_params.step_queue[i].normal.y());
    pitch = asin(data_to_robot.walk_params.step_queue[i].normal.x());
    yaw = data_to_robot.walk_params.step_queue[i].yaw;
    r1 = r1.RPY(roll, pitch, yaw);
    r1.GetQuaternion(x,y,z,w);
    asic.walk_params.step_queue[i].pose.orientation.x = x;
    asic.walk_params.step_queue[i].pose.orientation.y = y;
    asic.walk_params.step_queue[i].pose.orientation.z = z;
    asic.walk_params.step_queue[i].pose.orientation.w = w;
  }

  asic.step_params.use_demo_walk = data_to_robot.step_params.use_demo_walk;
  asic.step_params.desired_step.duration = data_to_robot.step_params.desired_step.duration;
  asic.step_params.desired_step.foot_index = data_to_robot.step_params.desired_step.foot_index;
  asic.step_params.desired_step.step_index = 1;
  asic.step_params.desired_step.swing_height = 0.3;
  asic.step_params.desired_step.pose.position.x = data_to_robot.step_params.desired_step.position.x();
  asic.step_params.desired_step.pose.position.y = data_to_robot.step_params.desired_step.position.y();
  asic.step_params.desired_step.pose.position.z = data_to_robot.step_params.desired_step.position.z();

  // data_to_robot.step_params.desired_step.normal.z(); // z is redundant
  roll = asin(data_to_robot.step_params.desired_step.normal.y());
  pitch = asin(data_to_robot.step_params.desired_step.normal.x());
  yaw = data_to_robot.step_params.desired_step.yaw;
  r1 = r1.RPY(roll, pitch, yaw);
  r1.GetQuaternion(x,y,z,w);
  asic.step_params.desired_step.pose.orientation.x = x;
  asic.step_params.desired_step.pose.orientation.y = y;
  asic.step_params.desired_step.pose.orientation.z = z;
  asic.step_params.desired_step.pose.orientation.w = w;

  //asic.stand_params.

  asic.manipulate_params.use_demo_mode = data_to_robot.manipulate_params.use_demo_mode;
  asic.manipulate_params.use_desired   = data_to_robot.manipulate_params.use_desired;
  asic.manipulate_params.desired.pelvis_height = data_to_robot.manipulate_params.desired.pelvis_height;
  asic.manipulate_params.desired.pelvis_lat    = data_to_robot.manipulate_params.desired.pelvis_lat;
  asic.manipulate_params.desired.pelvis_yaw    = data_to_robot.manipulate_params.desired.pelvis_yaw;

  pub_atlas_command_.publish(ac);
  pub_bdi1.publish(asic);

  ros::spinOnce();

  return NO_ERRORS;
}

AtlasErrorCode AtlasInterface::new_control_data_from_robot_available(double timeout,
                 bool* new_data_available)
{
  ros::spinOnce();

  *new_data_available = false;
  // sim sends data every 1mS, Atlas interface is every 3mS
  if (G_dataAvailable > 2)
  {
    *new_data_available = true;
  }

  return NO_ERRORS;
}

AtlasErrorCode AtlasInterface::get_control_data_from_robot(AtlasControlDataFromRobot* data_from_robot)
{
  // Waits until data is available
  while (G_dataAvailable < 3)
  {
    ros::spinOnce();
    //printf("waiting %d\n", G_dataAvailable);
    usleep(1000);
  }

  {
    boost::mutex::scoped_lock lock(G_atlasstate_lock);
    G_dataAvailable = 0;

    data_from_robot->air_sump_pressure = 77.7;

    data_from_robot->current_behavior = G_atlas_behavior;
    data_from_robot->filtered_imu.orientation_estimate.m_qx = G_atlasstate_msg.orientation.x;
    data_from_robot->filtered_imu.orientation_estimate.m_qy = G_atlasstate_msg.orientation.y;
    data_from_robot->filtered_imu.orientation_estimate.m_qz = G_atlasstate_msg.orientation.z;
    data_from_robot->filtered_imu.orientation_estimate.m_qw = G_atlasstate_msg.orientation.w;
    data_from_robot->filtered_imu.angular_velocity.set_x(G_atlasstate_msg.angular_velocity.x);
    data_from_robot->filtered_imu.angular_velocity.set_y(G_atlasstate_msg.angular_velocity.y);
    data_from_robot->filtered_imu.angular_velocity.set_z(G_atlasstate_msg.angular_velocity.z);
    data_from_robot->filtered_imu.linear_acceleration.set_x(G_atlasstate_msg.linear_acceleration.x);
    data_from_robot->filtered_imu.linear_acceleration.set_y(G_atlasstate_msg.linear_acceleration.y);
    data_from_robot->filtered_imu.linear_acceleration.set_z(G_atlasstate_msg.linear_acceleration.z);
    //data_from_robot->filtered_imu.imu_timestamp = ;


    data_from_robot->foot_sensors[0].fz = G_atlasstate_msg.l_foot.force.z;
    data_from_robot->foot_sensors[0].mx = G_atlasstate_msg.l_foot.torque.x;
    data_from_robot->foot_sensors[0].my = G_atlasstate_msg.l_foot.torque.y;
    data_from_robot->foot_sensors[1].fz = G_atlasstate_msg.r_foot.force.z;
    data_from_robot->foot_sensors[1].mx = G_atlasstate_msg.r_foot.torque.x;
    data_from_robot->foot_sensors[1].my = G_atlasstate_msg.r_foot.torque.y;

    for (int i = 0; i < Atlas::NUM_JOINTS; i++)
    {
      data_from_robot->j[i].f = G_atlasstate_msg.effort[i];
      data_from_robot->j[i].q = G_atlasstate_msg.position[i];
      data_from_robot->j[i].qd = G_atlasstate_msg.velocity[i];
    }

    //data_from_robot->processed_to_robot_packet_seq_id
    //data_from_robot->pump_inlet_pressure
    //data_from_robot->pump_pressure
    //data_from_robot->pump_return_pressure;
    //data_from_robot->pump_supply_pressure
    //data_from_robot->raw_imu
    //data_from_robot->robot_status_flags
    //data_from_robot->run_state;
    //data_from_robot->sensor_head_pps_timestamp;
    //data_from_robot->seq_id;
    //data_from_robot->stand_feedback;
    //data_from_robot->timestamp;
    data_from_robot->wrist_sensors[0].f.set_x(G_atlasstate_msg.l_hand.force.x);
    data_from_robot->wrist_sensors[0].m.set_x(G_atlasstate_msg.l_hand.torque.x);
    data_from_robot->wrist_sensors[0].f.set_y(G_atlasstate_msg.l_hand.force.y);
    data_from_robot->wrist_sensors[0].m.set_y(G_atlasstate_msg.l_hand.torque.y);
    data_from_robot->wrist_sensors[0].f.set_z(G_atlasstate_msg.l_hand.force.z);
    data_from_robot->wrist_sensors[0].m.set_z(G_atlasstate_msg.l_hand.torque.z);
    data_from_robot->wrist_sensors[1].f.set_x(G_atlasstate_msg.r_hand.force.x);
    data_from_robot->wrist_sensors[1].m.set_x(G_atlasstate_msg.r_hand.torque.x);
    data_from_robot->wrist_sensors[1].f.set_y(G_atlasstate_msg.r_hand.force.y);
    data_from_robot->wrist_sensors[1].m.set_y(G_atlasstate_msg.r_hand.torque.y);
    data_from_robot->wrist_sensors[1].f.set_z(G_atlasstate_msg.r_hand.force.z);
    data_from_robot->wrist_sensors[1].m.set_z(G_atlasstate_msg.r_hand.torque.z);
  }

  {
    boost::mutex::scoped_lock lock(G_atlassiminterface_lock);

    data_from_robot->behavior_feedback.status_flags = G_atlassiminterface_msg.behavior_feedback.status_flags;
    data_from_robot->behavior_feedback.trans_from_behavior_index = G_atlassiminterface_msg.behavior_feedback.trans_from_behavior_index;
    data_from_robot->behavior_feedback.trans_to_behavior_index = G_atlassiminterface_msg.behavior_feedback.trans_to_behavior_index;

    // Note - in the simulator a quaternion is provided for the foot pose, not just x/y/z
    data_from_robot->foot_pos_est[0].set_x(G_atlassiminterface_msg.foot_pos_est[0].position.x);
    data_from_robot->foot_pos_est[0].set_y(G_atlassiminterface_msg.foot_pos_est[0].position.y);
    data_from_robot->foot_pos_est[0].set_z(G_atlassiminterface_msg.foot_pos_est[0].position.z);
    data_from_robot->foot_pos_est[1].set_x(G_atlassiminterface_msg.foot_pos_est[1].position.x);
    data_from_robot->foot_pos_est[1].set_y(G_atlassiminterface_msg.foot_pos_est[1].position.y);
    data_from_robot->foot_pos_est[1].set_z(G_atlassiminterface_msg.foot_pos_est[1].position.z);


    data_from_robot->step_feedback.current_step_index     = G_atlassiminterface_msg.step_feedback.current_step_index;
    data_from_robot->step_feedback.next_step_index_needed = G_atlassiminterface_msg.step_feedback.next_step_index_needed;
    data_from_robot->step_feedback.status_flags           = G_atlassiminterface_msg.step_feedback.status_flags;
    data_from_robot->step_feedback.t_step_rem             = G_atlassiminterface_msg.step_feedback.t_step_rem;
    data_from_robot->step_feedback.desired_step_saturated.duration = G_atlassiminterface_msg.step_feedback.desired_step_saturated.duration;
    data_from_robot->step_feedback.desired_step_saturated.foot_index = G_atlassiminterface_msg.step_feedback.desired_step_saturated.foot_index;
    data_from_robot->step_feedback.desired_step_saturated.position.set_x(G_atlassiminterface_msg.step_feedback.desired_step_saturated.pose.position.x);
    data_from_robot->step_feedback.desired_step_saturated.position.set_y(G_atlassiminterface_msg.step_feedback.desired_step_saturated.pose.position.y);
    data_from_robot->step_feedback.desired_step_saturated.position.set_z(G_atlassiminterface_msg.step_feedback.desired_step_saturated.pose.position.z);
    data_from_robot->step_feedback.desired_step_saturated.step_index = G_atlassiminterface_msg.step_feedback.desired_step_saturated.step_index;
    data_from_robot->step_feedback.desired_step_saturated.swing_height = G_atlassiminterface_msg.step_feedback.desired_step_saturated.swing_height;

    KDL::Rotation r2;
    double yaw,pitch,roll, normx, normy, normz;
    r2 = r2.Quaternion(G_atlassiminterface_msg.step_feedback.desired_step_saturated.pose.orientation.x,
             G_atlassiminterface_msg.step_feedback.desired_step_saturated.pose.orientation.y,
             G_atlassiminterface_msg.step_feedback.desired_step_saturated.pose.orientation.z,
             G_atlassiminterface_msg.step_feedback.desired_step_saturated.pose.orientation.w);
    r2.GetRPY(roll, pitch, yaw);
    // fixme - test that these are correct
    normx = sin(pitch);
    normy = sin(roll);
    normz = sqrt(1.0 - (normx*normx + normy*normy)) ;
    data_from_robot->step_feedback.desired_step_saturated.normal.set_x(normx);
    data_from_robot->step_feedback.desired_step_saturated.normal.set_y(normy);
    data_from_robot->step_feedback.desired_step_saturated.normal.set_z(normz);
    data_from_robot->step_feedback.desired_step_saturated.yaw = yaw;


    data_from_robot->pos_est.position.set_x(G_atlassiminterface_msg.pos_est.position.x);
    data_from_robot->pos_est.velocity.set_x(G_atlassiminterface_msg.pos_est.velocity.x);
    data_from_robot->pos_est.position.set_y(G_atlassiminterface_msg.pos_est.position.y);
    data_from_robot->pos_est.velocity.set_y(G_atlassiminterface_msg.pos_est.velocity.y);
    data_from_robot->pos_est.position.set_z(G_atlassiminterface_msg.pos_est.position.z);
    data_from_robot->pos_est.velocity.set_z(G_atlassiminterface_msg.pos_est.velocity.z);

    data_from_robot->walk_feedback.current_step_index = G_atlassiminterface_msg.walk_feedback.current_step_index;
    data_from_robot->walk_feedback.next_step_index_needed = G_atlassiminterface_msg.walk_feedback.next_step_index_needed;
    data_from_robot->walk_feedback.status_flags = G_atlassiminterface_msg.walk_feedback.status_flags;
    data_from_robot->walk_feedback.t_step_rem = G_atlassiminterface_msg.walk_feedback.t_step_rem;
    for (int i = 0; i < 4 ; i++)
    {
      data_from_robot->walk_feedback.step_queue_saturated[i].duration = G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].duration;
      data_from_robot->walk_feedback.step_queue_saturated[i].foot_index = G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].foot_index;
      data_from_robot->walk_feedback.step_queue_saturated[i].position.set_x(G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].pose.position.x);
      data_from_robot->walk_feedback.step_queue_saturated[i].position.set_y(G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].pose.position.y);
      data_from_robot->walk_feedback.step_queue_saturated[i].position.set_z(G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].pose.position.z);
      data_from_robot->walk_feedback.step_queue_saturated[i].step_index = G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].step_index;
      data_from_robot->walk_feedback.step_queue_saturated[i].swing_height = G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].swing_height;

      r2 = r2.Quaternion(G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].pose.orientation.x,
           G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].pose.orientation.y,
           G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].pose.orientation.z,
           G_atlassiminterface_msg.walk_feedback.step_queue_saturated[i].pose.orientation.w);
      r2.GetRPY(roll, pitch, yaw);
      // fixme - test that these are correct
      normx = sin(pitch);
      normy = sin(roll);
      normz = sqrt(1.0 - (normx*normx + normy*normy)) ;
      data_from_robot->walk_feedback.step_queue_saturated[i].normal.set_x(normx);
      data_from_robot->walk_feedback.step_queue_saturated[i].normal.set_y(normy);
      data_from_robot->walk_feedback.step_queue_saturated[i].normal.set_z(normz);
      data_from_robot->walk_feedback.step_queue_saturated[i].yaw = yaw;
    }

    data_from_robot->manipulate_feedback.clamped.pelvis_height = G_atlassiminterface_msg.manipulate_feedback.clamped.pelvis_height;
    data_from_robot->manipulate_feedback.clamped.pelvis_lat =  G_atlassiminterface_msg.manipulate_feedback.clamped.pelvis_lat;
    data_from_robot->manipulate_feedback.clamped.pelvis_yaw =  G_atlassiminterface_msg.manipulate_feedback.clamped.pelvis_yaw;
    data_from_robot->manipulate_feedback.status_flags =  G_atlassiminterface_msg.manipulate_feedback.status_flags;
  }

  return NO_ERRORS;
}

AtlasErrorCode AtlasInterface::clear_faults(int64_t* packet_seq_id)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return NO_ERRORS;
}

#if ATLAS_VERSION_MAJOR >= 2 && ATLAS_VERSION_MINOR >= 4
AtlasErrorCode AtlasInterface::download_robot_log_file(std::string dest_directory, float duration)
{
  return NO_ERRORS;
}
#endif

std::string AtlasInterface::get_error_code_text(AtlasErrorCode ec)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return("hello");
}

std::string AtlasInterface::get_run_state_as_string(AtlasRobotRunState run_state)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return("hello");
}

std::string AtlasInterface::get_status_flag_as_string(AtlasRobotStatus robot_status_flag)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return("hello");
}

std::string AtlasInterface::link_name_from_link_id(AtlasLinkId link_id)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return("hello");
}

AtlasLinkId AtlasInterface::link_id_from_link_name(std::string link_name)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return LINK_PELVIS;
}

std::string AtlasInterface::joint_name_from_joint_id(AtlasJointId joint_id)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return("hello");
}

AtlasJointId AtlasInterface::joint_id_from_joint_name(std::string joint_name)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return JOINT_BACK_BKZ;
}

std::string AtlasInterface::behavior_name_from_behavior(AtlasRobotBehavior behavior)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return("hello");
}

AtlasRobotBehavior AtlasInterface::behavior_from_behavior_name(std::string behavior_name)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return BEHAVIOR_FREEZE;
}
