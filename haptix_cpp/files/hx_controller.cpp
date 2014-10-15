/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <unistd.h>
#include <iostream>
#include <ignition/transport.hh>
#include <haptix/comm/msg/hxDevice.pb.h>
#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxSensor.pb.h>

//////////////////////////////////////////////////
void printState(const haptix::comm::msgs::hxSensor &_sensor)
{
  std::cout << "Motors: " << std::endl;
  for (int i = 0; i < _sensor.motor_pos_size(); ++i)
  {
    std::cout << "\tMotor " << i << std::endl;
    std::cout << "\t\tPosition: " << _sensor.motor_pos(i) << std::endl;
    std::cout << "\t\tVelocity: " << _sensor.motor_vel(i) << std::endl;
    std::cout << "\t\tTorque: " << _sensor.motor_torque(i) << std::endl;
  }

  std::cout << "Joints: " << std::endl;
  for (int i = 0; i < _sensor.joint_pos_size(); ++i)
  {
    std::cout << "\tJoint " << i << ":" << std::endl;
    std::cout << "\t\tPosition: " << _sensor.joint_pos(i) << std::endl;
    std::cout << "\t\tVelocity: " << _sensor.joint_vel(i) << std::endl;
  }

  std::cout << "Contact sensors:" << std::endl;
  for (int i = 0; i < _sensor.contact_size(); ++i)
  {
    std::cout << "\t#" << i << std::endl;
    std::cout << "\t\tValue: " << _sensor.contact(i) << std::endl;
  }

  std::cout << "IMUs: " << std::endl;
  for (int i = 0; i < _sensor.imu_linacc_size(); ++i)
  {
    std::cout << "\t# " << i << ":" << std::endl;
    std::cout << "\t\tLinear acceleration: (" << _sensor.imu_linacc(i).x()
              << "," << _sensor.imu_linacc(i).y() << ","
              << "," << _sensor.imu_linacc(i).z() << std::endl;
    std::cout << "\t\tAngular velocity: (" << _sensor.imu_angvel(i).x()
              << "," << _sensor.imu_angvel(i).y() << ","
              << "," << _sensor.imu_angvel(i).z() << std::endl;
  }
}

//////////////////////////////////////////////////
void printDeviceInfo(const haptix::comm::msgs::hxDevice &_deviceInfo)
{
  std::cout << "Device information received:" << std::endl;
  std::cout << "Num motors: " << _deviceInfo.nmotor() << std::endl;
  std::cout << "Num joints: " << _deviceInfo.njoint() << std::endl;
  std::cout << "Num contact sensors: " << _deviceInfo.ncontactsensor()
            << std::endl;
  std::cout << "Num IMUs: " << _deviceInfo.nimu() << std::endl;
  std::cout << "Joint limits:" << std::endl;

  // Print joint limits.
  for (int i = 0; i < _deviceInfo.limit_size(); ++i)
  {
    std::cout << "\tJoint " << i << ":" << std::endl;
    std::cout << "\t\tMin: " << _deviceInfo.limit(i).min() << std::endl;
    std::cout << "\t\tMax: " << _deviceInfo.limit(i).max() << std::endl;
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  /// \brief Timeout used for the service requests (ms.).
  const unsigned int Timeout = 1000;
  const std::string DeviceInfoSvcName = "/haptix/gazebo/GetDeviceInfo";
  const std::string UpdateSvcName = "/haptix/gazebo/Update";

  int counter = 0;
  bool result;
  haptix::comm::msgs::hxDevice unused;
  haptix::comm::msgs::hxDevice deviceInfo;
  haptix::comm::msgs::hxSensor sensor;
  ignition::transport::Node hxNode;
  std::string service;

  // We need to fill the message with something.
  unused.set_nmotor(0.0);
  unused.set_njoint(0.0);
  unused.set_ncontactsensor(0.0);
  unused.set_nimu(0.0);
  haptix::comm::msgs::hxJointAngle *limit = unused.add_limit();
  limit->set_min(0.0);
  limit->set_max(0.0);

  // Set the service name for requesting device information.
  service = DeviceInfoSvcName;

  // Requesting device information.
  bool executed = hxNode.Request(service, unused, Timeout, deviceInfo, result);
  if (executed)
  {
    if (result)
    {
      // Print the device information.
      printDeviceInfo(deviceInfo);
    }
    else
    {
      std::cerr << "Getdevicefo() Service call failed." << std::endl;
      return -1;
    }
  }
  else
  {
    std::cerr << "hx_getdevicefo() Service call timed out." << std::endl;
    return -1;
  }

  // Set the service name for requesting a joint update.
  service = UpdateSvcName;

  // Send commands at ~100Hz.
  for (; ;)
  {
    haptix::comm::msgs::hxCommand cmd;

    // Create a new command based on a sinusoidal wave.
    for (int i = 0; i < deviceInfo.nmotor(); ++i)
    {
      cmd.add_ref_pos(0.5 * sin(0.05 * 2.0 * M_PI * counter * 0.01));
      cmd.add_ref_vel(1.0);
      cmd.add_gain_pos(1.0);
      cmd.add_gain_vel(1.0);
    }

    // Send the new joint command and receive the state update.
    executed = hxNode.Request(service, cmd, Timeout, sensor, result);
    if (!executed || !result)
    {
      std::cerr << "Update() Service call failed." << std::endl;
      continue;
    }

    // Debug output: Print the state.
    // printState(sensor);

    if (++counter == 10000)
      counter = 0;

    usleep(10000);
  }

  return 0;
}
