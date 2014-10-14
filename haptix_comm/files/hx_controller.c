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

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <haptix/comm/haptix.h>

//////////////////////////////////////////////////
void printState(const hxDeviceInfo *_deviceInfo, const hxSensor *_sensor)
{
  int i;

  printf("\tMotors:\n");
  for (i = 0; i < _deviceInfo->nmotor; ++i)
  {
    printf("\t\tMotor %d\n", i);
    printf("\t\t\tPosition: %f\n", _sensor->motor_pos[i]);
    printf("\t\t\tVelocity: %f\n", _sensor->motor_vel[i]);
    printf("\t\t\tTorque: %f\n" , _sensor->motor_torque[i]);
  }

  printf("\tJoints:\n");
  for (i = 0; i < _deviceInfo->njoint; ++i)
  {
    printf("\t\tJoint %d\n", i);
    printf("\t\t\tPosition: %f\n", _sensor->joint_pos[i]);
    printf("\t\t\tVelocity: %f\n", _sensor->joint_vel[i]);
  }

  printf("\tContact sensors:\n");
  for (i = 0; i < _deviceInfo->ncontactsensor; ++i)
  {
    printf("\t\t# %d\n", i);
    printf("\t\t\tvalue: %f\n", _sensor->contact[i]);
  }

  printf("\tIMUs:\n");
  for (i = 0; i < _deviceInfo->nIMU; ++i)
  {
    printf("\t\t# %d\n", i);
    printf("\t\t\tLinear acceleration: (%f, %f, %f)\n",
      _sensor->IMU_linacc[i][0], _sensor->IMU_linacc[i][1],
      _sensor->IMU_linacc[i][2]);
    printf("\t\t\tAngular velocity: (%f, %f, %f)\n",
      _sensor->IMU_angvel[i][0], _sensor->IMU_angvel[i][1],
      _sensor->IMU_angvel[i][2]);
  }
}

//////////////////////////////////////////////////
void printDeviceInfo(const hxDeviceInfo *_deviceInfo)
{
  printf("Device information received:\n");
  printf("Num motors: %d\n", _deviceInfo->nmotor);
  printf("Num joints: %d\n", _deviceInfo->njoint);
  printf("Num contact sensors: %d\n", _deviceInfo->ncontactsensor);
  printf("Num IMUs: %d\n", _deviceInfo->nIMU);
  printf("Joint limits: \n");

  // Print joint limits.
  int i;
  for (i = 0; i < _deviceInfo->njoint; ++i)
  {
    printf("\tJoint %d:\n", i);
    printf("\t\t Min: %f\n", _deviceInfo->limit[i][0]);
    printf("\t\t Max: %f\n", _deviceInfo->limit[i][1]);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  int counter = 0;
  hxDeviceInfo deviceInfo;
  hxCommand cmd;
  hxSensor sensor;

  // Requesting device information.
  if (hx_getdeviceinfo(hxGAZEBO, &deviceInfo) != hxOK)
  {
    printf("hx_getdeviceinfo(): Request error.\n");
    return -1;
  }

  // Print the device information.
  printDeviceInfo(&deviceInfo);

  // Send commands at ~100Hz.
  for (; ;)
  {
    // Create a new command based on a sinusoidal wave.
    for (i = 0; i < deviceInfo.nmotor; ++i)
    {
      cmd.ref_pos[i] = 0.5 * sin(0.05 * 2.0 * M_PI * counter * 0.01);
      cmd.ref_vel[i] = 1.0;
      cmd.gain_pos[i] = 1.0;
      cmd.gain_vel[i] = 1.0;
    }

    // Send the new joint command and receive the state update.
    if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
      printf("hx_update(): Request error.\n");

    // Debug output: Print the state.
    // printState(&deviceInfo, &sensor);

    if (++counter == 10000)
      counter = 0;

    usleep(10000);
  }

  return 0;
}
