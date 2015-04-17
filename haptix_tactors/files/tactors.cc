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

#include <fcntl.h>
#include <map>
#include <time.h>
#include <unistd.h>
#include <haptix/comm/haptix.h>

int main(int argc, char** argv)
{
  hxRobotInfo robotInfo;
  hxSensor sensor;
  const unsigned int sleeptime_us = 10000;
  const float minContactForce = 0;

  // Open the USB device for communication with the motors.
  int fd = open("/dev/ttyACM0", O_WRONLY);
  if (fd < 0)
  {
    perror("Failed to open /dev/ttyACM0");
    return -1;
  }

  // Initialize haptix-comm
  if (hx_robot_info(&robotInfo) != hxOK)
  {
    printf("hx_getdeviceinfo(): Request error.\n");
    return -1;
  }

  // Initialize sensor index to Lilypad motor index mapping
  // These sensor indices correspond to the JHU APL MPL arm
  std::map<int, char> sensorMotorIndexMapping;
  for (unsigned int i = 0; i < robotInfo.contact_sensor_count; i++)
  {
    // Uninitialized
    sensorMotorIndexMapping[i] = '0';
  }
  for (unsigned int i = 3; i <= 6; i++)
  {
    // Index
    sensorMotorIndexMapping[i] = '1';
  }
  for (unsigned int i = 11; i <= 14; i++)
  {
    // Middle
    sensorMotorIndexMapping[i] = '2';
  }
  for (unsigned int i = 17; i <= 20; i++)
  {
    // Ring
    sensorMotorIndexMapping[i] = '3';
  }
  for (unsigned int i = 7; i <= 9; i++)
  {
    // Little
    sensorMotorIndexMapping[i] = '4';
  }
  for (unsigned int i = 21; i <= 23; i++)
  {
    // Thumb
    sensorMotorIndexMapping[i] = '5';
  }

  for (; ;)
  {
    // Get a sensor update from the simulator.
    if (hx_read_sensors(&sensor) != hxOK)
      printf("hx_read_sensors(): Request error.\n");

    for (unsigned int i = 0; i < robotInfo.contact_sensor_count; i++)
    {
      if (sensor.contact[i] > minContactForce)
      {
        char j = sensorMotorIndexMapping[i];
        if (j <= '5' && j >= '1')
        {
          // Write to the corresponding motor to make it buzz
          char key[1] = {j};
          write(fd, key, 1);
        }
      }
    }

    // Wait until the next update
    usleep(sleeptime_us);
  }
  if (close(fd) == -1)
  {
    perror("Close failed.\n");
    return -1;
  }
  return 0;
}
