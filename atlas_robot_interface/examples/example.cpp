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

// An example program that uses the AtlasRobotInterface API.  The idea is that
// this program could be built against either the real AtlasRobotInterface
// library or against the simulation version of it, without any code changes.
// This program is not meant to actually control the robot in any reasonable
// fashion, and in fact it might be dangerous.

#include <stdio.h>
#include <unistd.h>

// From the AtlasRobotInterface SDK
#include <AtlasInterface.h>

int
main(int argc, char** argv)
{
  AtlasInterface atlas_interface;
  AtlasErrorCode ec;

  ec = atlas_interface.open_net_connection_to_robot("myrobot", 1234, 4321);
  if (ec != NO_ERRORS || !atlas_interface.net_connection_open())
  {
    printf("Failed to open network connection to the robot\n");
    return 1;
  }

  int64_t packet_seq_id = 0;
  int cnt = 0;
  while (true)
  {
    AtlasControlDataFromRobot data;
    bool data_available;
    // Query for new data, waiting up to 1 second
    ec = atlas_interface.new_control_data_from_robot_available(1.0,
           &data_available);
    if (ec != NO_ERRORS)
    {
      printf("Failed to query for new data availability.\n");
    }
    else if (data_available)
    {
      ec = atlas_interface.get_control_data_from_robot(&data);
      if (ec != NO_ERRORS)
      {
        printf("Failed to get data from robot\n");
      }
      // Every once in a while, print.
      else if (!(cnt % 100))
      {
        printf("Joint positions:\n");
        for (int i=0; i<Atlas::NUM_JOINTS; i++)
          printf(" %f", data.j[i].q);
        printf("\n");
      }
    }

    AtlasControlDataToRobot command;
    ec = atlas_interface.send_control_data_to_robot(command, &packet_seq_id);
    if (ec != NO_ERRORS)
      printf("Failed to send command from robot\n");

    usleep(50000);
  }
	
  return 0;
}

