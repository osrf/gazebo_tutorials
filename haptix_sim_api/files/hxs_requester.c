/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <stdio.h>
#include <haptix/comm/haptix_sim.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define M_PI 3.14159265359

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  hxsSimInfo sim_info;
  hxsModel model;
  hxsJoint joint;
  hxsLink link;
  hxsTransform transform, camera_transform, new_transform;
  hxsVector3 lin_vel, ang_vel, force, torque;
  hxsWrench wrench;
  hxsColor color;
  hxsCollideMode collide_mode;
  int i, j;

  // List the models in the world and print their links and joints.
  if (hxs_sim_info(&sim_info) != hxOK)
  {
    printf("hxs_sim_info(): Request error.\n");
    return -1;
  }
  printf("Models:\n");
  for (i = 0; i < sim_info.model_count; i++)
  {
    model = sim_info.models[i];
    printf("\t%s\n", model.name);
    printf("\tLinks:\n");
    for (j = 0; j < model.link_count; j++)
    {
      link = model.links[j];
      printf("\t\t%s\n", link.name);
    }
    for (j = 0; j < model.joint_count; j++)
    {
      printf("\tJoints:\n");
      joint = model.joints[j];
      printf("\t\t%s\n", joint.name);
    }
  }

  // Get the user camera transform.
  if (hxs_camera_transform(&camera_transform) != hxOK)
  {
    printf("hxs_camera_transform(): Request error.\n");
    return -1;
  }

  // Rotate and translate the camera.
  new_transform = camera_transform;
  new_transform.pos.z = new_transform.pos.z + 1;
  new_transform.orient.w = new_transform.orient.w + M_PI/4;
  if (hxs_set_camera_transform(&new_transform) != hxOK)
  {
    printf("hxs_set_camera_transform(): Request error.\n");
    return -1;
  }
#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Restore the original camera transform.
  if (hxs_set_camera_transform(&camera_transform) != hxOK)
  {
    printf("hxs_set_camera_transform(): Request error.\n");
    return -1;
  }
#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Change the table color to red
  color.r = 1;
  color.b = 0;
  color.g = 0;
  color.alpha = 1;
  if (hxs_set_model_color("table", &color) != hxOK)
  {
    printf("hxs_set_model_color(): Request error.\n");
    return -1;
  }
#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Green
  color.r = 0;
  color.b = 1;
  color.g = 0;
  color.alpha = 1;
  if (hxs_set_model_color("table", &color) != hxOK)
  {
    printf("hxs_set_model_color(): Request error.\n");
    return -1;
  }
#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Blue
  color.r = 0;
  color.b = 0;
  color.g = 1;
  color.alpha = 1;
  if (hxs_set_model_color("table", &color) != hxOK)
  {
    printf("hxs_set_model_color(): Request error.\n");
    return -1;
  }
#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Get the table color
  if (hxs_model_color("table", &color) != hxOK)
  {
    printf("hxs_model_color(): Request error.\n");
    return -1;
  }
  printf("Table color: %f, %f, %f, %f\n",
      color.r, color.b, color.g, color.alpha);

  printf("Sliding cube:\n");
  if (hxs_linear_velocity("wood_cube_5cm", &lin_vel) != hxOK)
  {
    printf("hxs_linear_velocity(): Request error.\n");
    return -1;
  }

  printf("Linear velocity of cube before force: %f, %f, %f\n",
      lin_vel.x, lin_vel.y, lin_vel.z);
  // Apply a small force for 0.2 seconds
  force.x = -1.0;
  force.y = 0;
  force.z = 0;
  if (hxs_apply_force("wood_cube_5cm", "link", &force, 0.2) != hxOK)
  {
    printf("hxs_apply_force(): Request error.\n");
    return -1;
  }
#ifdef _WIN32
  Sleep(50)
#else
  usleep(50000);
#endif

  if (hxs_linear_velocity("wood_cube_5cm", &lin_vel) != hxOK)
  {
    printf("hxs_linear_velocity(): Request error.\n");
    return -1;
  }
  printf("Linear velocity of cube during force application: %f, %f, %f\n",
      lin_vel.x, lin_vel.y, lin_vel.z);

#ifdef _WIN32
  Sleep(10000)
#else
  sleep(1);
#endif

  if (hxs_linear_velocity("wood_cube_5cm", &lin_vel) != hxOK)
  {
    printf("hxs_linear_velocity(): Request error.\n");
    return -1;
  }
  printf("Linear velocity of cube after force application: %f, %f, %f\n",
      lin_vel.x, lin_vel.y, lin_vel.z);

#ifdef _WIN32
  Sleep(10000)
#else
  sleep(1);
#endif

  printf("Spinning cube:\n");
  if (hxs_angular_velocity("wood_cube_5cm", &ang_vel) != hxOK)
  {
    printf("hxs_angular_velocity(): Request error.\n");
    return -1;
  }
  printf("Angular velocity of cube before torque: %f, %f, %f\n",
      ang_vel.x, ang_vel.y, ang_vel.z);
  // Apply a small force for 0.2 seconds
  torque.x = 0;
  torque.y = 0;
  torque.z = 0.1;
  if (hxs_apply_torque("wood_cube_5cm", "link", &torque, 0.1) != hxOK)
  {
    printf("hxs_apply_torque(): Request error.\n");
    return -1;
  }

#ifdef _WIN32
  Sleep(50)
#else
  usleep(50000);
#endif

  if (hxs_angular_velocity("wood_cube_5cm", &ang_vel) != hxOK)
  {
    printf("hxs_angular_velocity(): Request error.\n");
    return -1;
  }
  printf("Angular velocity of cube during torque application: %f, %f, %f\n",
      ang_vel.x, ang_vel.y, ang_vel.z);

#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  if (hxs_angular_velocity("wood_cube_5cm", &ang_vel) != hxOK)
  {
    printf("hxs_angular_velocity(): Request error.\n");
    return -1;
  }
  printf("Angular velocity of cube after torque application: %f, %f, %f\n",
      ang_vel.x, ang_vel.y, ang_vel.z);

  // Apply force and torque at the same time
  wrench.force.x = 0;
  wrench.force.y = 0;
  wrench.force.z = 1;
  wrench.torque.x = 0;
  wrench.torque.y = 0;
  wrench.torque.z = 0.1;
  if (hxs_apply_wrench("wood_cube_5cm", "link", &wrench, 0.1) != hxOK)
  {
    printf("hxs_apply_wrench(): Request error.\n");
    return -1;
  }

#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Set the linear velocity back to zero
  lin_vel.x = 0;
  lin_vel.y = 0;
  lin_vel.z = 0;

  if (hxs_set_linear_velocity("wood_cube_5cm", &lin_vel) != hxOK)
  {
    printf("hxs_linear_velocity(): Request error.\n");
    return -1;
  }
  // Set the angular velocity back to zero
  ang_vel.x = 0;
  ang_vel.y = 0;
  ang_vel.z = 0;

  if (hxs_set_angular_velocity("wood_cube_5cm", &ang_vel) != hxOK)
  {
    printf("hxs_angular_velocity(): Request error.\n");
    return -1;
  }

  // Get the current gravity mode
  int gravity_mode;
  if (hxs_model_gravity_mode("wood_cube_5cm", &gravity_mode) != hxOK)
  {
    printf("hxs_gravity_mode(): Request error.\n");
    return -1;
  }

  printf("Gravity mode on?: %d\n", gravity_mode);

  // Set the model's gravity mode
  if (hxs_set_model_gravity_mode("wood_cube_5cm", 0) != hxOK)
  {
    printf("hxs_set_gravity_mode(): Request error.\n");
    return -1;
  }

  // Get the pose of the cube
  if (hxs_model_transform("wood_cube_5cm", &transform) != hxOK)
  {
    printf("hxs_model_transform(): Request error.\n");
    return -1;
  }

  printf("Cube position: %f, %f, %f\n",
      transform.pos.x, transform.pos.y, transform.pos.z);

  printf("Cube orientation (quaternion): %f, %f, %f, %f\n", transform.orient.w,
      transform.orient.x, transform.orient.y, transform.orient.z);

#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Modify the cube transform
  transform.pos.x = 0;
  transform.pos.y = 1.0;
  transform.pos.z = 1.5;
  transform.orient.w = M_PI/4;
  if (hxs_set_model_transform("wood_cube_5cm", &transform) != hxOK)
  {
    printf("hxs_model_transform(): Request error.\n");
    return -1;
  }
  sleep(1);

  // Get the model's collide mode
  if (hxs_model_collide_mode("wood_cube_5cm", &collide_mode))
  {
    printf("hxs_model_collide_mode(): Request error.\n");
    return -1;
  }

  printf("Cube collide mode: ");
  switch(collide_mode)
  {
    case hxsCOLLIDE:
      printf("Collide\n");
      break;
    case hxsNOCOLLIDE:
      printf("Don't collide\n");
      break;
    case hxsDETECTIONONLY:
      printf("Detect collisions, no interaction\n");
      break;
    default:
      break;
      printf("Unknown!\n");
  }

#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Set the model's collide mode to NOCOLLIDE
  collide_mode = hxsNOCOLLIDE;

  if (hxs_set_model_collide_mode("wood_cube_5cm", &collide_mode))
  {
    printf("hxs_set_model_collide_mode(): Request error.\n");
    return -1;
  }
  // The cube won't fall immediately; nudge it through the table
  force.x = 0;
  force.y = 0;
  force.z = -0.1;
  if (hxs_apply_force("wood_cube_5cm", "link", &force, 0.1) != hxOK)
  {
    printf("hxs_apply_force(): Request error.\n");
    return -1;
  }

#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Define a new model.  Here, we're taking the cricket_ball model from:
  //  https://bitbucket.org/osrf/gazebo_models/src/default/cricket_ball/model.sdf
  // and tweaking it slightly (just changing the color from Red to Green).

  char sdf[1024] = "<sdf version=\"1.5\"> <model name=\"cricket_ball\"> <link name=\"link\"> <pose>0 0 0.0375 0 0 0</pose> <inertial> <mass>0.1467</mass> <inertia> <ixx>8.251875e-05</ixx> <ixy>0</ixy> <ixz>0</ixz> <iyy>8.251875e-05</iyy> <iyz>0</iyz> <izz>8.251875e-05</izz> </inertia> </inertial> <collision name=\"collision\"> <geometry> <sphere> <radius>0.0375</radius> </sphere> </geometry> </collision> <visual name=\"visual\"> <geometry> <sphere> <radius>0.0375</radius> </sphere> </geometry> <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/Green</name> </script> </material> </visual> </link> </model> </sdf>";

  if (hxs_add_model(sdf, "green_cricket_ball", 0, 0, 5, 0, 0, 0, 1, &model)
      != hxOK)
  {
    printf("hxs_add_model(): Request error.\n");
    return -1;
  }

#ifdef _WIN32
  Sleep(2000)
#else
  sleep(2);
#endif

  // Remove the model
  if (hxs_remove_model("green_cricket_ball") != hxOK)
  {
    printf("hxs_remove_model(): Request error.\n");
    return -1;
  }

  // Set the state of a wrist joint.  Note that, because there's a controller
  // acting on the wrist, this change will only be transient; the controller will
  // restore the wrist back to the current target position.
  if (hxs_set_model_joint_state("mpl_haptix_right_forearm", "wristy", 0.5, 0.0)
      != hxOK)
  {
    printf("hxs_set_model_joint_state(): Request error.\n");
    return -1;
  }

#ifdef _WIN32
  Sleep(1000)
#else
  sleep(1);
#endif

  // Set the position of the arm. Note that if the motion tracking device is
  // active and unpaused, this change will be transient.
  transform.pos.x = 1.0;
  transform.pos.y = 0;
  transform.pos.z = 1.5;
  if (hxs_set_model_transform("mpl_haptix_right_forearm", &transform) != hxOK)
  {
    printf("hxs_set_model_transform(): Request error.\n");
    return -1;
  }

  // Move the camera
  if (hxs_set_camera_transform(&new_transform) != hxOK)
  {
    printf("hxs_set_camera_transform(): Request error.\n");
    return -1;
  }

#ifdef _WIN32
  Sleep(1000)
  sleep(1);
#endif

  // Reset the world, which will move the camera back
  if (hxs_reset(1) != hxOK)
  {
    printf("hxs_reset(): Request error.\n");
    return -1;
  }

  return 0;
}
