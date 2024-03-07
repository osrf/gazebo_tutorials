# Overview

This tutorial will demonstrate how to automatically place the Atlas robot in the driver's seat of the DRC Vehicle using ROS topics provided by the VRC Plugin. Note that this is a development aid provided for testing and will not be available during the competition. In the course of this tutorial, we're going to automatically cause Atlas to enter and exit the DRC Vehicle, which can be used to test perception while driving and manipulation of the vehicle controls.

## Install DRC Simulator

[Click to see the instructions](/tutorials/?tut=drcsim_install&cat=drcsim) for installing the DRC simulator and associated utilities.

## Launch the DRC Simulator

Launch the simulator using the `atlas_drc_vehicle_fire_hose.launch`

~~~
VRC_CHEATS_ENABLED=1 roslaunch drcsim_gazebo atlas_drc_vehicle_fire_hose.launch
~~~

## Place the Atlas robot near the vehicle

The VRC plugin exposes ROS topics for entering and exiting the vehicle. Currently, these topics can be invoked from anywhere in the world, but that is subject to change in future versions of drcsim. These topics are named:

~~~
/drc_world/robot_enter_car
/drc_world/robot_exit_car
~~~

The topics take a pose message as an offset from the default positions relative to the vehicle. Be sure to wait at least 10 seconds after starting the simulation so Atlas can be unpinned (you will see the robot start swaying a little bit), or internal pin joints will fight against each other. To place the Atlas robot outside the driver's side door, use the robot_exit_car topic with a the pose offset specified below:

~~~
rostopic pub --once /drc_world/robot_exit_car geometry_msgs/Pose '{ position: {x: -0.58, y: -0.2}}'
~~~

To place the robot on the passenger's side of the DRC Vehicle facing the vehicle, specify a different pose offset and a different orientation (note that the orientation is specified as a [quaternion](http://en.wikipedia.org/wiki/Quaternion)):

~~~
rostopic pub --once /drc_world/robot_exit_car geometry_msgs/Pose \
    '{ position: {x: -0.5, y: -3.5}, orientation: {w: 0.707, z: 0.707}}'
~~~

Be careful to wait several seconds between the publishing of the robot_exit_car messages, or the simulated joints used to teleport the robot may not have time to deactivate, causing strange behavior.

## Place the Atlas robot in the vehicle

Place Atlas in the driver's seat with the robot_enter_car topic and the identity pose:

~~~
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{}'
~~~

Place Atlas in the passenger's seat by specifying a pose offset:

~~~
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{position: {y: -0.6, z: 0.01}}'
~~~

Please note that the VRC plugin uses a fixed joint to place Atlas in the car. If you Reset Model Poses or use the Move or Translate tools on either model, while the joint is in place, the joint constraint will be violated and cause strange things to happen. You will likely have to restart the simulation.

## Test perception with Atlas in the vehicle

With Atlas placed in the vehicle, you can use the methods described in [another tutorial](/tutorials/?tut=drcsim_vehicle&cat=drcsim) to drive the vehicle using open-loop commands. This will allow testing of Atlas's perception systems while driving in the vehicle.

Disengage the hand brake:

~~~
rostopic pub --once /drc_vehicle_xp900/hand_brake/cmd std_msgs/Float64 '{ data : 0 }'
~~~

Turn the steering wheel:

~~~
rostopic pub --once /drc_vehicle_xp900/hand_wheel/cmd std_msgs/Float64 '{ data : 3.14 }'
~~~

Press the gas pedal:

~~~
rostopic pub --once /drc_vehicle_xp900/gas_pedal/cmd std_msgs/Float64 '{ data : 1 }'
~~~

Now the DRC Vehicle should be driving with Atlas in the passenger's seat. As described a [tutorial on using rviz](/tutorials/?tut=drcsim_visualization&cat=drcsim) and a [tutorial on the MultiSense head](/tutorials/?tut=drcsim_multisense&cat=drcsim), you can use rviz to view the camera or laser scan data while driving.

## Use Atlas robot's foot to press gas pedal

This section is under construction.
