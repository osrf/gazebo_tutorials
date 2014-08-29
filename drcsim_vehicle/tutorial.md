# Overview

This tutorial will demonstrate how to control the the DRC Vehicle using ROS topics provided by the VRC Plugin. Note that this tutorial demonstrates a development aid provided for testing and will not be available during the competition. In the course of this tutorial, we're going to drive the DRC Vehicle using direct commands to the steering wheel, hand brake, gas pedal, and brake pedal, which can be used to test vehicle navigation without using Atlas.

## Install DRC Simulator

[Click to see the instructions](http://gazebosim.org/tutorials/?tut=drcsim_install) for installing the DRC simulator and associated utilities. This tutorial requires drcsim-3.1 or later.

## Launch the DRC Simulator

Launch the simulator using the `atlas_drc_vehicle_fire_hose.launch` and with the development aid enabled:

    VRC_CHEATS_ENABLED=1 roslaunch drcsim_gazebo atlas_drc_vehicle_fire_hose.launch

## Read the DRC Vehicle interface states

The VRC plugin exposes ROS topics for the DRC Vehicle interface elements. In addition to the steering wheel, hand brake, and pedals, the DRC Vehicle also has a key switch and a 3-way direction switch (Forward / Neutral / Reverse). The current state of each interface element can be read on the following ROS topics, which broadcast at 1 Hz:

%%%
    /drc_vehicle/brake_pedal/state
    /drc_vehicle/direction/state
    /drc_vehicle/gas_pedal/state
    /drc_vehicle/hand_brake/state
    /drc_vehicle/hand_wheel/state
    /drc_vehicle/key/state
%%%

These topics can be viewed from the command line using, for example:

    rostopic echo /drc_vehicle/brake_pedal/state


The `brake_pedal`, `gas_pedal`, and `hand_brake` topics send a Float64 value scaled from 0 (disengaged) to 1 (fully engaged). The DRC Vehicle defaults to the pedals disengaged and the hand brake engaged. The `hand_wheel` topic reports the steering wheel angle in radians. Note that the steering wheel has a range of more than [-7 rad, 7 rad].

The direction state and key state topics send an Int8 value. The direction state reports "1" for Forward, "0" for Neutral, and "-1" for Reverse. The key state reports "1" for On, "0" for Off, and "-1" for an error caused by turning the key to On when the direction switch is not in Neutral. Putting the direction switch back to neutral will restore the key state to "1". The key switch defaults to "On" and the direction to "Forward", but this may not be the case in future versions of the software or in the competition.

## Control the DRC Vehicle with open-loop commands ##

The DRC Vehicle model currently does not have visual elements attached to the wheels. To see the steering, make the DRC Vehicle model transparent with collisions showing. This can be done for every model by selecting menu options View->Transparent and View->Collisions, or for the DRC Vehicle specifically by right-clicking on the vehicle in the rendering window and selecting the same options.

With these rendering options set, start by sending a command to turn the steering wheel to the left:

    rostopic pub --once /drc_vehicle/hand_wheel/cmd std_msgs/Float64 '{ data : 3.14 }'

Look at the front wheels to see a change.

Turn back to the right:

    rostopic pub --once /drc_vehicle/hand_wheel/cmd std_msgs/Float64 '{ data : -3.14 }'

Press the gas pedal:

    rostopic pub --once /drc_vehicle/gas_pedal/cmd std_msgs/Float64 '{ data : 1 }'

but the vehicle's not moving! The hand brake is engaged by default. Let's disengage it:

    rostopic pub --once /drc_vehicle/hand_brake/cmd std_msgs/Float64 '{ data : 0 }'

The vehicle will start driving in circles. Send the following command to turn the engine off:

    rostopic pub --once /drc_vehicle/key/cmd std_msgs/Int8 '{ data : 0 }'
