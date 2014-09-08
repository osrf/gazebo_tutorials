# Overview

DRCSim v2.4 introduces an example interface to the Atlas Sim Interface and BDI Controller. It consists of an Actionlib server and a keyboard_teleop python ROS node. This tutorial explains how to use the teleop.

## Setup

Launch the simulator (be sure to first `source /usr/share/drcsim/setup.sh` as usual):

~~~
roslaunch drcsim_gazebo atlas_sandia_hands.launch
~~~

## Teleop

Launch keyboard teleop

~~~
roslaunch drcsim_gazebo keyboard_teleop.launch
~~~

You will see a screen like below:

~~~
Keyboard Teleop for AtlasSimInterface 1.0.5
Copyright (C) 2013 Open Source Robotics Foundation
Released under the Apache 2 License
--------------------------------------------------
Linear movement:

	i    
   j         l
	,    
	
Turn movements:
o/u Turn around a point
m/. Turn in place

1-9: Change the length of step trajectory
E: View and Edit Parameters
R: Reset robot to standing pose
Q: Quit
~~~

### Initializing/Resetting the robot

Press `R` to move the robot to a BDI-controlled standing position.  You should do this once at startup, before issuing other commands.  You can do it again at any time (e.g., after the robot has fallen over).

### Movements

Press `i` to move forward, `,` to move backward. `J` sidesteps laterally to the left, and `L` sidesteps to the right.

Pressing `o`, or `u` walks the atlas in a circle around a point to the left or right by 2 meters.

Pressing `m` and `.` turns the robot around on a point

This is an image of Atlas walking in a 2 meter circle.

[[file:files/Atlas_keyboard_teleop.png|800px]]

### Changing parameters

You can change a number of parameters to experiment with the walking controller. Press `E` on the main screen to enter the parameter adjustment screen. It should look like this:

~~~
0 : Walk Sequence Length  5
1 : Forward Stride Length 0.15
2 : Step Height           0
3 : Stride Duration       0.63
4 : Stride Width          0.2
5 : Lateral Stride Length 0.15
6 : In Place Turn Size    0.196349540849
7 : Swing Height          0.3
8 : Turn Radius           2
X : Exit
Enter number of param you want to change: 
~~~

Enter a number of the param you to change, in this case we can increase the forward stride length by pressing `1` and `Enter`. It brings up the following:

~~~
New value for Forward Stride Length [min: 0, max: 1, type: float]?
~~~

Type in 0.3 and press enter to increase the stride length. Then type x and press `Enter` to exit the screen.

Press `i` and the robot will walk forward a total of 1.5 meters.

It is recommended to keep the forward stride length below 0.4 meters, and the lateral stride length below 0.5 meters. The stride duration should be kept between 0.75 and 0.55, but it may depend on the chosen step width.

## Exiting
Press `Q` to exit `keyboard_teleop.py`, then Ctrl-C to kill `roslaunch`.

## Known Issues
 * Rotating in place with an Atlas model with hands is not very stable, use rotate around a point instead and plan your trajectories carefully. [Ticket](https://bitbucket.org/osrf/drcsim/issue/205/atlas-falls-while-rotating-in-place-using)
