# Overview

This tutorial demonstrates how to start the 5-step behavior demo implemented in the Boston Dynamics Atlas Behavior Library.  Assuming DRCSim 2.7 or above was installed on your system, Atlas simulation behavior library documentations can be found locally, simply point your browser [here](https://bitbucket.org/osrf/drcsim/src/default/drcsim_model_resources/AtlasSimInterface_1.1.1/doc/html/?at=default). See also the [Keyboard teleop walking](http://gazebosim.org/tutorials/?tut=drcsim_keyboard_teleop) tutorial.

# Install DRC Simulator

[Click to see the instructions](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim) for installing the DRC simulator and associated utilities. This tutorial requires drcsim-2.7 or later.

# Launch the DRC Simulator

Launch the simulator using the [atlas.launch](https://bitbucket.org/osrf/drcsim/src/default/drcsim_gazebo/launch/atlas.launch)

~~~
source /usr/share/drcsim/setup.sh
VRC_CHEATS_ENABLED=1 roslaunch drcsim_gazebo atlas.launch
~~~

with the environment variable `VRC_CHEATS_ENABLED` set to 1.

The robot should start with individual joints position PID controlled:

[[file:files/Gazebo_with_drc_robot.png|300px]]

To start the walking behavior, within a separate terminal, execute:

~~~
source /usr/share/drcsim/setup.sh
rosrun drcsim_gazebo 5steps.py
~~~

The robot will execute a simple 5 step behavior.

[[file:files/atlas_5steps.png|300px]]

# 5steps.py Explained

Examining [5steps.py](https://bitbucket.org/osrf/drcsim/src/948dd560cf6b/ros/atlas_utils/scripts/5steps.py?at=drcsim_2.2),

~~~
mode.publish("harnessed")
control_mode.publish("stand-prep")
rospy.sleep(5.0)
mode.publish("nominal")
rospy.sleep(0.3)
control_mode.publish("stand")
rospy.sleep(1.0)
control_mode.publish("walk")
~~~

The core of it simply publishes over ros topic a set of string topics to perform mode switching in [VRC simulation world plugin](https://bitbucket.org/osrf/drcsim/src/948dd560cf6b/ros/atlas_msgs/VRCPlugin.cpp?at=drcsim_2.2#cl-157) and in [Atlas model plugin](https://bitbucket.org/osrf/drcsim/src/948dd560cf6b/ros/atlas_msgs/AtlasPlugin.cpp?at=drcsim_2.2#cl-598).

The basic sequence includes:

  1.  `harnessed`:  Tell the `VRCPlugin` to tether the robot inertially with its feet slight off the ground.
  1.  `stand-prep`:  Tell the controller within `AtlasPlugin` to go into a passive stance pose.
  1.  `nominal`:  Instructs the `VRCPlugin` to release the robot from its inertial tether.  Atlas will fall the the ground.
  1.  `stand`:  Instructs the controller within `AtlasPlugin` to begin its active balancing stand-behavior.
  1.  `walk`:  Instructs the controller within `AtlasPlugin` to begin its 5 step walking behavior.
