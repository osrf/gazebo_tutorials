# Introduction

This tutorial describes how to use the Atlas Sim Interface to command Atlas to walk dynamically or step statically.

## Setup

We assume that you've already done the [DRCSim installation step](http://gazebosim.org/tutorials?tut=drcsim_install).

If you haven't done so, make sure to source the environment setup.sh files with every new terminal you open:

~~~
source /usr/share/drcsim/setup.sh
~~~

To save on typing, you can add this script to your **.bashrc** files, so it's automatically sourced every time you start a new terminal.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

But remember to remove them from your **.bashrc** file when they are not needed any more.

### Create a ROS Package Workspace

If you haven't already, create a ros directory in your home directory and add it to your $ROS\_PACKAGE\_PATH. From the command line

~~~
mkdir ~/ros
export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}
~~~

Use [roscreate-pkg]( http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on **rospy** and **atlas_msgs**:

~~~
cd ~/ros
roscreate-pkg atlas_sim_interface_tutorial rospy atlas_msgs
~~~

### Create a ROS Node
Download [`walk.py`](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py) into **`~/ros/atlas_sim_interface_tutorial/scripts/walk.py`**. This file contains the following code:

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

## The code explained

This node needs the following imports.

<include from='/1/' to='/11/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

Initializing the publishers and subscribers for this node. We publish to `/atlas/atlas_sim_interface_command` and `/atlas/mode` and listen to `/atlas/atlas_sim_interface_state` and `/atlas/state`.

<include from='/13/' to='/34/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

This is the `atlas_sim_interface_state` callback. It provides a lot of useful information. We can get the robot's current position (as estimated by the BDI controller). This position is what is needed to transform a local step coordinate to a global step coordinate.

<include from='/36/' to='/45/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

There are two types of walking behavior, static and dynamic. Dynamic is much faster, but foot placement is not as precise. Also, it is much easier to give bad walking commands that cause the atlas robot to fall. Static, is stable throughout the entire step trajectory.

<include from='/47/' to='/50/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

If the robot is rotated to the world frame, the orientation may need to be accounted for in positioning the steps. This is how you can do that. However, this node does not make use of orientation.

<include from='/52/' to='/56/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

This function walks the robot dynamically in a circle. It is necessary to publish 4 steps at any time, starting with the `next_step_index_needed`. This helps the walking controller plan for a stable walking trajectory. Some message fields aren't used or implemented in this walking behavior. Dynamic walking behavior is best for flat surfaces with no obstructions.

<include from='/58/' to='/88/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

This is an example of static walking/step behavior. You only specify one step at a time, and you have to check the step_feedback field in the state message to determine when you can send the next step command. It is statically stable throughout the entire step trajectory. If you need to step over objects, or step onto steps this behavior is necessary.

<include from='/91/' to='/126/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

This method calculates the pose of a step around a circle, based on the current step_index

<include from='/130/' to='/169/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

Main method to run walk. It checks if static is specified or not.

<include from='/171/' to='/178/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_interface/files/walk.py' />

# Running

Ensure that the above python file is executable

~~~
chmod +x ~/ros/atlas_sim_interface_tutorial/scripts/walk.py
~~~

Start up simulation

~~~
roslaunch drcsim_gazebo atlas_sandia_hands.launch
~~~

Rosrun the executable, specifying static if desired

Dynamic

~~~
rosrun atlas_sim_interface_tutorial walk.py
~~~

Static

~~~
rosrun atlas_sim_interface_tutorial walk.py static
~~~

## What you should see

Atlas should begin walking in a circle. Swiftly if it is dynamic behavior, or slowly if static behavior like the image below.

[[file:files/Asi_walk.png|800px]]
