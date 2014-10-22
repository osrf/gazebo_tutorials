# Overview

This tutorial will demonstrate how to control the Atlas robot with a joint trajectory controller.  In the course of this tutorial we're going to make the Atlas robot try to take a step.  It will fall down, and that's OK, because our trajectory is incredibly simple and not at all reactive.  You're welcome to extend the example here to make the robot walk or go through other motions.

We're using the ROS topics demonstrated in the [previous tutorial that used C++](http://gazebosim.org/tutorials/?tut=drcsim_ros_cmds&cat=drcsim).  This general-purpose controller can be used to make a set of joints follow a desired trajectory specified as joint angles.  The controller is executed as part of a Gazebo plugin.  This arrangement allows the controller to run in-line with the simulation, approximating the on-robot situation in which the controller runs in a real-time environment.

**Important note:** The approach to robot control described here is not the best or only way to control the Atlas robot. It is provided for demonstration purposes. DRC participants should be aware that we expect the control system in simulation to change substantially as more sophisticated controller become available.

## Install DRC Simulator

[Click to see the instructions](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim) for installing the DRC simulator and associated utilities.

## Create a new ROS package

If you haven't already, create a ros directory in your home directory and add it to your `$ROS_PACKAGE_PATH`. From the command line

~~~
mkdir ~/ros
echo "export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}" >> ~/.bashrc
source ~/.bashrc
~~~

Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on a ROS package called `osrf_msgs`:

~~~
cd ~/ros
roscreate-pkg tutorial_atlas_control osrf_msgs
~~~

## The Code

Move to the directory `tutorial_atlas_control`

~~~
roscd tutorial_atlas_control
~~~

Copy the file [traj_yaml.py](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py) into it:

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Then download the [Traj_data2.yaml](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/Traj_data2.yaml) YAML file of a few trajectories, and place it in the same directory (`~/ros/tutorial_atlas_control`).

## Trying it out

If you haven't brought down your previous instance of the DRC simulator, kill the process by pressing Control + C in it's terminal. Now launch the robot

~~~
roslaunch drcsim_gazebo atlas.launch
~~~

You should see the Atlas standing in an empty world.  It will likely sway back and forth; that's an artifact of the controllers holding position on the joints.

In a separate terminal, put Atlas in `User` mode:

~~~
rostopic pub /atlas/control_mode std_msgs/String -- "User"
~~~

In a separate terminal, run the node that you just built:

~~~
roscd tutorial_atlas_control
python traj_yaml.py Traj_data2.yaml step_and_fall
~~~

You should see the robot try to take a step with its right leg and then fall down.

Here is another trajectory, since it's football season:

~~~
python traj_yaml.py Traj_data2.yaml touchdown
~~~

Then, just for fun:

~~~
python traj_yaml.py Traj_data2.yaml touchdown_exhausted
~~~

### Restarting

To try it again, go to the Gazebo "Edit" menu and click on "Reset Model Poses" (you might need to do this several times, use `Shift+Ctrl+R` for convenience).  That will teleport the robot back to its initial pose, from where you can run a trajectory again.  In this way, you can iterate, making changes to the program sending the trajectory and checking the effect in simulation, without shutting everything down.

## The Code Explained

<include to='/import ceil/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Specify the names of the joints in the correct order.

<include from='/atlasJointNames/' to='/'atlas::r_arm_mwx'\]/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Create a ROS callback for reading the `JointState` message published on `/atlas/joint_states`

<include from='/currentJointState/' to='/currentJointState = msg/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Import the files that we need.

<include from='/if __name/' to='/traj_name\]\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Set up the joint states subscriber.

<include from='/  # Setup subscriber/' to='/, jointStatesCallback\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Initialize the joint command message.

<include from='/  # initialize JointCommands/' to='/command.i_effort_max = zeros\(n\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Get the current controller gains from the parameter server.

<include from='/  # now get gains/' to='/-command.i_effort_max\[i\]/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Set up the joint command publisher.

<include from='/  # set up the publisher/' to='/, queue_size=1\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />

Read in each line from the yaml file as a trajectory command, and the current joint states from the ROS topic. Publish trajectory commands that interpolate between the initial state and the desired position to generate a smooth motion.

<include from='/  # for each trajectory/' to='/dt \/ float\(n\)\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_python/files/traj_yaml.py' />
