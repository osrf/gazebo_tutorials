# DRC Tutorial: Switching control modes

***Note: In recent versions of DRCSim, the demo program provided in this tutorial will cause the robot to fall over when trying to walk.  Patches to fix that problem are most welcome!***

This tutorial will explain how to use the BDI-provided behavior library to control the Atlas robot, and how to switch between behavior-library control and your own controller.

## Background

The robot offers two gross modes of control:

* BDI mode: send a high-level goal, such as **stand** or **walk**, to BDI's behavior library
 * ROS topic: `/atlas/atlas_sim_interface_command`
 * message type: [atlas/AtlasSimInterfaceCommand](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/AtlasSimInterfaceCommand.msg)
* user mode: send setpoints and gains to the PID controllers that are running on each joint.
 * ROS topic: `/atlas/atlas_command`
 * message type: [atlas/AtlasCommand](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/AtlasCommand.msg)

You can mix and match these modes on a per-joint basis.  Each message type has a `k_effort` array, with one element per joint.  To exert BDI-mode control for joint `i`, set `k_effort[i]` to 0; for user-mode control, set it to 255.  In this way, you can, for example, ask the BDI library to stand but then control the arms yourself.  Note that some (perhaps many) combinations don't make sense, e.g., asking the BDI library to walk but retaining user-mode control of one of the legs.

You should always begin and end BDI-mode control in the **stand** mode.  We'll cover this below in the example.

## Setup 

There aren't special requirements beyond running drcsim >= 2.5.x with gazebo >= 1.7.y.  Be sure to start with the usual shell setup: `source /usr/share/drcsim/setup.sh`.  Let's bring the robot up in a world with some stuff in it:

~~~
roslaunch drcsim_gazebo qual_task_1.launch
~~~

## The demo 

What follows is, in a way, the worst sort of robot program: a single sequence of steps, executed open loop based on timers and without any feedback.  But it will serve to illustrate the point of this tutorial, which is how to manage BDI mode and User mode control of the Atlas robot.

This program proceeds in 5 steps:

1. Send the robot to a home position with all joints at position=0.  This step is executed in full User mode, with all joints controlled by the new command.
1. Switch to BDI STAND mode.  Note that this step is undertaken in two parts:
 1. Ask for STAND_PREP mode, but retain User mode control of all joints.  Wait a little bit to let the BDI library get ready to stand.
 1. Ask for STAND mode, handing control of all joints to BDI.
1. Take control of some of the arm and neck joints and move them to new positions. This is done while leaving the rest of the robot in BDI mode.  We're starting to challenge the STAND mode by moving the arms, but the robot should not fall over.
1. Ask for WALK mode, sending the robot forward 4 steps.  This is done while retaining User mode control of a few joints.  We're really challenging the WALK mode by keeping control of the arms, but it mostly succeeds.  Note that BDI mode expects foot poses to be expressed in the world frame; we first express them in the robot's ego-centric frame and then transform them into the world frame using received data from `/atlas/imu` and `/atlas/atlas_sim_interface_state`.
1. Send the robot back to home in full User mode.

To summarize, this program shows how to go from full User mode to full BDI mode to mixed User/BDI mode and back to full User mode.  Many other sequences of transitions are possible.

### Creating ROS environment to launch demo

For running the demo script a minimal ROS setup is needed. Use a directory under `ROS_PACKAGE_PATH` and follow the instructions to create a package there:

~~~
. /usr/share/drcsim/setup.sh
cd ~/ros
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
roscreate-pkg control_mode_switch atlas_msgs rospy python_orocos_kdl
roscd control_mode_switch
# copy the python script below with name demo.py
chmod +x demo.py
# To run the demo:
./demo.py
~~~ 

### The code 

The [`demo script`](http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_switch_modes/files/demo.py) in python which implements the instructions detailed in this section:

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_switch_modes/files/demo.py' />
