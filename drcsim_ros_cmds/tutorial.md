# Overview

In this tutorial, we'll send target joint positions to a robot in simulation through the use of a simple joint position command ROS topic publisher.

The [DRCSim User Guide](https://bitbucket.org/osrf/drcsim/wiki/DRC/UserGuide) provides specifications of basic controller API over [ROS topics](http://www.ros.org/wiki/Topics).

In particular, we will make use of two ROS topics in this tutorial:

  - `/atlas/joint_states` published by the robot, and
  - `/atlas/joint_commands` subscribed by the robot plugin.

## Setup

We assume that you've already done the [installation step](/tutorials/?tut=drcsim_install).

If you haven't done so, make sure to source the environment setup.sh files with every new terminal you open:

~~~
source /usr/share/drcsim/setup.sh
~~~

To save on typing, you can add this script to your `.bashrc` files, so it's automatically sourced every time you start a new terminal.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

But remember to remove them from your `.bashrc` file when they are not needed any more.

### Create a ROS Package Workspace

If you haven't already, create a ros directory in your home directory and add it to your $ROS_PACKAGE_PATH. From the command line

~~~
mkdir ~/ros
export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}
~~~

Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on `roscpp`, `sensor_msgs` and `osrf_msgs`:

~~~
cd ~/ros
roscreate-pkg drcsim_joint_commands_tutorial roscpp trajectory_msgs osrf_msgs
~~~

### Create a ROS Node

**Note: Atlas versions 4 and 5 require different joint names. See [Atlas v4 and v5 instructions](/tutorials?cat=drcsim&tut=drcsim_ros_cmds#Atlasv4andv5)**

Copy and paste the following code as file
 `~/ros/drcsim_joint_commands_tutorial/src/publish_joint_commands.cpp`
 with any text editor (e.g. gedit, vi, emacs)
 or [download it here](http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp):

<include src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Compiling the ROS Node

Edit `~/ros/drcsim_joint_commands_tutorial/CMakeLists.txt`
 so that it looks like below
 (or [download it here](http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/CMakeLists.txt)):

<include src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/CMakeLists.txt' />


To compile, type below in a terminal:

~~~
roscd drcsim_joint_commands_tutorial
make
~~~

If you have gazebo installed in a non-standard location (such as a local install), you must set the `PKG_CONFIG_PATH` appropriately. For example, use the following if you have installed into ~/local:

~~~
roscd drcsim_joint_commands_tutorial
PKG_CONFIG_PATH=~/local/lib/pkgconfig make
~~~


## The code explained

### Headers and Global Variable Declarations
Below contains file license, various system and library dependency includes and
a couple of global variables.

<include to='/JointCommands jointcommands;/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### ROS Topic Callback Function

`SetJointStates` is a callback function for ROS topic `/atlas/joint_states`.
When a `JointState` message is received, following code section copies the
header time stamp from the `JointState` message into `JointCommands` message
for the purpose of measuring the age of the `JointCommands` message.
This callback then populates target joint positions with some arbitrarily chosen values for purpose of demo, and publishes them over ROS topic `/atlas/joint_commands`.

<include from='/void SetJointStates/' to='/publish\(jointcommands\);\n  \}\n}/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Main Subroutine
Initialize ros and creates a `ros::NodeHandle`.

<include from='/int main/' to='/= new ros::NodeHandle\(\);/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Waits for Simulation Time Update
Make sure simulation time has propagated to this node before running the rest of this demo code:

<include from='/  // Waits/' to='\false;\n  }\' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Hardcoded List of Joint Names

List of joint names in the Atlas robot.  Note the order must not change for this function to work correctly.

<include from='/  // must/' to='/r_arm_mwx"\);/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Size JointCommands Variables

Resize `JointCommands` based on the size of joint names list.

<include from='/  unsigned int n/' to='i_effort_max.resize\(n\);/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Fill in JointCommands Gains and Target Values

Retrieve JointCommands gains from ROS parameter server.
Set target velocities and efforts to zero.

<include from='/for \(unsigned int i = 0; i < n/' to='/jointcommands.kp_velocity\[i\]  = 0;\n  }/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Subscribe to `/atlas/joint_states` Message

Subscribe to the JointState message, but also set the subscriber option to use
unreliable transport (i.e. UDP connection).

<include from='/  // ros topic subscriptions/' to='/1000, SetJointStates\);/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Setup Publisher

Initialize JointCommands publisher.

<include from='/pub_joint_commands_ =/' to='/, 1, true\);/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Call `ros::spin()`

to process messages in the ROS callback queue

<include from='/ros::spin\(\);/' to='/return 0;\n  }/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

## Running the Simulation

* In a terminal, source the DRC simulator setup script and start the DRC robot simulation:

    ~~~
    roslaunch drcsim_gazebo atlas_sandia_hands.launch
    ~~~

* In a separate terminal, put Atlas in User mode:

    ~~~
    rostopic pub /atlas/control_mode std_msgs/String --  "User"
    ~~~

* In a separate terminal, run the node constructed above, which will cause
 Atlas to writhe around on the ground with all joints cycling through their full range of motion
 (see [this video](https://www.youtube.com/watch?v=-zpZ3lUvccI#t=23s)
  for a variant of this behavior):

    ~~~
    export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}
    rosrun drcsim_joint_commands_tutorial publish_joint_commands
    ~~~

## Altering the code

The reference trajectories for each joint position is a set of sinusoids. Since we know the exact function for the positions, we can compute the desired velocities by differentiation and supply this to the controller. This will improve the smoothness of the controller.

To add a reference velocity, alter the for loop in the SetJointStates function to match the following **(Note this only works if kp_velocity is non-zero)**:

~~~
// assign sinusoidal joint angle and velocity targets
for (unsigned int i = 0; i < jointcommands.name.size(); i++)
{
  jointcommands.position[i] =
    3.2* sin((ros::Time::now() - startTime).toSec());
  jointcommands.velocity[i] =
    3.2* cos((ros::Time::now() - startTime).toSec());
}
~~~

Then rebuild and re-run the node:

~~~
make
rosrun drcsim_joint_commands_tutorial publish_joint_commands
~~~

## Atlas v4 and v5

The sample code given above will not work for Atlas v4 and v5 because these later models have different joint names and more joints.

To control Atlas v4/v5, change lines 60-87 of `publish_joint_commands.cpp` to the following (or download a [modified version](http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands_v4v5.cpp) of the code):

<include from='/  jointcommands.name.push_back\("atlas::l_leg_hpz"\);/' to='/jointcommands.name.push_back\("atlas::back_bkx"\);/' src='http://github.com/osrf/gazebo_tutorials/raw/master/drcsim_ros_cmds/files/publish_joint_commands_v4v5.cpp' />

Next, make the ROS package as above. Note that if you downloaded the source code, you will have to either rename the file from `publish_joint_commands_v4v5.cpp` to `publish_joint_commands.cpp`. Alternatively, you can build a new executable from `publish_joint_commands_v4v5.cpp` by modifying `CMakeLists.txt`.

To run the new joint publisher, follow the steps above, but start DRCSim with the following command to launch Atlas v4:

~~~
roslaunch drcsim_gazebo atlas_sandia_hands.launch model_args:="_v4"
~~~

Or you can launch Atlas v5:

~~~
roslaunch drcsim_gazebo atlas_sandia_hands.launch model_args:="_v5"
~~~
