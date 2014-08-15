# Overview

In this tutorial, we'll send target joint positions to a robot in simulation through the use of a simple joint position command ROS topic publisher.

The [DRCSim User Guide](https://bitbucket.org/osrf/drcsim/wiki/DRC/UserGuide) provides specifications of basic controller API over [ROS topics](http://www.ros.org/wiki/Topics).

In particular, we will make use of two ROS topics in this tutorial:

  - `/atlas/joint_states` published by the robot, and
  - `/atlas/joint_commands` subscribed by the robot plugin.

## Setup

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install).

If you haven't done so, make sure to source the environment setup.sh files with every new terminal you open:

    source /usr/share/drcsim/setup.sh

To save on typing, you can add this script to your `.bashrc` files, so it's automatically sourced every time you start a new terminal.

    echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
    source ~/.bashrc

But remember to remove them from your `.bashrc` file when they are not needed any more.

### Create a ROS Package Workspace

If you haven't already, create a ros directory in your home directory and add it to your $ROS_PACKAGE_PATH. From the command line

    mkdir ~/ros
    export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}

Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on `roscpp`, `sensor_msgs` and `osrf_msgs`:

    cd ~/ros
    roscreate-pkg drcsim_joint_commands_tutorial roscpp trajectory_msgs osrf_msgs

### Create a ROS Node
Copy and paste the following code as file [`~/ros/drcsim_joint_commands_tutorial/src/publish_joint_commands.cpp`](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_cmds/files/publish_joint_commands.cc) with any text editor (e.g. gedit, vi, emacs):

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Compiling the ROS Node

Edit [`~/ros/drcsim_joint_commands_tutorial/CMakeLists.txt`](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_ros_cmds/files/CMakeLists.txt) so that it looks like below:

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/CMakeLists.txt' />


To compile, type below in a terminal:

    roscd drcsim_joint_commands_tutorial
    make

If you have gazebo installed in a non-standard location (such as a local install), you must set the PKG_CONFIG_PATH appropriately. For example, use the following if you have installed into ~/local:

    roscd drcsim_joint_commands_tutorial
    PKG_CONFIG_PATH=~/local/lib/pkgconfig make


## The code explained

### Headers and Global Variable Declarations
Below contains file license, various system and library dependency includes and
a couple of global variables.

<include to='/JointCommands jointcommands;/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### ROS Topic Callback Function

`SetJointStates` is a callback function for ROS topic `/atlas/joint_states`.
When a `JointState` message is received, following code section copies the
header time stamp from the `JointState` message into `JointCommands` message
for the purpose of measuring the age of the `JointCommands` message.
This callback then populates target joint positions with some arbitrarily chosen values for purpose of demo, and publishes them over ROS topic `/atlas/joint_commands`.

<include from='/void SetJointStates/' to='/publish\(jointcommands\);\n  \}\n}/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Main Subroutine
Initialize ros and creates a `ros::NodeHandle`.

<include from='/int main/' to='/= new ros::NodeHandle\(\);/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Waits for Simulation Time Update
Make sure simulation time has propagated to this node before running the rest of this demo code:

<include from='/  // Waits/' to='\false;\n  }\' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Hardcoded List of Joint Names

List of joint names in the Atlas robot.  Note the order must not change for this function to work correctly.

<include from='/  // must/' to='/r_arm_mwx"\);/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Size JointCommands Variables

Resize `JointCommands` based on the size of joint names list.

<include from='/  unsigned int n/' to='i_effort_max.resize\(n\);/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />


### Fill in JointCommands Gains and Target Values

Retrieve JointCommands gains from ROS parameter server.
Set target velocities and efforts to zero.

<include from='/for \(unsigned int i = 0; i < n/' to='/jointcommands.kp_velocity\[i\]  = 0;\n  }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Subscribe to `/atlas/joint\_states` Message

Subscribe to the JointState message, but also set the subscriber option to use
unreliable transport (i.e. UDP connection).

<include from='/  // ros topic subscriptions/' to='/1000, SetJointStates\);/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Setup Publisher

Initialize JointCommands publisher.

<include from='/pub_joint_commands_ =/' to='/, 1, true\);/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

### Call `ros::spin()`

to process messages in the ROS callback queue

<include from='/ros::spin\(\);/' to='/return 0;\n  }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/drcsim_ros_cmds/files/publish_joint_commands.cpp' />

## Running the Simulation

1. In terminal, source the DRC simulator setup script and start the DRC robot simulation:

        roslaunch drcsim_gazebo atlas_sandia_hands.launch

    **For drcsim < 3.1.0**: The package and launch file had a different name:

        roslaunch atlas_utils atlas_sandia_hands.launch

1. In a separate terminal, put Atlas in User mode:

        rostopic pub /atlas/control_mode std_msgs/String --  "User"

1. In a separate terminal, run the node constructed above:

        export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}
        rosrun drcsim_joint_commands_tutorial publish_joint_commands

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

        make
        rosrun drcsim_joint_commands_tutorial publish_joint_commands


## Next ##

[Next: Atlas control over ROS topics using python](http://gazebosim.org/tutorials/?tut=drcsim_ros_python)
