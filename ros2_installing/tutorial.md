# Introduction

The set of ROS 2 packages for interfacing with Gazebo are contained within a
meta package named `gazebo_ros_pkgs`.
See
[Overview ROS 2 integration](http://gazebosim.org/tutorials/?tut=ros2_overview)
for background information before continuing here.

These instructions are for using the Gazebo with the current ROS 2 master
branch, to be released on December 2018 as ROS 2 Crystal Clemmys.

## Prerequisites

You should understand the basic concepts of ROS 2 and have gone through the
[ROS 2 Tutorials](https://github.com/ros2/ros2/wiki/Tutorials).

### Install ROS 2

You'll need to install ROS 2's master branch from source until the Crystal
release. See the
[ROS 2 installation page](https://github.com/ros2/ros2/wiki/Installation)
for detailed instructions.

> **Tip**: Be sure to source your `setup.bash` script by following the
  instructions on the ROS installation page.

### Install Gazebo

You can install Gazebo either from source or from pre-build Ubuntu debians. Be
sure to install Gazebo 9.

See [Install Gazebo](http://gazebosim.org/tutorials?cat=install). If installing from source, be sure to build the `gazebo9` branch.

## Install gazebo\_ros\_pkgs

Before the ROS 2 Crystal release, installation must be done from source.

### Install from Source (on Ubuntu)

> **Tip**: These instructions require the use of the
  [colcon](https://colcon.readthedocs.io/en/released/) build tool, which is the
  standard tool used in ROS 2.

1. Create a directory for the colcon workspace:

        mkdir -p ~/ws/src

1. Make sure `git` is installed on your Ubuntu machine:

        sudo apt install git

1. Download the source code from the [`gazebo_ros_pkgs` repository](https://github.com/ros-simulation/gazebo_ros_pkgs):

        cd ~/ws/src
        git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b ros2

1. Then go to the workspace root directory and build:

        cd ~/ws
        colcon build

1. If you've had any problems building, be sure to ask for help at
   [answers.gazebosim.org](http://answers.gazebosim.org/questions/).

1. Be sure to source this workspace's install setup for every new terminal
   you open:

        . ~/ws/install/setup.bash

    > **Tip**: You can make this be automatically sourced for every new terminal
      with the following command:

            echo "source ~/ws/install/setup.bash" >> ~/.bashrc

## Testing Gazebo and ROS 2 integration

Assuming your ROS 2 and Gazebo environments have been properly setup and built,
you should now be able to load Gazebo worlds which contain ROS 2 plugins, and to
insert models into Gazebo at runtime which have ROS 2 plugins in them.

Gazebo ROS packages provides several demo worlds for you to get a quick start
with the plugins. The demo worlds can be found
[here](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/worlds).
Each world comes with instructions on the top of some example commands that you
can run to test its functionality, be sure to check that out.

Let's try loading one of them now!

1. Open a new terminal

1. Source your ROS 2 installation as instructed when you installed ROS 2.

1. Source your `gazebo_ros_pkgs` workspace:

        . ~/ws/install/setup.bash

1. Download the differential drive example world:

        mkdir ~/ros2_gazebo_demos
        cd ~/ros2_gazebo_demos
        wget https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/ros2/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world

1. Load the world with Gazebo:

        gazebo --verbose gazebo_ros_diff_drive_demo.world

1. The Gazebo GUI should appear with a simple vehicle:

    [[file:figs/gazebo_ros_diff_drive.png|800px]]

1. On a new terminal (this is the 2nd one), run the following command to take a
   look at the world file.

        gedit ~/ros2_gazebo_demos/gazebo_ros_diff_drive_demo.world

1. See how the block on the top has a few example commands.
   Let's open a 3d terminal and source ROS 2 and `gazebo_ros_pkgs` as described
   above.

1. Then run one of the commands, for example:

        ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1

1. You'll see the vehicle moving forward:

    [[file:figs/gazebo_ros_diff_drive_lin_vel.png|800px]]

1. Try out the other commands listed on the file, and try mofidying their
   values to get a feeling of how things work.

