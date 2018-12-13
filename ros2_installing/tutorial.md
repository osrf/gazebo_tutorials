# Introduction

The set of ROS 2 packages for interfacing with Gazebo are contained within a
meta package named `gazebo_ros_pkgs`.
See
[ROS 2 Overview](http://gazebosim.org/tutorials/?tut=ros2_overview)
for background information before continuing here.

The packages support ROS 2 Crystal and Gazebo 9, and can be installed from
debian packages or from source.

## Prerequisites

You should understand the basic concepts of ROS 2 and have gone through some
[ROS 2 Tutorials](https://index.ros.org/doc/ros2/Tutorials).

### Install ROS 2

To install ROS 2 Crystal, see the
[ROS 2 installation page](https://index.ros.org/doc/ros2/Installation).
Either a source or a binary installation should work; **be sure to install the
Crystal** distribution.

> **Tip**: Don't forget to source your `setup.bash` script as instructed
  on the ROS installation page.

### Install Gazebo

You can install Gazebo either from source or from pre-build packages. See
[Install Gazebo](http://gazebosim.org/tutorials?cat=install).

You should install Gazebo 9. If installing from source, be sure to build the
`gazebo9` branch.

> **Tip**: You may need to source Gazebo's setup file if you're having difficulty
finding plugins and other resources. For example: `. /usr/share/gazebo/setup.sh`.

## Install gazebo\_ros\_pkgs

Follow either the instructions to install from debian packages, or the
instructions to install from source.

### Install from debian packages (on Ubuntu)

Assuming you already have some Crystal debian packages installed, install
`gazebo_ros_pkgs` as follows:

    sudo apt install ros-crystal-gazebo-ros-pkgs

### Install from source (on Ubuntu)

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

1. Install all dependencies:

        cd ~/ws
        rosdep install --from-paths src --ignore-src -r -y

1. Then build:

        cd ~/ws
        colcon build

1. If you've had any problems building, be sure to ask for help at
   [answers.gazebosim.org](http://answers.gazebosim.org/questions/).

1. Be sure to source this workspace's install setup for every new terminal
   you open:

        . ~/ws/install/setup.bash

    > **Tip**: You can make this be automatically sourced for every new terminal
      by running this once: `echo "source ~/ws/install/setup.bash" >> ~/.bashrc`

## Testing Gazebo and ROS 2 integration

Assuming your ROS 2 and Gazebo environments have been properly setup and built,
you should now be able to load Gazebo worlds which contain ROS 2 plugins, and to
insert models at runtime which have ROS 2 plugins in them.

Gazebo ROS packages provides several demo worlds for you to get a quick start
with the plugins. The demo worlds can be found
[here](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/worlds), and are installed by default under
`/opt/ros/<distro>/share/gazebo_plugins/worlds/`.

Each world file comes with instructions at the top with some example commands
that you can run to test its functionality, be sure to check that out!

Let's try loading one of them now!

1. Open a new terminal

1. Source  ROS 2 as instructed when you installed ROS 2.

1. If you installed `gazebo_ros_pkgs` from source, source the workspace:

        . ~/ws/install/setup.bash

1. Load the differential drive world with Gazebo:

        gazebo --verbose /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world

1. The Gazebo GUI should appear with a simple vehicle:

    [[file:figs/gazebo_ros_diff_drive.png|600px]]

1. On a new terminal (this is the 2nd one), run the following command to take a
   look at the world file.

        gedit /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world

1. See how the block on the top has a few example commands? Let's open a 3rd
   terminal and, again, source ROS 2 and `gazebo_ros_pkgs` as described above.

1. Then run one of the commands, for example:

        ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1

1. You'll see the vehicle moving forward:

    [[file:figs/gazebo_ros_diff_drive_lin_vel.gif|600px]]

1. Try out the other commands listed on the file, and try mofidying their
   values to get a feeling of how things work. Also try out other demo worlds!

