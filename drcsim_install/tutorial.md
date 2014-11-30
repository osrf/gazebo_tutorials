# Overview

This page explains how to install the DRC Simulator.

## Pre-compiled binaries


### Ubuntu Linux

**Note: as of drcsim 2.2.0, binary packages are available for 64-bit systems only.  A 32-bit version is still available as source, but 64-bit is required to run the BDI walking controller.**

#### First time setup

If this is your first time installing the simulator, there's some system configuration that you need to do.

1. Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the [Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.

1. Setup your computer to accept software from packages.osrfoundation.org and packages.ros.org (the DRC Simulator uses some parts of ROS).

    **Ubuntu 12.04 (precise)**

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu precise main" > /etc/apt/sources.list.d/drc-latest.list'

    **Ubuntu 14.04 (trusty)** (since drcsim-4.0)

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu trusty main" > /etc/apt/sources.list.d/drc-latest.list'

1. Retrieve and install the keys for the ROS and DRC repositories.

        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
        wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -

1. Update apt-get and install the DRC Simulator.

        sudo apt-get update

1. Install drcsim package:

        sudo apt-get install drcsim

#### Upgrading

If you have a previous version of the simulator installed, upgrading is easy:

        sudo apt-get update
        sudo apt-get install gazebo drcsim

Note that we explicitly mention gazebo in the install line to ensure that we'll get the newest version of it, too.

#### Running

Now that everything is installed, here's how to run it:

1. Configure your environment.  You need to do this in every shell where you run drcsim software (you might want to add it to your `~/.bashrc` or equivalent).

        source /opt/ros/[Your ROS Distro Name]/setup.bash   # this is necessary for drcsim 3.1.0 and later
        source /usr/share/drcsim/setup.sh

    If you get "Warning: failed to find Gazebo's setup.sh.  You will need to source it manually.", do as described [here](https://bitbucket.org/osrf/drcsim/issue/51/drcsim-110-setupsh-wont-load-gazebo-13)

1. To run the DRC Simulator, try the default [launch](http://ros.org/wiki/roslaunch) file:

        roslaunch drcsim_gazebo atlas.launch

    **For drcsim < 3.1.0**: The package and launch file had a different name:

        roslaunch atlas_utils atlas.launch

Note that the robot launches with an extremely simple controller running only on the ankle joints, and thus will appear to "wobble" back and forth, and may eventually fall down. If this happens, click "Edit->Reset Model Poses" to set the robot back on its feet. This simplistic "standing" controller is just a placeholder; it is meant to be replaced by your own control software.

Also, at the time of writing, the simulation will start with the robot "pinned" to the world for 10 (simulated seconds) before the controllers will engage.

### Other platforms (TODO)

Please help us by contributing patches and configuration to build packages on your favorite platform!

## Compiling from source
-
### Prerequisite: Install Gazebo
There are several ways of getting a working gazebo installation to use with drcsim from source installation:

1. Install using apt-get from the OSRF repository
2. Install gazebo from source

Both are very well documented in the [Gazebo Installation](http://gazebosim.org/tutorials?tut=install&cat=get_started) page.

### Ubuntu and ROS Hydro

Here we'll explain how to build drcsim from source. You will need a working installation of gazebo, explained in the previous section.


1. Configure your system to install packages from [ROS hydro](http://www.ros.org/wiki/hydro/Installation/Ubuntu).  E.g., on precise:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

1. Install compile-time prerequisites:

        sudo apt-get update

        # Install osrf-common's dependencies
        sudo apt-get install -y cmake               \
                                debhelper           \
                                ros-hydro-ros      \
                                ros-hydro-ros-comm

        # Install sandia-hand's dependencies
        sudo apt-get install -y ros-hydro-xacro        \
                                ros-hydro-ros          \
                                ros-hydro-image-common \
                                ros-hydro-ros-comm     \
                                ros-hydro-common-msgs  \
                                libboost-dev            \
                                avr-libc                \
                                gcc-avr                 \
                                libqt4-dev

        # Install gazebo-ros-pkgs
        sudo apt-get install -y libtinyxml-dev                 \
                                ros-hydro-opencv2             \
                                ros-hydro-angles              \
                                ros-hydro-cv-bridge           \
                                ros-hydro-driver-base         \
                                ros-hydro-dynamic-reconfigure \
                                ros-hydro-geometry-msgs       \
                                ros-hydro-image-transport     \
                                ros-hydro-message-generation  \
                                ros-hydro-nav-msgs            \
                                ros-hydro-nodelet             \
                                ros-hydro-pcl-conversions     \
                                ros-hydro-pcl-ros             \
                                ros-hydro-polled-camera       \
                                ros-hydro-rosconsole          \
                                ros-hydro-rosgraph-msgs       \
                                ros-hydro-sensor-msgs         \
                                ros-hydro-trajectory-msgs     \
                                ros-hydro-urdf                \
                                ros-hydro-dynamic-reconfigure \
                                ros-hydro-rosgraph-msgs       \
                                ros-hydro-tf                  \
                                ros-hydro-cmake-modules

        # Install drcsim's dependencies
        sudo apt-get install -y cmake debhelper                         \
                             ros-hydro-std-msgs ros-hydro-common-msgs   \
                             ros-hydro-image-common ros-hydro-geometry  \
                             ros-hydro-pr2-controllers                  \
                             ros-hydro-geometry-experimental            \
                             ros-hydro-robot-state-publisher            \
                             ros-hydro-image-pipeline                   \
                             ros-hydro-image-transport-plugins          \
                             ros-hydro-compressed-depth-image-transport \
                             ros-hydro-compressed-image-transport       \
                             ros-hydro-theora-image-transport           \
                             ros-hydro-ros-controllers                  \
                             ros-hydro-moveit-msgs                      \
                             ros-hydro-joint-limits-interface           \
                             ros-hydro-transmission-interface           \
                             ros-hydro-laser-assembler

1. Create the catkin workspace
Default branches of ros gazebo plugins, osrf-common, sandia-hand and drcsim will be included into the workspace.

         # Setup the workspace
         mkdir -p /tmp/ws/src
         cd /tmp/ws/src

         # Download needed software
         git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
         hg clone https://bitbucket.org/osrf/osrf-common
         hg clone https://bitbucket.org/osrf/sandia-hand
         hg clone https://bitbucket.org/osrf/drcsim

         # Source ros distro's setup.bash
         source /opt/ros/hydro/setup.bash

         # Build and install into workspace
         cd /tmp/ws
         catkin_make install

2. Run from the catkin workspace

         # Source the setup from workspace
         source /tmp/ws/install/setup.bash

         # Source drcsim setup file
         source /tmp/ws/install/share/drcsim/setup.sh

         # Test installation
         roslaunch drcsim_gazebo atlas.launch

### Ubuntu and ROS Indigo

Here we'll explain how to build drcsim from source. You will need a working installation of gazebo, explained in the previous section.


1. Configure your system to install packages from [ROS Indigo](http://www.ros.org/wiki/indigo/Installation/Ubuntu). E.g., on trusty:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

1. Install compile-time prerequisites:

        sudo apt-get update

        # Install osrf-common's dependencies
        sudo apt-get install -y cmake               \
                                debhelper           \
                                ros-indigo-ros      \
                                ros-indigo-ros-comm

        # Install sandia-hand's dependencies
        sudo apt-get install -y ros-indigo-xacro        \
                                ros-indigo-ros          \
                                ros-indigo-image-common \
                                ros-indigo-ros-comm     \
                                ros-indigo-common-msgs  \
                                libboost-dev            \
                                avr-libc                \
                                gcc-avr                 \
                                libqt4-dev

        # Install gazebo-ros-pkgs
        sudo apt-get install -y libtinyxml-dev                 \
                                ros-indigo-vision-opencv       \
                                ros-indigo-angles              \
                                ros-indigo-cv-bridge           \
                                ros-indigo-driver-base         \
                                ros-indigo-dynamic-reconfigure \
                                ros-indigo-geometry-msgs       \
                                ros-indigo-image-transport     \
                                ros-indigo-message-generation  \
                                ros-indigo-nav-msgs            \
                                ros-indigo-nodelet             \
                                ros-indigo-pcl-conversions     \
                                ros-indigo-pcl-ros             \
                                ros-indigo-polled-camera       \
                                ros-indigo-rosconsole          \
                                ros-indigo-rosgraph-msgs       \
                                ros-indigo-sensor-msgs         \
                                ros-indigo-trajectory-msgs     \
                                ros-indigo-urdf                \
                                ros-indigo-dynamic-reconfigure \
                                ros-indigo-rosgraph-msgs       \
                                ros-indigo-tf                  \
                                ros-indigo-cmake-modules

        # Install drcsim's dependencies
        sudo apt-get install -y cmake debhelper                          \
                             ros-indigo-std-msgs ros-indigo-common-msgs  \
                             ros-indigo-image-common ros-indigo-geometry \
                             ros-indigo-ros-control                      \
                             ros-indigo-geometry-experimental            \
                             ros-indigo-robot-state-publisher            \
                             ros-indigo-image-pipeline                   \
                             ros-indigo-image-transport-plugins          \
                             ros-indigo-compressed-depth-image-transport \
                             ros-indigo-compressed-image-transport       \
                             ros-indigo-theora-image-transport           \
                             ros-indigo-laser-assembler


1. Create the catkin workspace
Default branches of ros gazebo plugins, osrf-common, sandia-hand and drcsim will be included into the workspace.

         # Setup the workspace
         mkdir -p /tmp/ws/src
         cd /tmp/ws/src

         # Download needed software
         git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
         hg clone https://bitbucket.org/osrf/osrf-common
         hg clone https://bitbucket.org/osrf/sandia-hand
         hg clone https://bitbucket.org/osrf/drcsim

         # Change to the *indigo* branch in gazebo_ros_pkgs
         cd gazebo_ros_pkgs
         git checkout indigo-devel
         cd ..

         # Source ros distro's setup.bash
         source /opt/ros/indigo/setup.bash

         # Build and install into workspace
         cd /tmp/ws
         catkin_make install

2. Run from the catkin workspace

         # Source the setup from workspace
         source /tmp/ws/install/setup.bash

         # Source drcsim setup file
         source /tmp/ws/install/share/drcsim/setup.sh

         # Test installation
         roslaunch drcsim_gazebo atlas.launch


### Other platforms (TODO)

Please help us by contributing patches and configuration to build from source on your favorite platform!

# Atlas Simulation Interface 3.0.0

This section is for DRC competitors who have received the Atlas Simulation Interfaces library version 3.0.0 from BDI.

DRCSim version 4.2 or greater is required.

Follow this following steps:

1. Install DRCSim >= 4.2

1. Copy BDI's `libAtlasSimInterface.so.3.0.0` file over the library provided by DRCSim. BDI provides the `libAtlasSimInterface.so.3.0.0` to DRC competitors. If you are a DRC competitor with an Atlas robot, please contact BDI to acquire `libAtlasSimInterface.so.3.0.0`.

    a. Source your ROS setup file. For example, if you are using ROS Indigo:

    ~~~
    source /opt/ros/indigo/setup.sh
    ~~~

    b. If you installed DRCSim from debian:

    ~~~
    sudo cp libAtlasSimInterface.so.3.0.0 /opt/ros/$ROS_DISTRO/libAtlasSimInterface3.so.3.0.0
    sudo cp libQuadProg_x86-64_gcc46_noqt.so /opt/ros/$ROS_DISTRO/lib/
    # if provided also run: sudo cp libqpOASES_x86-64_gcc46_noqt.so /opt/ros/$ROS_DISTRO/lib/
    ~~~

    c. If you installed DRCSim in a catkin workspace:

    ~~~
    cp libAtlasSimInterface.so.3.0.0 <catkin_ws_path>/install/lib/
    cp libQuadProg_x86-64_gcc46_noqt.so <catkin_ws_path>/install/lib/
    # if provided also run: cp libqpOASES_x86-64_gcc46_noqt.so <catkin_ws_path>/install/lib/
    ~~~

1. The new interface needs `ulimit` set stack and core to unlimited:

    ~~~
    ulimit -s unlimited
    ulimit -c unlimited
    ~~~

1. Launch drcsim as usual

    ~~~
    roslaunch drcmsim_gazebo atlas.launch model_args:="_v5"
    ~~~

    Optionally add `--verbose` flag to get more console outputs:

    ~~~
    roslaunch drcmsim_gazebo atlas.launch model_args:="_v5" extra_gazebo_args:="--verbose"
    ~~~
