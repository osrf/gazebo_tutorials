# DRC Simulator Installation

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

    A. In the case of using Ubuntu/Precise, choose between ROS/Groovy or ROS/Hydro:

        sudo apt-get install drcsim-groovy
        ... or ...
        sudo apt-get install drcsim-hydro

    B. In the case of using Ubuntu/Trusty use just drcsim package name:

        sudo apt-get install drcsim

#### Upgrading

If you have a previous version of the simulator installed, upgrading is easy:

        sudo apt-get update
        sudo apt-get install gazebo drcsim
        ... or ...
        sudo apt-get install gazebo drcsim-groovy (in Precise/Groovy)
        ... or ...
        sudo apt-get install gazebo drcsim-groovy (in Precise/Hydro)


Note that we explicitly mention gazebo in the install line to ensure that we'll get the newest version of it, too.

#### Running

Now that everything is installed, here's how to run it:

1. Configure your environment.  You need to do this in every shell where you run drcsim software (you might want to add it to your `~/.bashrc` or equivalent).

        source /opt/ros/[Your ROS Distro Name]/setup.bash   # this is necessary for drcsim 3.1.0 and later
        source /usr/share/drcsim/setup.sh

    **For drcsim < 1.3.0**: you need to source the setup file from a versioned directory.  E.g., for drcsim 1.2.X, you would do this:

        # OLDER VERSIONS ONLY
        source /usr/share/drcsim-1.2/setup.sh

    If you get "Warning: failed to find Gazebo's setup.sh.  You will need to source it manually.", do as described [here](https://bitbucket.org/osrf/drcsim/issue/51/drcsim-110-setupsh-wont-load-gazebo-13)

1. To run the DRC Simulator, try the default [launch](http://ros.org/wiki/roslaunch) file:

        roslaunch drcsim_gazebo atlas.launch

    **For drcsim < 3.1.0**: The package and launch file had a different name:

        roslaunch atlas_utils atlas.launch

    **For drcsim < 1.3.0**: The package and launch file had a different name:

        # OLDER VERSIONS ONLY
        roslaunch drc_robot_utils drc_robot.launch

Note that the robot launches with an extremely simple controller running only on the ankle joints, and thus will appear to "wobble" back and forth, and may eventually fall down. If this happens, click "Edit->Reset Model Poses" to set the robot back on its feet. This simplistic "standing" controller is just a placeholder; it is meant to be replaced by your own control software.

Also, at the time of writing, the simulation will start with the robot "pinned" to the world for 10 (simulated seconds) before the controllers will engage.

### Other platforms (TODO)

Please help us by contributing patches and configuration to build packages on your favorite platform!

Compiling from source
-
### Prerequisite: Install Gazebo
There are several ways of getting a working gazebo installation to use with drcsim from source installation:

1. Install using apt-get from the OSRF repository
2. Install gazebo from source

Both are very well documented in the [Gazebo Installation](http://gazebosim.org/tutorials?tut=install&cat=get_started) page.

### Ubuntu and ROS Groovy

Here we'll explain how to build drcsim from source. You will need a working installation of gazebo, explained in the previous section.


1. Configure your system to install packages from [ROS groovy](http://www.ros.org/wiki/groovy/Installation/Ubuntu).  E.g., on precise:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

1. Install compile-time prerequisites:

        sudo apt-get update

        # Install osrf-common's dependencies
        sudo apt-get install -y cmake               \
                                debhelper           \
                                ros-groovy-ros      \
                                ros-groovy-ros-comm

        # Install sandia-hand's dependencies
        sudo apt-get install -y ros-groovy-xacro        \
                                ros-groovy-ros          \
                                ros-groovy-image-common \
                                ros-groovy-ros-comm     \
                                ros-groovy-common-msgs  \
                                libboost-dev            \
                                avr-libc                \
                                gcc-avr                 \
                                libqt4-dev

        # Install gazebo-ros-pkgs
        sudo apt-get install -y libtinyxml-dev                 \
                                ros-groovy-opencv2             \
                                ros-groovy-angles              \
                                ros-groovy-cv-bridge           \
                                ros-groovy-driver-base         \
                                ros-groovy-dynamic-reconfigure \
                                ros-groovy-geometry-msgs       \
                                ros-groovy-image-transport     \
                                ros-groovy-message-generation  \
                                ros-groovy-nav-msgs            \
                                ros-groovy-nodelet             \
                                ros-groovy-pcl-conversions     \
                                ros-groovy-pcl-ros             \
                                ros-groovy-polled-camera       \
                                ros-groovy-rosconsole          \
                                ros-groovy-rosgraph-msgs       \
                                ros-groovy-sensor-msgs         \
                                ros-groovy-trajectory-msgs     \
                                ros-groovy-urdf                \
                                ros-groovy-dynamic-reconfigure \
                                ros-groovy-rosgraph-msgs       \
                                ros-groovy-tf                  \
                                ros-groovy-cmake-modules

        # Install drcsim's dependencies
        sudo apt-get install -y cmake debhelper                          \
                             ros-groovy-std-msgs ros-groovy-common-msgs  \
                             ros-groovy-image-common ros-groovy-geometry \
                             ros-groovy-pr2-controllers                  \
                             ros-groovy-geometry-experimental            \
                             ros-groovy-robot-model-visualization        \
                             ros-groovy-robot-state-publisher            \
                             ros-groovy-image-pipeline                   \
                             ros-groovy-image-transport-plugins          \
                             ros-groovy-compressed-depth-image-transport \
                             ros-groovy-compressed-image-transport       \
                             ros-groovy-theora-image-transport

#### **Option I:** Install using a catkin workspace
1. Create the catkin workspace
Default branches of ros gazebo plugins, osrf-common, sandia-hand and drcsim will be included into the workspace.

         # Setup the workspace
         mkdir -p /tmp/ws/src
         cd /tmp/ws/src

         # Download needed software
         git clone https://github.com/ros-simulation/gazebo_ros_pkgs
         hg clone https://bitbucket.org/osrf/osrf-common
         hg clone https://bitbucket.org/osrf/sandia-hand
         hg clone https://bitbucket.org/osrf/drcsim

         # We don't need the gazebo_ros_control package
         touch gazebo_ros_pkgs/gazebo_ros_control/CATKIN_IGNORE

         # Source ros distro's setup.bash
         source /opt/ros/groovy/setup.bash   # groovy or hydro

         # use CMakeLists.txt from drcsim (replace default caktin toplevel cmake file)
         cd /tmp/ws/src
         ln -s drcsim/CMakeLists.txt .

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

#### **Option II:** Install to standard system locations

1. Retrieve the code for ros-gazebo-plugins, osrf-common, sandia-hand, and drcsim

    **Release tarballs:** The best place to go for the code is the [distribution archive](http://gazebosim.org/assets/distributions/), which has a source `.tar.bz2` file for each version that has been released.  It's usually best to pick the latest versions of each package from that archive.

    **Repository:**  If you're feeling brave (or if you plan to do development with this software, you can get the latest code from the repositories:

        git clone https://github.com/ros-simulation/gazebo_ros_pkgs
        hg clone https://bitbucket.org/osrf/osrf-common
        hg clone https://bitbucket.org/osrf/sandia-hand
        hg clone https://bitbucket.org/osrf/drcsim

    You'll probably want to be using particular branches of each repository; if you need help on this issue, post to the [Q&A forum](http://answers.gazebosim.org/questions/).

1. Configure, build, and install osrf-common:

        cd osrf-common
        . /opt/ros/groovy/setup.sh
        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
        make
        sudo make install

1. Configure, build, and install sandia-hand:

        cd sandia-hand
        . /opt/ros/groovy/setup.sh
        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
        make
        sudo make install


1. Configure, build, and install gazebo-ros-pkgs:

        cd gazebo-ros-pkgs
        . /opt/ros/groovy/setup.sh

        # gazebo_msgs
        cd gazebo_msgs
        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
        make
        sudo make install

        # gazebo_plugins
        cd ../../gazebo_plugins
        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
        make
        sudo make install

        # gazebo_ros
        cd ../../gazebo_plugins
        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
        make
        sudo make install

1. Configure, build, and install drcsim, first pulling in ROS's and gazebo's configuration.  Note: drcsim will only compile on 64 bit OS.

        cd drcsim
        mkdir build
        cd build
        . /opt/ros/groovy/setup.sh
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
        make
        sudo make install

1. To run the DRC Simulator, first source the drcsim shell configuration file, then launch it:

        . /usr/share/drcsim/setup.sh
        roslaunch drcsim_gazebo atlas.launch


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
        sudo apt-get install -y cmake debhelper                          \
                             ros-hydro-std-msgs ros-hydro-common-msgs  \
                             ros-hydro-image-common ros-hydro-geometry \
                             ros-hydro-pr2-controllers                  \
                             ros-hydro-geometry-experimental            \
                             ros-hydro-robot-model-visualization        \
                             ros-hydro-robot-state-publisher            \
                             ros-hydro-image-pipeline                   \
                             ros-hydro-image-transport-plugins          \
                             ros-hydro-compressed-depth-image-transport \
                             ros-hydro-compressed-image-transport       \
                             ros-hydro-theora-image-transport

1. Create the catkin workspace
Default branches of ros gazebo plugins, osrf-common, sandia-hand and drcsim will be included into the workspace.

         # Setup the workspace
         mkdir -p /tmp/ws/src
         cd /tmp/ws/src

         # Download needed software
         git clone https://github.com/osrf/gazebo_ros_pkgs-current.git
         hg clone https://bitbucket.org/osrf/osrf-common
         hg clone https://bitbucket.org/osrf/sandia-hand
         hg clone https://bitbucket.org/osrf/drcsim

         # We don't need the gazebo_ros_control package
         touch gazebo_ros_pkgs-current/gazebo_ros_control/CATKIN_IGNORE

         # Source ros distro's setup.bash
         source /opt/ros/hydro/setup.bash   # hydro or hydro

         # use CMakeLists.txt from drcsim (replace default caktin toplevel cmake file)
         cd /tmp/ws/src
         ln -s drcsim/CMakeLists.txt .

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
                                ros-indigo-opencv2             \
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
                             ros-indigo-std-msgs ros-hydro-common-msgs  \
                             ros-indigo-image-common ros-hydro-geometry \
                             ros-indigo-pr2-controllers                  \
                             ros-indigo-geometry-experimental            \
                             ros-indigo-robot-model-visualization        \
                             ros-indigo-robot-state-publisher            \
                             ros-indigo-image-pipeline                   \
                             ros-indigo-image-transport-plugins          \
                             ros-indigo-compressed-depth-image-transport \
                             ros-indigo-compressed-image-transport       \
                             ros-indigo-theora-image-transport

1. Create the catkin workspace
Default branches of ros gazebo plugins, osrf-common, sandia-hand and drcsim will be included into the workspace.

         # Setup the workspace
         mkdir -p /tmp/ws/src
         cd /tmp/ws/src

         # Download needed software
         git clone https://github.com/osrf/gazebo_ros_pkgs-current.git
         hg clone https://bitbucket.org/osrf/osrf-common
         hg clone https://bitbucket.org/osrf/sandia-hand
         hg clone https://bitbucket.org/osrf/drcsim

         # Change to the *indigo* branch in gazebo_ros_pkgs
         cd gazebo_ros_pkgs
         git checkout origin/fix_build
         cd ..

         # We don't need the gazebo_ros_control package
         touch gazebo_ros_pkgs-current/gazebo_ros_control/CATKIN_IGNORE

         # Source ros distro's setup.bash
         source /opt/ros/indigo/setup.bash

         # use CMakeLists.txt from drcsim (replace default caktin toplevel cmake file)
         cd /tmp/ws/src
         ln -s drcsim/CMakeLists.txt .

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