# Overview

This tutorial creates a simulation world with a simple box that is animated
in a 10 second repeating loop where it slides around on the ground.

This tutorial also demonstrates several different ways of viewing,
accessing, and interacting with simulation using the Gazebo-classic executable
or your own custom executable.

The simulated box broadcasts its pose,
and a callback is created to receive the pose
and print out the location and timestamp of the box.

# Setup

Create a working directory.

~~~
mkdir ~/gazebo_animatedbox_tutorial
cd ~/gazebo_animatedbox_tutorial
~~~

# Animate box code

Copy [animated_box.cc](https://github.com/osrf/gazebo/blob/gazebo6_6.0.0/examples/stand_alone/animated_box/animated_box.cc), [independent_listener.cc](https://github.com/osrf/gazebo/blob/gazebo6_6.0.0/examples/stand_alone/animated_box/independent_listener.cc), [integrated_main.cc](https://github.com/osrf/gazebo/blob/gazebo6_6.0.0/examples/stand_alone/animated_box/integrated_main.cc), [CMakeLists.txt](https://github.com/osrf/gazebo/blob/gazebo6_6.0.0/examples/stand_alone/animated_box/CMakeLists.txt), and [animated_box.world](https://github.com/osrf/gazebo/blob/gazebo6_6.0.0/examples/stand_alone/animated_box/animated_box.world) into the current directory.

On OS X, you can replace `wget` with `curl -OL`.

~~~
wget http://github.com/osrf/gazebo/raw/gazebo6_6.0.0/examples/stand_alone/animated_box/animated_box.cc
wget http://github.com/osrf/gazebo/raw/gazebo6_6.0.0/examples/stand_alone/animated_box/independent_listener.cc
wget http://github.com/osrf/gazebo/raw/gazebo6_6.0.0/examples/stand_alone/animated_box/integrated_main.cc
wget http://github.com/osrf/gazebo/raw/gazebo6_6.0.0/examples/stand_alone/animated_box/CMakeLists.txt
wget http://github.com/osrf/gazebo/raw/gazebo6_6.0.0/examples/stand_alone/animated_box/animated_box.world
~~~

Build the plugin

~~~
mkdir build
cd build
cmake ../
make
~~~

Make sure Gazebo-classic can load the plugins later

~~~
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
~~~

# Simulate with gazebo

This example demonstrates how to use the normal
gazebo executable with a plugin.

Run using gazebo itself with:

~~~
cd ~/gazebo_animatebox_tutorial
gazebo animated_box.world
~~~

In another terminal, use "gz topic" user interface to view the pose:

~~~
gz topic -v /gazebo/animated_box_world/pose/info
~~~

You should see a graphical interface that displays the pose of the box.

# Connect to a simulation with your own executable

Make sure Gazebo-classic is not running.

We will start Gazebo-classic as above, and then run the independent listener
executable that connects to Gazebo. The independent listener receives
the location and timestamp of the box and prints it out.

~~~
cd ~/gazebo_animatebox_tutorial
gazebo animated_box.world & ./build/independent_listener
~~~

# Run the simulation and connect with your own executable

Make sure Gazebo-classic is not running.

The integrated_main example demonstrates the following:

1. Start the box simulation.
2. Connect a listener to the simulation as part of the same executable.
3. The listener gets the timestamp and pose, then prints each out.

Run integrated_main:

~~~
cd ~/gazebo_animatebox_tutorial
./build/integrated_main animated_box.world
~~~

To view the simulation run the command:

~~~
gzclient
~~~

# SOURCE CODE

### independent_listener.cc

  Executable that will connect to a running simulation, receive updates from the pose information topic, and print the object position.

### integrated_main.cc

  Executable that will create a simulation, receive updates from the pose information topic, and print the object position.

### animated_box.cc

  Shared library plugin that defines the animation component of the simulation, moving the box that is in the world.

### animated_box.world

  XML file that defines the simulation physical world space and the single box that is in it.

### CMakeLists.txt

  CMake build script.
