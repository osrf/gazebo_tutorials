# Animated Box Simulation Demo

This tutorial creates a simulation world with a simple box that is animated
in a 10 second repeating loop where it slides around on the ground.

This tutorial also demonstrates several different ways of viewing,
accessing, and interacting with simulation using the Gazebo executable
or your own custom executable.

The simulated box broadcasts its pose,
and a callback is created to receive the pose
and print out the location and timestamp of the box.

## Setup

Create a working directory.

~~~
mkdir ~/gazebo_animatebox_tutorial
cd ~/gazebo_animatebox_tutorial
~~~

## Animate box code

Copy [animate_box.cc](https://bitbucket.org/osrf/gazebo/src/issue_1114_animate_pose/examples/stand_alone/animated_box/animated_box.cc), [independent_listener.cc](https://bitbucket.org/osrf/gazebo/src/issue_1114_animate_pose/examples/stand_alone/animated_box/independent_listener.cc), [integrated_main.cc](https://bitbucket.org/osrf/gazebo/src/issue_1114_animate_pose/examples/stand_alone/animated_box/integrated_main.cc), and [CMakeLists.txt](https://bitbucket.org/osrf/gazebo/src/issue_1114_animate_pose/examples/stand_alone/animated_box/CMakeLists.txt) into the current directory.

~~~
wget http://bitbucket.org/osrf/gazebo/raw/issue_1114_animate_pose/examples/stand_alone/animated_box/animated_box.cc
wget http://bitbucket.org/osrf/gazebo/raw/issue_1114_animate_pose/examples/stand_alone/animated_box/independent_listener.cc
wget http://bitbucket.org/osrf/gazebo/raw/issue_1114_animate_pose/examples/stand_alone/animated_box/integrated_main.cc
wget http://bitbucket.org/osrf/gazebo/raw/issue_1114_animate_pose/examples/stand_alone/animated_box/CMakeLists.txt
wget http://bitbucket.org/osrf/gazebo/raw/issue_1114_animate_pose/examples/stand_alone/animated_box/animated_box.world
~~~

Build the plugin

~~~
mkdir build
cd build
cmake ../
make 
~~~

Make sure Gazebo can load the plugins later

~~~
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
~~~


## Simulate with gazebo

This example demonstrates how use the normal 
gazebo executable with a plugin.

Run using gazebo itself with:

~~~
cd ~/gazebo_animatebox_tutorial
gazebo animated_box.world
~~~

In another terminal, use "gz topic" user interface to view the pose:

~~~
gz topic -v /gazebo/animated_box_world/pose/local/info
~~~

You should see a graphical interface that display the pose of the box.

## Connect to a simulation with your own executable

Make sure Gazebo is not running.

We will Gazebo as above, and then run the independent listener
executable that connects to Gazebo. The indepdent listener receives
the location and timestamp of the box and prints it out.

~~~
cd ~/gazebo_animatebox_tutorial
gazebo animated_box.world & ./build/independent_listener
~~~

## Run the simulation and connect with your own executable

Make sure Gazebo is not running.

The integrated_main example demonstrates the folowing:

1. start the box simulation 
2. connects a listener to it that is part of the same executable. 
3. The listner gets the timestamp and pose, then prints each out.

Run integrated_main:

~~~
cd ~/gazebo_animatebox_tutorial
./build/integrated_main animated_box.world
~~~

To view the simulation run the command:

~~~
gzclient
~~~

## SOURCE CODE

#### independent_listener.cc

  executable that will connect to a running simulation and receive + print position output
  
#### integrated_main.cc

  executable that will create a simulation and receive + print position output

#### animated_box.cc

  shared library plugin that defines the animation component of the simulation, moving the box that is in the world.
  
#### animated_box.world

  XML file that defines the simulation physical world space and the single box that is in it.

#### CMakeLists.txt
  
  CMake build script
