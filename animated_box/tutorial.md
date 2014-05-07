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

## Animate box plugin

Start by creating the plugin.

~~~
cd ~/gazebo_animatebox_tutorial
gedit animate_box.cc
~~~

Copy and paste the following into [animate_box.cc](https://bitbucket.org/osrf/gazebo/src/issue_1114_animate_pose/examples/stand_alone/animated_box/animated_box.cc):

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo/raw/issue_1114_animate_pose/examples/stand_alone/animated_box/animated_box.cc' />


Create a CMakeLists.txt file that will build the plugin.

~~~
gedit CMakeLists.txt
~~~

Copy an past the following into [CMakeLists.txt](https://bitbucket.org/osrf/gazebo/src/issue_1114_animate_pose/examples/stand_alone/animated_box/CMakeLists.txt):

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo/raw/issue_1114_animate_pose/examples/stand_alone/animated_box/CMakeLists.txt' />

Build the plugin

~~~
mkdir build
cd build
cmake ../
make 
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
~~~






To build on platforms with make run:

    mkdir build
    cd build

    cmake ..
    make -j4
        
    # make sure gazebo can load the plugins later
    export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
    
    cd ..



## Simulate with gazebo
This example demonstrates how use the normal 
gazebo executable with a plugin.

Run using gazebo itself with:

      gazebo path/to/animated_box.world

Make sure to keep gazebo running in the background.

To use "gztopic" user interface to view the pose, run the command:

      gztopic view /gazebo/animated_box_world/pose/local/info
  
So that as the simulation is running you can see a 
printout of the lcoation and timestamp.


## Connect to a simulation with your own executable

Run the simulation with gazebo (split off on a separate bash script thread)
as above, then run the independent listener executable that
connects to the simulation, receives the location and timestamp,
and prints it out.
 
    gazebo path/to/animated_box.world &
      
    ./build/independent_listener


## Run the simulation and connect with your own executable

The integrated_main example demonstrates the folowing:

1. start the box simulation 
2. connects a listener to it that is part of the same executable. 
3. The listner gets the timestamp and pose, then prints each out.

Run integrated_main:
   
    ./build/integrated_main


To view the simulation run the command:

    gzclient


## SOURCE CODE

### listener.cc

  executable that will connect to a running simulation and receive + print position output
  
### integrated_main.cc

  executable that will create a simulation and receive + print position output

### animated_box.cc

  shared library plugin that defines the animation component of the simulation, moving the box that is in the world.
  
### animated_box.world

  XML file that defines the simulation physical world space and the single box that is in it.

### CMakeLists.txt
  
  CMake build script
