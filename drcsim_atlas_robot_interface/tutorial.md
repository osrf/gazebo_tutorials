# Introduction

This tutorial demonstrates instructions to build a library that can be used as a stand-in for the
Boston Dynamics AtlasRobotInterface v3.0.0 library.  It implements the same API as that library but on
the back end, instead of talking to a physical Atlas, it talks to 
Gazebo/DRCSim via ROS.

To try it out:

## Configure and build

* Download AtlasRobotInterface v3.0.0 from your team files portal and unpack it somewhere.  Let's call that directory /work/AtlasRobotInterface_3.0.0.

* Build and install drcsim as usual (or you could install the latest 4.2.x package via apt-get).

* Source the drcsim setup file, e.g.:

    ~~~
    source /usr/share/drcsim/setup.sh
    ~~~

* Create a ros package for atlas_interface and cd into it:

    ~~~
    catkin_create_pkg atlas_interface roscpp atlas_msgs sensor_msgs osrf_msgs std_msgs rosgraph_msgs cmake_modules
    cd atlas_interface
    ~~~

* Add exports to the [package.xml](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/package.xml) if we want to have other files depned on this package:
    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/package.xml' />

* Create a `src` directory and add [src/atlas_interface.cpp](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/src/atlas_interface.cpp).

    ~~~
    mkdir src
    gedit src/atlas_interface.cpp
    ~~~

* Populate it with content from below
    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/src/atlas_interface.cpp' />

* To use the library `libatlas_interface.so` we created here, create an `examples` directory and add [examples/example.cpp](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/examples/example.cpp).

    ~~~
    mkdir examples
    gedit examples/example.cpp
    ~~~

* Populate it with content from below:
    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/examples/example.cpp' />

* Also create a launch file [examples/example.launch](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/examples/example.launch)

    ~~~
    gedit examples/example.launch
    ~~~

* Populate it with content from below:
    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/examples/example.launch' />

* Update the [CMakeLists.txt](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/CMakeLists.txt) to look like below:
    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_robot_interface/files/atlas_interface/CMakeLists.txt' />

* Add this package's directory (`atlas_interface`) to your ROS_PACKAGE_PATH, e.g.:

    ~~~
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ~~~

* Set the `ATLAS_ROBOT_INTERFACE_ROOT` environment variable to point to where
you unpacked AtlasRobotInterface, e.g.:

    ~~~
    export ATLAS_ROBOT_INTERFACE_ROOT=/work/AtlasRobotInterface_3.0.0
    ~~~

* Build this package:

    ~~~
    mkdir build
    cd build
    cmake ..
    make
    ~~~

## Running

* Try the example launch file:

    ~~~
    roslaunch atlas_interface example.launch
    ~~~

That will bring up Atlas in an empty world, with the example program
(example/example.cpp) running.  The intent is that the example is the kind of
user code that is normally linked against the AtlasRobotInterface library and
used with a physical robot.  That same code can instead be linked against 
atlas_interface and be used with a simulated robot.

## Development / testing

* The details of integrating this library into your system will depend on your
code and build system.  In general, the idea is to compile against the headers
provided by AtlasRobotInterface, but link against libatlas_interface.so,
provided here.
