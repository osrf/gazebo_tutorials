# Overview

This tutorial will create a source file that is a system plugin designed to save images into the director `/tmp/gazebo_frames`.

# Source code

We'll start with the source file. Create a file called `system_gui.cc` with the following contents:

~~~
$ mkdir ~/system_plugin
$ cd ~/system_plugin
$ gedit system_gui.cc
~~~

Copy the following into `system_gui.cc`

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo/raw/gazebo_4.0/examples/plugins/system_gui_plugin/system_gui.cc' />

Both the `Load` and `Init` functions must not block. The `Load` and `Init` functions are called at startup, before Gazebo is loaded.

On the first `Update`, we get a pointer to the user camera (the camera used in the graphical interface) and enable saving of frames.

1.  Get the user camera

        this->userCam = gui::get_active_camera();
2.  Enable save frames

        this->userCam->EnableSaveFrame(true);
3.  Set the location to save frames

        this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");

## Compiling Camera Plugin

Create a CMakeLists.txt 

~~~
$ gedit CMakeLists.txt 
~~~

and copy the following into the file:

~~~
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE-Terrain_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

add_library(system_gui SHARED system_gui.cc)
target_link_libraries(system_gui ${GAZEBO_libraries} ${OGRE_LIBRARIES})
~~~

Make a build directory, run cmake, and compile. You should end up with a libsystem_gui.so library.

Create a `build` directory

~~~
$ mkdir build; cd build
~~~

Compile the plugin

~~~
$ cmake ../; make
~~~

Make sure the location of the plugin is in your `$GAZEBO_PLUGIN_PATH`

~~~
$ export GAZEBO_PLUGIN_PATH=$PWD:$GAZEBO_PLUGIN_PATH
~~~

## Running Plugin

First start gzserver in the background:

~~~
$ gzserver &
~~~

Run the client with plugin:

~~~
$ gzclient -g libsystem_gui.so
~~~

Inside `/tmp/gazebo_frames` you should see many saved images from the current plugin.
