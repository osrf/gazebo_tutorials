#Prerequisites

  [Overview / HelloWorld](/tutorials?tut=plugins_hello_world) Plugin Tutorial

# Overview

Source: [gazebo/examples/plugins/system\_gui\_plugin](https://github.com/osrf/gazebo/blob/gazebo5/examples/plugins/system_gui_plugin)

This tutorial will create a source file that is a system plugin for gzclient
designed to save images into the directory `/tmp/gazebo_frames`.

# Source code

We'll start with the source file. Create a file called `system_gui.cc` with the following contents:

~~~
$ cd ~/gazebo_plugin_tutorial
$ gedit system_gui.cc
~~~

Copy the following into `system_gui.cc`

<include from='/#include/' src='http://github.com/osrf/gazebo/raw/gazebo5/examples/plugins/system_gui_plugin/system_gui.cc' />

Both the `Load` and `Init` functions must not block. The `Load` and `Init` functions are called at startup, before Gazebo is loaded.

On the first `Update`, we get a pointer to the user camera (the camera used in the graphical interface) and enable saving of frames.

1.  Get the user camera

        this->userCam = gui::get_active_camera();

2.  Enable save frames

        this->userCam->EnableSaveFrame(true);

3.  Set the location to save frames

        this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");

## Compiling Camera Plugin

Assuming the reader has gone through the [Hello WorldPlugin tutorial](/tutorials?tut=plugins_hello_world) all that needs to be done is to add the following lines to `~/gazebo_plugin_tutorial/CMakeLists.txt`

<include from="/add_library/" src='http://github.com/osrf/gazebo/raw/gazebo5/examples/plugins/system_gui_plugin/CMakeLists.txt' />

Rebuild, and you should end up with a libsystem_gui.so library.

~~~
$ cd ~/gazebo_plugin_tutorial/build
$ cmake ../
$ make
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


Note: Remember to also terminate the background server process after you quit the client. In the same terminal, bring the process to foreground:

~~~
$ fg
~~~

and press `Ctrl-C` to abort the process. Alternatively, just kill the `gzserver` process:

~~~
$ killall gzserver
~~~
