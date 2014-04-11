#Tutorial: Programmatic World Control#

This plugin example programmatically modifies gravity.

**Prerequisite**: See the [Plugins Overview](http://gazebosim.org/tutorials/?tut=plugins_hello_world) tutorial.

## Setup
Source: [gazebo/examples/plugins/world_edit](https://bitbucket.org/osrf/gazebo/src/gazebo_2.2/examples/plugins/world_edit)

1. Create a working directory

~~~
mkdir ~/world_edit; cd ~/world_edit
~~~

1. Create a file called `~/world_edit/world_edit.world`

~~~
gedit world_edit.world
~~~

Add the following contents to it:
<include src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/world_edit/world_edit.world' />


## Code ##

1.  Create a file called `~/world_edit/world_edit.cc`:

~~~
gedit world_edit.cc
~~~

Add the following content to it:
<include from="/#include/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/world_edit/world_edit.cc' />

### The Code Explained ###
<include from="/^.*Create a new transport node/" to="/node.*Init.*$/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/world_edit/world_edit.cc' />

We create a new node pointer, and initialize it to using the world name.
The world name allows the node to communicate with one specific world.
<include from="@  *// Create a publisher@" to="/Advertise.*/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/world_edit/world_edit.cc' />

A publisher is created for sending physics messages on the "~/physics" topic.
<include from="/^.*physicsMsg/" to="/physicsPub.*Publish.*$/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/world_edit/world_edit.cc' />

A physics message is created, and the step time and gravity are altered.
This message is then published to the "~/physics" topic.

## Build ##
Create a CMake script called `~/world_edit/CMakeLists.txt`:


~~~
gedit CMakeLists.txt
~~~

Copy the following content into it:
<include src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/world_edit/CMakeLists.txt' />

Create a `build` directory

~~~
mkdir build; cd build
~~~

Compile the plugin

~~~
cmake ../; make
~~~

## Run Tutorial ##

First you need to add the folder to the `GAZEBO_PLUGIN_PATH` environment variable:

~~~
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/world_edit/build/
~~~

Then in a terminal

~~~
cd ~/world_edit/build
gazebo ../world_edit.world
~~~

You should see an empty world.

Now add a box to the world using the Box icon located above the render window.
The box should float up and away from the camera.
