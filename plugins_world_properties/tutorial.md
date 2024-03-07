This plugin example programmatically modifies gravity.

# Prerequisites:

 * [Model Manipulation](/tutorials/?tut=plugins_model)
 * [Plugin Tutorial](/tutorials/?tut=plugins_hello_world)

# Setup:

Source: [gazebo/examples/plugins/world_edit](https://github.com/osrf/gazebo/blob/gazebo9/examples/plugins/world_edit)

Use the `gazebo_plugin_tutorial` from the previous plugin tutorials

    $ mkdir ~/gazebo_plugin_tutorial; cd ~/gazebo_plugin_tutorial

Create a file called `~/gazebo_plugin_tutorial/world_edit.world`

    $ gedit world_edit.world

Add the following contents to it:

<include src='http://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/world_edit/world_edit.world' />


# Code

Create a file called `~/gazebo_plugin_tutorial/world_edit.cc`:

    $ gedit world_edit.cc

Add the following content to it:

<include from="/#include/" src='http://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/world_edit/world_edit.cc'/>


## The Code Explained

<include from="@  * // Create a new transport node@" to="/node.*Init.*/" src='http://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/world_edit/world_edit.cc' />

We create a new node pointer, and initialize it to using the world name.
The world name allows the node to communicate with one specific world.
<include from="@  *// Create a publisher@" to="/Advertise.*/" src='http://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/world_edit/world_edit.cc' />

A publisher is created for sending physics messages on the "~/physics" topic.
<include from="/  * msgs::Physics physicsMsg/" to="/physicsPub.*Publish.*/" src='http://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/world_edit/world_edit.cc' />

A physics message is created, and the step time and gravity are altered.
This message is then published to the "~/physics" topic.

# Build

Assuming the reader has gone through the [Plugin Overview Tutorial](/tutorials/?tut=plugins_hello_world), all that needs to be done in addition is save the above code as `~/gazebo_plugin_tutorial/world_edit.cc` and add the following lines to `~/gazebo_plugin_tutorial/CMakeLists.txt`

<include from="/add_library/" src='http://github.com/osrf/gazebo/raw/gazebo9/examples/plugins/world_edit/CMakeLists.txt' />

Compiling this code will result in a shared library, `~/gazebo_plugin_tutorial/build/libworld_edit.so`, that can be inserted in a Gazebo simulation.

~~~
$ mkdir ~/gazebo_plugin_tutorial/build
$ cd ~/gazebo_plugin_tutorial/build
$ cmake ../
$ make
~~~

# Run Tutorial

First you need to add the folder to the `GAZEBO_PLUGIN_PATH` environment variable:

~~~
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugin_tutorial/build/
~~~

Then in a terminal

~~~
$ cd ~/gazebo_plugin_tutorial
$ gazebo world_edit.world
~~~

You should see an empty world.

Now add a box to the world using the Box icon located above the render window.
The box should float up and away from the camera.
