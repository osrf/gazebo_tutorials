#Tutorial: Manipulate a model using a model plugin#

##Prerequisites

  [Overview / HelloWorld](http://gazebosim.org/tutorials?tut=plugins_hello_world) Plugin Tutorial

> Note: If you're continuing from the previous tutorial, make sure you put in the proper `#include` lines for this tutorial that are listed below.

##Code

Source: [ gazebo/examples/plugins/model_push](https://bitbucket.org/osrf/gazebo/src/gazebo_2.2/examples/plugins/model_push)

Plugins allow complete access to the physical properties of models and their underlying elements (links, joints, collision objects). The following plugin will apply a linear velocity to its parent model.

~~~
cd ~/gazebo_plugin_tutorial
gedit model_push.cc
~~~

Plugin Code:
<include from="/#include/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/model_push/model_push.cc' />

### Compiling the Plugin ###

Assuming the reader has gone through the [[tutorials/2.2/plugins/HelloWorld|Hello WorldPlugin tutorial]], all that needs to be done is to add the following lines to `~/gazebo_plugin_tutorial/CMakeLists.txt`

<include from="/add_library/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/model_push/CMakeLists.txt' />

Compiling this code will result in a shared library, ~/gazebo_plugin_tutorial/build/libmodel_push.so, that can be inserted in a Gazebo simulation.

~~~
cd ~/gazebo_plugin_tutorial/build
cmake ../
make
~~~

### Running the Plugin ###

This plugin is used in the world file `examples/plugins/model_push/model_push.world`.

~~~
cd ~/gazebo_plugin_tutorial
gedit model_push.world
~~~

<include src='http://bitbucket.org/osrf/gazebo/raw/gazebo_2.2/examples/plugins/model_push/model_push.world' />

The hook to attach a plugin to a model is specified at the end of the model element block using:

+++<plugin name="model_push" filename="libmodel_push.so"/>+++


To start simulation, run

~~~
cd ~/gazebo_plugin_tutorial/
gzserver -u model_push.world
~~~

The `-u` option starts the server in a paused state.

In a separate terminal, start the gui

~~~
gzclient
~~~

Click on the play button in the gui to unpause the simulation, and you should see the box move.
