# Overview

A new code introspection utility has been introduced in Gazebo 8. This new
service allow clients to receive updates with the value of some requested
variables. The introspection service can be used to debug the state of internal
variables within Gazebo, plugins, or even stand-alone applications.

# Registering items

Two steps are involved when using the introspection service: registration and subscription. The registration phase registers a particular variable into the
introspection service. By registering a variable, you're making it
introspectable. Note that registering a variable will not trigger the
publication of any update or cause relevant overhead in the system.

The introspection manager is the entity that offers the ability to register
items. GzServer has an introspection manager instance running and we already
preregistered some items that allow us to introspect simulation time or the
position, velocity, and acceleration of models and links, among other items.

[[file:files/introspection_registration.png|300px]]

You can learn more about the introspection manager and its API by looking at the
Util/IntrospectionManager class in the Gazebo source code.

# Receiving item updates

Once all potential introspectable items are registered, a client needs to
notify the introspection service that it's interested in one or multiple items.
Hence, the client needs to create a filter, where all the items are specified
and the Id of an introspection manager is also passed.

[[file:files/introspection_subscription.png|640px]]

This operation is essentially creating a dedicated channel of communication
between the introspection manager and the client. The channel contains
messages with the value of the variables specified in the filter.
If one or more variables were not registered, they will not be received.

# Example: introspecting your plugin

In this example, we are going to create a very simple world plugin that
increments an integer variable at every world update. The interesting part of
this plugin is that we are going to make the counter introspectable. We will
write an executable named watcher that will display the value of the simulation
time and our custom counter.


## Compile the plugin and watcher executable

Create a new directory for storing all files in this tutorial:

~~~
$ mkdir ~/gazebo_instrospection_tutorial
$ cd ~/gazebo_instrospection_tutorial
~~~

Download the code of the plugin from [here](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/introspectable_plugin.cc) and
the code of the watcher from [here](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_tactors/files/watcher.cc). You'll also need a [CMakeLists.txt](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_tactors/files/CMakeLists.txt) file.

Let's compile the code:

~~~
$ mkdir build && cd build
$ cmake ..
$ make
~~~

You should have a libintrospectable_plugin.so plugin and a watcher executable
ready for testing.

## Run the code

Download a world file from [here](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/empty.world), that will load your world plugin.

Start gazebo:

~~~
$ GAZEBO_PLUGIN_PATH=`pwd` gazebo --verbose ../empty.world
~~~

Note that we are setting the GAZEBO_PLUGIN_PATH with the path to our current
directory in order to help Gazebo finding our plugin. Once Gazebo is ready,
execute the following command on a new terminal:

~~~
cd ~/gazebo_introspection_tutorial/build
./watcher
~~~

You should observe an output similar to the following block:

~~~
param {
  name: "data://world/default?p=time/sim_time"
  value {
    type: TIME
    time_value {
      sec: 12
      nsec: 616000000
    }
  }
}

param {
  name: "data://my_plugin/counter"
  value {
    type: INT32
    int_value: 12617
  }
}

...
~~~

As you can see, watcher is continuously printing the value of the simulation
time and the counter.

## Walkthrough

First, let's take a look at the introspectable_plugin:

<include from='/void Load/' to='/\n/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/introspection/introspection/files/introspectable_plugin.cc' />
