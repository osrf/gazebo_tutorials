# Overview

A new code introspection utility has been introduced in Gazebo 8. This new
service allows clients to receive updates with the value of some requested
variables. The introspection service can be used to debug the state of internal
variables within Gazebo, plugins, or even stand-alone applications.

# Registering items

Two steps are involved when using the introspection service: registration and subscription. The registration phase registers a particular variable into the
introspection service. By registering a variable, you're making it
introspectable. Note that registering a variable will not trigger the
publication of any update or cause relevant overhead in the system.

The introspection manager is the entity that offers the ability to register
variables, referred to as "items". GzServer has an introspection manager instance running and we already
preregistered some items that allow us to introspect simulation time or the
position, velocity, and acceleration of models and links, among other items.

[[file:files/introspection_registration.png|300px]]

You can learn more about the introspection manager and its API by looking at the
[Util/IntrospectionManager class](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1util_1_1IntrospectionManager.html) in the Gazebo source code.

# Subscribing for receiving item updates

Once all potential introspectable items are registered, a client needs to
notify the introspection service that it's interested in one or multiple items.
Hence, the client needs to create a filter which is composed of the
introspection manager Id and the list of interested items.

[[file:files/introspection_subscription.png|640px]]

This operation creates a dedicated channel for communication
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
mkdir ~/gazebo_introspection_tutorial
cd ~/gazebo_introspection_tutorial
~~~

Download the code for the plugin, the watcher program and a `CMakeLists.txt` file:

~~~
wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/introspectable_plugin.cc
wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/watcher.cc
wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/CMakeLists.txt
~~~

Let's compile the code:

~~~
mkdir build && cd build
cmake ..
make
~~~

You should have a `libintrospectable_plugin.so` plugin and a `watcher` executable
ready for testing.

## Run the code

Download a world file that will load your world plugin:

~~~
cd ~/gazebo_introspection_tutorial
wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/empty.world
~~~

Start gazebo:

~~~
cd ~/gazebo_introspection_tutorial/build
GAZEBO_PLUGIN_PATH=`pwd` gazebo --verbose ../empty.world
~~~

Note that we are setting the `GAZEBO_PLUGIN_PATH` with the path to our build
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

First, let's take a look at the `introspectable_plugin`:

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/introspectable_plugin.cc' />

On `Load()`, we connect the world update event with our `OnUpdate()` function.
The rest of the code in `Load()` is registering the counter in the
introspection manager. You can see how we get an instance of the manager and
call `Register()`. We have to specify the type of our item (`int` in this case), a
string representation of the item (`data://my_plugin/counter`) and a callback.
In this example, the callback is a lambda function.

The introspection manager is going to associate this callback with
`data://my_plugin/counter`. Essentially, the string is the name of the item in
the manager. The callback lets the manager retrieve the next
value from this item. So, if there is any client interested in this value, the
manager will call this callback every time it needs an update. In the callback
we're directly returning the value of our member variable counter but you have
freedom to fill this function with any code that you need.

Now, let's study the watcher program:

<include from='/  \/\/ Use the introspection service/' to='/      std::chrono::seconds\(2\)\);/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/watcher.cc"/>

This executable is in charge of the subscription to a specific set of items that
are introspectable. We created the [`IntrospectionClient` class](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1util_1_1IntrospectionClient.html) to help all the
clients of the introspection service. As you can see, we instantiate one object
of type `IntrospectionClient`, and then, we wait for the introspection manager to
come online.

<include from='/  \/\/ Pick up the first manager/' to='/managerIds.begin\(\);/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/watcher.cc"/>

In theory, we could have multiple introspection managers running, although in
the case of Gazebo we will only have one. We're working under this assumption,
so we'll save the Id of the first introspection manager detected.

<include from='/  \/\/ sim_time is a pr-registered/' to='/  \/\/ The variables to watch are registered with the manager/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/watcher.cc"/>

This code block performs a sanity check to make sure that both items are
registered in the introspection manager.

<include from='/  \/\/ Create a filter for watching the items/' to='/waitForShutdown\(\);/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/introspection/files/watcher.cc"/>

This is the part where we notify our manager that we're interested in a set of
items (`simTime` and `counter`). `filterId` and `topic` are output variables. After
this function, the manager will create a channel of communication under the
topic `topic` with our custom updates. The `filterId` is a unique identifier for
our filter, in case we want to update it or remove it in the future.

Finally, we instantiate an `ignition::transport::Node` and we use it to subscribe to
our recently created topic. Note that we pass a `cb` callback as an argument. This is
the callback that will be periodically executed with the requested values.

