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

~~~
class ModelPush : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));

    // Introspection callback.
    auto fCounterValue = [this]()
    {
      return this->counter;
    };

    // Register the counter element.
    gazebo::util::IntrospectionManager::Instance()->Register
    <int>("data://my_plugin/counter", fCounterValue);
  }

  // Called by the world update start event
  public: void OnUpdate()
  {
    ++this->counter;
  }

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  // A counter for testing the introspection capabilites.
  private: int counter = 0;
};
~~~


On `Load()`, we connect the world update event with our `OnUpdate()` function.
The rest of the code in `Load()` is registering the counter in the
introspection manager. You can see how we get an instance of the manager and
call `Register()`. We have to specify the type of our item (`int` in this case), a
string representation of the item ("data://my_plugin/counter") and a callback.
In this example, the callback is a lambda function.

The introspection manager is going to associate this callback with
"data://my_plugin/counter". Essentially, the string is the name of the item in
the manager. The callback is the way that the manager has to retrieve the next
value from this item. So, if there is any client interested in this value, the
manager will call this callback every time it needs an update. In the callback
we're directly returning the value of our member variable counter but you have
freedom to fill this function with any code that you need.

Now, let's study the watcher program:

~~~
// Use the introspection service for finding the "sim_time" item.
gazebo::util::IntrospectionClient client;

// Wait for the managers to come online.
std::set<std::string> managerIds = client.WaitForManagers(
    std::chrono::seconds(2));
~~~

This executable is in charge of the subscription to a specific set of items that
are introspectable. We created the class `IntrospectionClient` to help all the
clients of the introspection service. As you can see, we instantiate one object
of type `IntrospectionClient`, and then, we wait for the introspection manager to
come online.

~~~
// Pick up the first manager.
std::string managerId = *managerIds.begin();
~~~

In theory, we could have multiple introspection managers running, although in
the case of Gazebo we will only have one. We're working under this assumption,
so we'll save the Id of the first introspection manager detected.

~~~
// The 'sim_time' is not registered.
if (!client.IsRegistered(managerId, simTime))
{
  std::cerr << "The sim_time item is not registered on the manager.\n";
  return -1;
}

// The 'counter' is not registered.
if (!client.IsRegistered(managerId, counter))
{
  std::cerr << "The counter item is not registered on the manager.\n";
  return -1;
}
~~~

This code block performs a sanity check to make sure that both items are
registered in the introspection manager.

~~~
// Create a filter for watching the items.
  std::string filterId, topic;
  if (!client.NewFilter(managerId, {simTime, counter}, filterId, topic))
    return -1;

  // Let's subscribe to the topic for receiving updates.
  ignition::transport::Node node;
  node.Subscribe(topic, cb);

  /// zZZZ.
  ignition::transport::waitForShutdown();
~~~

This is the part where we notify our manager that we're interested on a set of
topics (`simTime` and `counter`). `filterId` and `topic` are output variables. After
this function, the manager will create a channel of communication under the
topic `topic` with our custom updates. The `filterId` is a unique identifier for
our filter, in case we want to update it or remove it in the future.

Next, we instantiate an `ignition::transport::Node` and we use it to subscribe to
our recently created topic. Note that we pass a callback as an argument. This is
the callback that will be periodically executed with the requested values.

