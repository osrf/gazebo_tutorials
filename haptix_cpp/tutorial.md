# Overview

This tutorial will explain how to use the Ignition Transport C++ library for
sending new joint commands to the hand and receiving state updates.

We assume that you have already done the [installation steps](TODO). It's highly
recommended to go through the Ignition-Transport tutorials before continuing to
get familiar with some of the concepts used in the library.


# Compile your controller

In this tutorial we include a very basic controller that applies a sinusoidal
wave to all the hand joints. First, you should compile your controller and link
it to the `haptix_comm` and `ignition-transport` libraries. `Ignition-transport`
provides the communication capabilities in C++. Besides the C API, Haptix_comm
exposes a set of protobuf messages that can be used in C++ programs.

1. Create a new directory named `haptix_controller_cpp` for this tutorial:

    ~~~
    mkdir ~/haptix_controller_cpp
    cd ~/haptix_controller_cpp
    ~~~

1. Download the source code of the controller and the cmake file:

    ~~~
    wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_cpp/files/hx_controller.cpp
    wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_cpp/files/CMakeLists.txt
    ~~~

1. Create a build directory and compile the source code.

    ~~~
    mkdir build
    cd build
    cmake ..
    make
    ~~~

Now, we are ready to test our controller with the HAPTIX simulator.

# Running the simulation with your controller

1. In a terminal, start the HAPTIX simulation:

    ~~~
    roslaunch handsim_gazebo handsim.launch
    ~~~

1. In a different terminal, go to the build directory where you have your
controller executable:

    ~~~
    cd ~/haptix_controller_cpp/build
    ~~~

1. Start the controller:

    ~~~
    ./hx_controller
    ~~~

You should see your fingers following a smooth trajectory.

<iframe width="500" height="313" src="//player.vimeo.com/video/108959804" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

# The code explained

<include from='/int main/' to='/return -1;\n  }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_cpp/files/hx_controller.cpp' />

The HAPTIX Gazebo plugin advertises two services with topic names
`/haptix/gazebo/GetDeviceInfo` and `/haptix/gazebo/Update`. Clients can request
service calls to `/haptix/gazebo/GetDeviceInfo` for receiving information about
the simulated device in Gazebo. By requesting service calls to the topic
'/haptix/gazebo/Update`, clients can send new joint commands and receive the
current hand state.

During the first part of the main program we declare multiple variables,
including a transport node and haptix messages. The first call to
`hxNode.Request()` makes a blocking service request to the
`/haptix/gazebo/GetDeviceInfo` service. The second argument is the input
parameter passed to the service. Although we do not need an input parameter,
we need to pass it and it has to be initialized. Future versions of
`Ignition-transport` will allow empty input parameters. The next argument is a
timeout expressed in milliseconds. This is the maximum time that the request
will be waiting for a response. The next argument is the output parameter and
in our case it will contain the device information sent from our simulated
hand. The output `result` parameter will flag the result of the operation
(true if the operation succeed of false otherwise). The return value of the
request will tell the caller if the operation timed out or reached the service
provider.

We have included in our example a helper function `printDeviceInfo()` that will
print all the received fields for debugging purposes. Inside `printDeviceInfo()`
you can see how to read the different fields from a protobuf message.

<include from='/  // Set the service name for requesting a joint update/' to='/    usleep\(10000\);\n  }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_cpp/files/hx_controller.cpp' />

Once we confirm the device information we can start sending commands for
controlling the hand. The next call to `hxNode.Request()` is located inside a
loop that runs approximately at 100Hz. This is the main control loop that sends
joint commands to the simulated hand and receive its state. The first argument
is the name of the service in charge of receiving new joint commands. The second
argument is the Command message containing the new command to send to the hand.
Next, we use a timeout in milliseconds to set the maximum waiting time for the
response. The forth parameter contains the output parameter of the call and in
this case is filled in with the state information coming from the hand. Finally,
the last argument will contain the result of the request (true if the request
succeed of false otherwise). The return value of the request tells the caller if
the operation timed out or reached the service provider.

We have included a helper function `printState()` that shows all the state
information for debugging purposes.

The protobuf message definition for hxCommand, hxDevice and hxSensor can be
found [here](https://bitbucket.org/osrf/haptix_comm/src/cfd7e09c00ad045c0ee99a871f786971dc527fc5/msg/?at=default).
