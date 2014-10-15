# Overview

This tutorial will explain how to use the Ignition Transport for sending new
joint commands to the hand and receiving state updates using C++.

We assume that you have already done the [installation step](TODO).

# Compile your controller

In this tutorial we include a very basic controller that applies a sinusoidal
wave to all the hand joints. First, you should compile your controller and link
it to the haptix_comm and ignition-transport libraries. Ignition-transport
provides the communication capabilities in C++. Besides the C API, Haptix_comm
exposes a set of protobuf messages that can be used in C++ programs.

1. Create a new directory named `haptix_controller_cpp` for this tutorial:

    ~~~
    mkdir ~/haptix_controller_cpp
    cd ~/haptix_controller_cpp
    ~~~

1. Download the source code of the controller and the cmake file:

    ~~~
    wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_comm/files/hx_controller.cpp
    wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_comm/files/CMakeLists.txt
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
    cd ~/haptix_controller/build
    ~~~

1. Start the controller:

    ~~~
    ./hx_controller
    ~~~

You should see your fingers following a smooth trajectory.

<iframe width="500" height="313" src="//player.vimeo.com/video/108959804" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

# The code explained

<include from='/int main/' to='/return -1;\n  }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_cpp/files/hx_controller.c' />

The HAPTIX C API is composed of two C function calls: `hx_getdeviceinfo()` and
`hx_update()`. `hx_getdeviceinfo()` requests information from a given device.
In this tutorial, our device is a hand simulated in Gazebo. Note that this call
blocks until the response is received.

We use the `hxGAZEBO` constant to specify that the target device is a hand inside a Gazebo
simulation. For other available targets check the [haptix_comm API](https://bitbucket.org/osrf/haptix_comm/src/cfd7e09c00ad045c0ee99a871f786971dc527fc5/include/haptix/comm/haptix.h?at=default). The second parameter of `hx_getdeviceinfo()` is a `hxDeviceInfo` struct that
contains the number of motors, joints, contact sensors, IMUs and joint limits
for the requested device. If we have a valid response, the returned value is `hxOK`.

We have included in our example a helper function `printDeviceInfo()` that will
print all the received fields for debugging purposes.

<include from='/  // Send commands/' to='/    usleep\(10000\);\n  }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_comm/files/hx_controller.c' />

Once we confirm the device information we can start sending commands for
controlling the hand. The function `hx_update()` is in charge of sending a new
command and receiving the current state of the hand.

First of all, we need to fill a `hxCommand` struct that contains the positions,
velocities, and gains for each joint. Check the [haptix_comm API](https://bitbucket.org/osrf/haptix_comm/src/cfd7e09c00ad045c0ee99a871f786971dc527fc5/include/haptix/comm/haptix.h?at=default)
for a detailed view of the `hxCommand` struct. In our case, we are modifying the
position of all the joints according to a sinusoidal function.

The function `hx_update()` accepts a first argument that is the target device
where the command will be sent (similar to `hx_getdeviceinfo()`). The second
parameter `cmd` is the command that we want to send to the device, which we already
filled in. A third output command `sensor` is passed to the function; it will contain the
state of the hand after applying the command.

We have included a helper function `printState()` that shows all the state
information for debugging purposes. Similar to `hx_getdeviceinfo()`, the function
`hx_update()` returns `hxOK` when the command was successfully sent and the
state has been received.
