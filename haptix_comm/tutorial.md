# Overview

This tutorial will explain how to use the HAPTIX-Comm C-API for sending new
joint commands to the hand and receiving state updates.

# Simulation setup

We assume that you have already done the installation step.

# Compile your controller.

In this tutorial we include a very basic controller that applies a sinusoidal
wave to all the hand joints. First, you should compile your controller and link
it against the haptix_comm library.

1. Create a new directory named `haptix_controller` for this tutorial:

    ~~~
    mkdir ~/haptix_controller
    cd ~/haptix_controller
    ~~~

1. Download the source code of the controller and the cmake file:

    ~~~
    wget http://bitbucket.org/osrf/TODO
    wget http://bitbucket.org/osrf/TODO
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

You should see your fingers moving following a smooth trajectory in an infinite
loop.

# The code explained

<include from='/int main/' to='/return -1;\n  }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_comm/files/hx_controller.c' />

The HAPTIX C API is composed by two C function calls: `hx_getdeviceinfo()` and
`hx_update()`. `hx_getdeviceinfo()` requests information to a given device.
In this tutorial, our device is a hand simulated in Gazebo. Note that this call
blocks for a certain amount of time until the response is received.

We use the `hxGAZEBO` to specify that the target device is inside a Gazebo
simulation. For other targets available, check the [haptix_comm API](https://bitbucket.org/osrf/haptix_comm/src/cfd7e09c00ad045c0ee99a871f786971dc527fc5/include/haptix/comm/haptix.h?at=default). The second parameter of `hx_getdeviceinfo()` is a `hxDeviceInfo` struct. Check the [haptix_comm API](https://bitbucket.org/osrf/haptix_comm/src/cfd7e09c00ad045c0ee99a871f786971dc527fc5/include/haptix/comm/haptix.h?at=default)
that contains the number of motors, joints, contact sensors, IMUs and joint
limits for the requested device. If we have a response, the returned value is
`hxOK`.
