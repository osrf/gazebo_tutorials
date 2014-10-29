# Overview

This tutorial will explain how to use Matlab or Octave for
sending new joint commands to the hand and receiving state updates.

We assume that you have already done the [installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix).

# Start the Gazebo simulation

Open a new terminal in the machine running Gazebo and start the HAPTIX simulation:

~~~
gazebo worlds/arat_test.world
~~~

# Run your controller in Matlab or Octave

The HAPTIX client library SDK includes two `mex` files that allow you to run
the functions `hx_getdeviceinfo()` and `hx_update()` from your Matlab/octave
console or from a .m file.

Open the file [`hx_matlab_controller.m`](ToDo) in Matlab/Octave. Then, type in
the Matlab/Octave prompt:

~~~
hx_matlab_controller
~~~

# Controller visualization.

While your controller is running, you should see your fingers following a smooth
trajectory in Gazebo.

<iframe width="500" height="313" src="//player.vimeo.com/video/108959804" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

# The code explained

<include from='/int main/' to='/printDeviceInfo\(.deviceInfo\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_comm/files/hx_controller.c' />

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
