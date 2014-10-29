# Overview

This tutorial will explain how to use Matlab or Octave for
sending new joint commands to the simulated hand and receiving state updates.

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

<include from='/int main/' to='/printDeviceInfo\(.deviceInfo\)/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_matlab_controller.m' />

The HAPTIX C API is composed of two C function calls: `hx_getdeviceinfo()` and
`hx_update()`. `hx_getdeviceinfo()` requests information from a given device.
In this tutorial, our device is a hand simulated in Gazebo. Note that this call
blocks until the response is received.

The result value of `hx_getdeviceinfo() is a struct containing the number of
motors, joints, contact sensors, IMUs and joint limits for the requested device,
 as well as the result of the request. If we have a valid response, the
 returned value is 0.

<include from='/  // Send commands/' to='/    usleep\(10000\);\n  }/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_matlab_controller.m' />

Once we confirm the device information we can start sending commands for
controlling the hand. The function `hx_update()` is in charge of sending a new
command and receiving the current state of the hand.

First of all, we need to fill a command struct that contains the positions,
velocities, and gains for each joint. It is important to use the same names for
the fields that we are using in this example.

The function `hx_update()` accepts an argument that is the command that we want
to send to the device, which we already filled in. The returned value will
contain the state of the hand after applying the command and the result of the
request (0 on success).
