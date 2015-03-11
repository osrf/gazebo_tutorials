# Overview

This tutorial will explain how to use Matlab in Windows or Octave in Linux for requesting a
description of the hand, sending new joint commands, and receiving state updates. The Matlab
and Octave systems have the same API, but the steps to run each system differ slightly.

We assume that you have already done the [installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix).

# Start the Gazebo simulation

Double-click on the `haptixStart` desktop icon.

# Run your controller in Matlab

Before opening Matlab you should make sure that the environment variable
`IGN_IP` is properly set. Go to this [this tutorial]
(http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix#Networkconfiguration), and scroll to the Network Configuration section for detailed instructions.

Open Matlab and click on the `Browse for folder` icon.

%%%
[[file:files/matlab.png|800px]]
%%%

A new pop-up window will appear. Browse to the folder `matlab\` inside the
directory where you unzipped the HAPTIX client library SDK.

The HAPTIX client library SDK includes one `mex` file that allows you to call
the functions `hx_connect()`, `hx_robot_info()`,
`hx_update()`, `hx_read_sensors()` and `hx_close()` from your Matlab/Octave
console or from a .m file.

Open the file `hx_matlab_controller.m` in Matlab. Then, type in
the Matlab Command Window:

~~~
hx_matlab_controller
~~~

You should see the arm move like in the controller visualization video below.

# Run your controller in Octave

First, install Octave if you haven't already:

~~~
sudo apt-get install octave
~~~

Then, change directories to the `octave` subdirectory of the `haptix-comm` install directory.
This directory should contain several `.m` files.

~~~
cd /usr/lib/x86_64-linux-gnu/haptix-comm/octave
~~~

If that directory does not exist, try:

~~~
cd /usr/lib/haptix-comm/octave
~~~

Start Octave:

~~~
octave
~~~

You should be able to call `hx_connect()`, `hx_robot_info()`, `hx_update()`, `hx_read_sensors()`,
and `hx_close()` from Octave (the parentheses are optional).

To run a controller for the simulated arm, type:

~~~
hx_matlab_controller
~~~

You should see the arm move like in the controller visualization video below.

# Controller visualization

While your controller is running, you should see your fingers following a smooth
trajectory in Gazebo.

<iframe width="500" height="313" src="//player.vimeo.com/video/108959804" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

# The code explained

<include lang='m' from='/counter =/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hx_matlab_controller.m' />

The HAPTIX Matlab/Octave API is composed of five functions: `hx_connect()`, `hx_robot_info()`,
`hx_update()`, `hx_read_sensors()` and `hx_close()`. `hx_connect()` and `hx_close()` are
optional for the Gazebo simulator, but are included for compatibility with MuJoCo.

`hx_robot_info()` requests information from a given device.
In this tutorial, our device is a hand simulated in Gazebo. Note that this call
blocks until the response is received.

The result value of `hx_robot_info()` is a struct containing the number of
motors, joints, contact sensors, IMUs and joint limits for the requested device.
It also contains the update rate, the frequency at which the device is updated.

Once we confirm the device information we can start sending commands for
controlling the hand. The function `hx_update()` is in charge of sending a new
command and receiving the current state of the hand.

First of all, we need to fill a command struct that contains the positions,
velocities, and gains for each joint. It is important to use the same names for
the fields that we are using in this example.
