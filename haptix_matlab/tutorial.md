# Overview

This tutorial will explain how to use Matlab or Octave for requesting a
description of the hand, sending new joint commands, and receiving state updates.

We assume that you have already done the [installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix).

# Start the Gazebo simulation

First, be sure about the IP address used by the network interface connecting
both machines.  Open a terminal and run `ifconfig` to show the list of network
interfaces that you currently have:

    ~~~
    ifconfig
    ~~~
    %%%
    eth1  Link encap:Ethernet  HWaddr 90:2b:34:d7:51:7a
          inet addr:172.23.2.37  Bcast:172.23.3.255  Mask:255.255.252.0
          inet6 addr: fe80::922b:34ff:fed7:517a/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:13253086 errors:0 dropped:4 overruns:0 frame:0
          TX packets:6567550 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:7740317824 (7.7 GB)  TX bytes:1601307286 (1.6 GB)
          Interrupt:20 Memory:f7500000-f7520000

    lo    Link encap:Local Loopback
          inet addr:127.0.0.1  Mask:255.0.0.0
          inet6 addr: ::1/128 Scope:Host
          UP LOOPBACK RUNNING  MTU:65536  Metric:1
          RX packets:3680508884 errors:0 dropped:0 overruns:0 frame:0
          TX packets:3680508884 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:0
          RX bytes:1098106845630 (1.0 TB)  TX bytes:1098106845630 (1.0 TB)
    %%%


Now, we are ready to test our controller with the HAPTIX simulator. Open a new
terminal on the Linux machine running Gazebo and start the HAPTIX simulation
using `IGN_IP` with the appropriate IP address (depending if you want to use
your wired or wireless connection):

~~~
IGN_IP=172.23.2.37 gazebo worlds/arat.world
~~~

# Run your controller in Matlab or Octave

Before opening Matlab/Octave you should make sure that the environment variable
`IGN_IP` is properly set. Check out [this tutorial]
(http://gazebosim.org/tutorials?tut=haptix_comm&cat=haptix) for detailed instructions.

Open Matlab and click on the `Browse for folder` icon.

%%%
[[file:files/matlab.png|800px]]
%%%

A new pop-up window will appear. Browse to the folder `matlab\` inside the
directory where you unzipped the HAPTIX client library SDK.

The HAPTIX client library SDK includes two `mex` files that allow you to call
the functions `hx_getdeviceinfo()` and `hx_update()` from your Matlab/Octave
console or from a .m file.

Open the file `hx_matlab_controller.m` in Matlab/Octave. Then, type in
the Matlab/Octave Command Window:

~~~
hx_matlab_controller
~~~

# Controller visualization

While your controller is running, you should see your fingers following a smooth
trajectory in Gazebo.

<iframe width="500" height="313" src="//player.vimeo.com/video/108959804" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

# The code explained

<include from='/counter =/' to='/end/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hx_matlab_controller.m' />

The HAPTIX Matlab API is composed of two mex function: `hx_getdeviceinfo()` and
`hx_update()`. `hx_getdeviceinfo()` requests information from a given device.
In this tutorial, our device is a hand simulated in Gazebo. Note that this call
blocks until the response is received.

The result value of `hx_getdeviceinfo()` is a struct containing the number of
motors, joints, contact sensors, IMUs and joint limits for the requested device,
 as well as the result of the request. If we have a valid response, the
 returned value is 0.

<include from='/while counter/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hx_matlab_controller.m' />

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
