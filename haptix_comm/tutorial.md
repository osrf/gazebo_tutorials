# Overview

This tutorial will explain how to use the C client library `haptix-comm` for
requesting a description of the hand, sending new joint commands, and receiving state updates.

We assume that you have already done the [installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix).


# Compile your controller

In this tutorial we include a very basic controller that applies a sinusoidal
function to all the hand joints. First, you should compile your controller and
link it to the haptix-comm library.

## Windows

Before opening Visual Studio you need to do some network configuration to specify
the IP address that you will use for communicating with Gazebo.

Click on the `Start button` and type `cmd` in the `Search box`. Once you are in
the terminal run the following command:

~~~
ipconfig
~~~

Look for the `IPv4 Address` inside the network interface that you want to use
for communicating with Gazebo. Here, we will use the wired interface, so you
should look for `Ethernet adapter Local Area Connection`.

%%%
[[file:files/ipconfig.png|600px]]
%%%

Click on the `Start button` and search for `environment variables`.
Click on `Edit the system environment variables`, and then, click on the button
`Environment variables`. Click on the `New` button from the `User variables`
section to create a new environment variable with name `IGN_IP`. **The value
should be the WIRED IP address of the network interface that you are using to
communicate with Gazebo**.

Open Visual Studio 2013 and create a new project for your hand controller. Click
on `File`->`New Project`->`Visual C++`->`Win32 Console Application`. Select an
appropriate name for your project, for example `MyBasicController`. Click `OK`,
and then, click on `Finish`.

Replace the source code from your current project with our basic controller. Copy
the code from [**here**](http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c) and paste it in your current project.

Add the following line at the begginning of your source code:

~~~
#include "stdafx.h"
~~~

Open the Property Manager view by clicking on `View`->`Other Windows`->`Property
Manager`. This will allow you to use the property sheet provided by the HAPTIX
library SDK. Move to the `Property Manager` tab and right click on your project.
Then, select `Add Existing Property Sheet...`. A new popup window
will appear. Browse to the folder where you downloaded the HAPTIX client library
SDK and select the property sheet named `haptix-comm`. This will handle all the
dependencies for your project.

Switch the target build type from `Debug` to `Release` using the upper toolbar.
Then, click on `BUILD`->`Build Solution` to build your controller.

## Linux

1. Install the following packages required for downloading and compiling the
example:

    ~~~
    sudo apt-get install wget cmake build-essential
    ~~~

1. Create a new directory named `haptix_controller` for this tutorial:

    ~~~
    mkdir ~/haptix_controller
    cd ~/haptix_controller
    ~~~

1. Download the source code and the cmake file for the controller:

    ~~~
    wget http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c
    wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_comm/files/CMakeLists.txt
    ~~~

1. Create a build directory and compile the source code.

    ~~~
    mkdir build
    cd build
    cmake ..
    make
    ~~~

# Running the simulation with your controller

Now, we are ready to test our controller with the HAPTIX simulator. Open a new
terminal on the Linux machine running Gazebo and start the HAPTIX simulation:

~~~
gazebo worlds/arat.world
~~~

## Windows

Your code should be ready to be executed using Visual Studio. Click on `DEBUG`->
`Start Without Debugging...` (alternatively you can press Ctrl+F5).

## Linux

1. Go to the Linux machine where you want to run your controller code. Open a
terminal and go to the `build/` directory where you have your controller
executable:

    ~~~
    cd ~/haptix_controller/build
    ~~~

1. Run `ifconfig` to show the list of network interfaces that you currently have.

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

1. Start the controller filling `IGN_IP` with the appropriate IP address (depending if
you want to use your wired or wireless connection):

    ~~~
    IGN_IP=172.23.2.37 ./hx_controller
    ~~~

# Controller visualization

While your controller is running, you should see your fingers following a smooth
trajectory in Gazebo.

<iframe width="500" height="313" src="//player.vimeo.com/video/108959804" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

# The code explained

<include from='/int main/' to='/printDeviceInfo\(.deviceInfo\)/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c' />

The HAPTIX C API is composed of two C function calls: `hx_getdeviceinfo()` and
`hx_update()`. `hx_getdeviceinfo()` requests information from a given device.
In this tutorial, our device is a hand simulated in Gazebo. Note that this call
blocks until the response is received.

We use the `hxGAZEBO` constant to specify that the target device is a hand inside a Gazebo
simulation. For other available targets check the [haptix-comm API](https://bitbucket.org/osrf/haptix-comm/src/cfd7e09c00ad045c0ee99a871f786971dc527fc5/include/haptix/comm/haptix.h?at=default). The second parameter of `hx_getdeviceinfo()` is a `hxDeviceInfo` struct that
contains the number of motors, joints, contact sensors, IMUs and joint limits
for the requested device. If we have a valid response, the returned value is `hxOK`.

We have included in our example a helper function `printDeviceInfo()` that will
print all the received fields for debugging purposes.

<include from='/  // Send commands/' to='/    usleep\(10000\);\n  }/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c' />

Once we confirm the device information we can start sending commands for
controlling the hand. The function `hx_update()` is in charge of sending a new
command and receiving the current state of the hand.

First of all, we need to fill a `hxCommand` struct that contains the positions,
velocities, and gains for each joint. Check the [haptix-comm API](https://bitbucket.org/osrf/haptix-comm/src/cfd7e09c00ad045c0ee99a871f786971dc527fc5/include/haptix/comm/haptix.h?at=default)
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
