# Overview

**Looking for the [C-API documentation](http://gazebosim.org/haptix/api)? Try [http://gazebosim.org/haptix/api](http://gazebosim.org/haptix/api).**

This tutorial will explain how to use the C HAPTIX client library `haptix-comm` for
requesting a description of the hand, sending new joint commands, and receiving state updates.

We assume that you have already done the [installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix).


# Compile your controller

In this tutorial we include a very basic controller that applies a sinusoidal
function to all the hand joints. First, you should compile your controller and
link it to the haptix-comm library.

## Windows

Open Visual Studio 2013 and create a new project for your hand controller. Click
on `File`->`New Project`->`Visual C++`, and select `Win32 Console Application`
from the right side window. Select an appropriate name for your project, for
example `MyBasicController`. Click `OK`, and then, click on `Finish`.

Replace the source code from your current project with our basic controller. Copy
the code from [**here**](http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c) and paste it in your current project.

Add the following line at the beginning of your source code:

~~~
#include "stdafx.h"
~~~

### Configuration for 64-bit SDK

**Note: Skip this section if you are using the 32-bit SDK.**


Click on the dropdown menu with the `Win32` option in the toolbar and select
`Configuration Manager`.

%%%
[[file:files/visual-studio.png|800px]]
%%%

After that, look for the `Active solution platform` dropdown menu and select
`<New...>`. Click on `Type or select the new platform` and change `ARM` to `x64`
, and then, click the `Close` button.

### Compile your code

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

On the Linux machine, double-click on the `haptixStart` desktop icon to start
the simulation.

## Windows

Your code should be ready to be executed using Visual Studio. Click on `DEBUG`->
`Start Without Debugging...` (alternatively you can press Ctrl+F5).

**Note:** if the Windows firewall is enabled, it will show a Window called
"Windows security alert. Windows Firewall has blocked some features of this
program" asking for permissions to run the recently compiled application. You
could leave the default option ("Private networks, such as my home or work
network.") and click on "Allow access".

## Linux

1. Go to the Linux machine where you want to run your controller code. Open a
terminal and go to the `build/` directory where you have your controller
executable:

    ~~~
    cd ~/haptix_controller/build
    ~~~

1. Start the controller:

    ~~~
    ./hx_controller
    ~~~

# Controller visualization

While your controller is running, you should see your fingers following a smooth
trajectory in Gazebo.

<iframe width="500" height="313" src="//player.vimeo.com/video/108959804" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

# The code explained

<include from='/int main/' to='/printDeviceInfo\(.deviceInfo\)/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c' />

The HAPTIX C API is composed of five C function calls: `hx_connect()`, `hx_robot_info()`,
`hx_update()`, `hx_read_sensors()`, and `hx_close()`. `hx_connect()` and `hx_close()` are
optional for the Gazebo simulator, but are included for compatibility with MuJoCo.

`hx_robot_info()` requests information from a given device.
In this tutorial, our device is a hand simulated in Gazebo. Note that this call
blocks until the response is received.

The parameter to `hx_robot_info()` is a `hxRobotInfo` struct that
contains the number of motors, joints, contact sensors, IMUs and joint limits
for the requested device. It also includes the update rate, which is how frequently the
device updates. If we have a valid response, the returned value is `hxOK`.

We have included in our example a helper function `printRobotInfo()` that will
print all the received fields for debugging purposes.

<include from='/  // Send commands/' to='/    usleep\(sleeptime_us\);/' src='http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c' />

Once we confirm the robot information we can start sending commands for
controlling the hand. The function `hx_update()` is in charge of sending a new
command and receiving the current state of the hand.

First of all, we need to fill a `hxCommand` struct that contains the positions,
velocities, and gains for each joint. Check the [haptix-comm API](https://s3.amazonaws.com/osrf-distributions/haptix/api/0.2.2/haptix_8h.html)
for a detailed view of the `hxCommand` struct. In our case, we are modifying the
position of all the joints according to a sinusoidal function.

The first parameter to `hx_update()`, `cmd`, is the command that we want to send to the device, which we already
filled in. The second output command `sensor` is passed to the function; it will contain the
state of the hand after applying the command.

We have included a helper function `printState()` that shows all the state
information for debugging purposes. Similar to `hx_robot_info()`, the function
`hx_update()` returns `hxOK` when the command was successfully sent and the
state has been received.

## Troubleshooting

### I can not connect the Windows client with Gazebo Linux Server

Usually you will receive a message from the command line launched by Visual
Studio with an error message `hx_robot_info() Service call timed out`. This
means that the communication to the Gazebo Linux server failed.

**First check:** Are both machines in the same network and can reach each other?
Simply using the ping command from both (windows command line and Linux shell),
should be enough to check connectivity. If it is not working, there is a problem
in network configuration, be sure that both are connected to the same network.

**Second check:** Is the communication layer using the right network interface?
Double check that you have set the `IGN_IP` properly in **both** Gazebo Linux server
and Windows. In Windows this can be done from the Windows command line `echo %IGN_IP%`.
To be completely sure, logout from your user session, login again and open Visual Studio.

**Third check:** Is the Windows firewall affecting the communication? It can be
disabled from the Windows Control Panel and try to launch the Visual Studio
application again.

**Fourth check:** Is the router cutting the communication? Several solutions:
Login into the router, disable any kind of firewall. If you can not do that,
connect both machines using an ethernet cable directly. You will need to
manually setup both in the same subnet (192.168.X.Y/255.255.255.0 and
192.168.X.Y+1/255.255.255.0).

### In the Windows Firewall question, I clicked the wrong option (cancel)

If by an error you pressed the wrong button, you can access to the `Windows
Firewall advanced security` from the Windows control panel, go to `Inbound
rules` and remove the ones with the name `ConsoleApplication` (or the name that
you gave to the application).

Another option is to modify your application name, in the `Project Properties
name` (not the solution name) that appears in the `Solution Explorer window`.
Clean, rebuild and run again.
