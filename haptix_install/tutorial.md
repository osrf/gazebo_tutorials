# Overview

The HAPTIX project simulates advanced robotic prosthetic limbs to allow
researchers and developers to test their control software before running on the
real hardware. The HAPTIX setup is composed of a Gazebo simulator, a client
library and a client controller. The Gazebo simulator runs on a Linux machine,
whereas the HAPTIX client library and the client controller can run on Linux and
Windows hosts. This tutorial will guide your through the installation of Gazebo
and the HAPTIX client library. Check out the rest of the HAPTIX tutorials for
instructions on how to create your own hand controller.

[[file:files/haptix_overview.png|800px]]

# Gazebo installation

**Note: This section is only required if you do not yet have a Linux server
running Gazebo with the HAPTIX packages.**

Before installing Gazebo, you need a machine with Ubuntu 14.04 64-bit
installed. Once your Linux machine is ready, open up a terminal and run the
following command:

~~~
wget -O /tmp/haptix_gazebo_install.sh http://osrf-distributions.s3.amazonaws.com/haptix/haptix_gazebo_install.sh; sudo sh /tmp/haptix_gazebo_install.sh
~~~

You can test your Gazebo installation by running the next
command in your terminal:

~~~
gazebo worlds/arat.world
~~~

Gazebo will load a manipulation environment with a right Modular Prosthetic
Limb (MPL). Alternatively, you can load a left arm by typing:

~~~
gazebo worlds/arat_left.world
~~~

Once you are done testing, please close Gazebo.

# HAPTIX Client library SDK

The HAPTIX client library SDK is a stack of libraries that enables communication
between your control software and the simulated hand in Gazebo. The main library is
called `haptix-comm`. `haptix-comm` exposes the API that will allow your software to
request information about the hand (number of joints, motors, number of sensors,
etc.), as well as send new joint commands and receive the hand state.

## Installing on Windows

We currently support Windows 7, Visual Studio 2013, and Matlab R2014b, all
64-bit (you can determine the Matlab version by selecting `Help`->`About
MATLAB` from the main menu).  We are working on 32-bit versions; they will be
available soon.

Download the SDK by going to:

* [Windows Client SDK Download](http://gazebosim.org/distributions/haptix)

and selecting a version.  You probably want the latest version that is
available (but check below for information on version compatibility).

Unzip the zip file into your preferred HAPTIX folder. For example: `C:\Users\osrf\Desktop\haptix-ws`.

### Network configuration

Now, you need to do some network configuration to specify
the IP address that you will use for communicating with Gazebo.

Click on the `Start button` and type `cmd` in the `Search box`. Once you are in
the terminal run the following command:

~~~
ipconfig
~~~

Look for the `IPv4 Address` inside the `Ethernet adapter Local Area Connection` section.

[[file:files/ipconfig.png|600px]]

Click on the `Start button` and search for `environment variables`.
Click on `Edit the system environment variables`, and then, click on the button
`Environment variables`. Click on the `New` button from the `User variables`
section to create a new environment variable with name `IGN_IP`.
**Enter the IPv4 Address mentioned above**.

Now, we want to **create a partition** to group your Gazebo machine with all the
related machines (e.g.: your MATLAB/Octave/Visual Studio development machine).
This is important because if you decide to later run a second or third Gazebo
instance on the same network, you will get crosstalk if there is no partition.

Click on the `New` button again and create a new environment variable with name
`IGN_PARTITION`. Enter a name like `gazebo1`. Repeat this process on any Windows
machine that you plan to use with this Gazebo instance.

## Installing on Linux

###  One-line install

~~~
wget -O /tmp/haptix_sdk_install.sh http://osrf-distributions.s3.amazonaws.com/haptix/haptix_sdk_install.sh; sudo sh /tmp/haptix_sdk_install.sh
~~~

### or Step-by-step install

1. Setup your computer to accept software from packages.osrfoundation.org.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'

1. Setup keys.

        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -;
        sudo apt-get update

1. Install the SDK.

        sudo apt-get install libhaptix-comm-dev

### Network configuration

Now, you need to do some network configuration to specify
the IP address that you will use for communicating with the other machines (e.g. your MATLAB development machine).

Open a terminal and run the following command:

~~~
ifconfig
~~~

Look for `inet addr` in the `eth0` section.

[[file:files/ifconfig.png|600px]]

Run the following command in the terminal, replacing the `IGN_IP` value with the
`inet_addr` mentioned above:

~~~
echo "export IGN_IP=172.23.2.37" >> ~/.bashrc
~~~

Now, we want to add our Linux machines (Gazebo/Octave) to the partition created
in the previous section (in this example we called it `gazebo1`).

Run the following command in the terminal:

~~~
echo "export IGN_PARTITION=gazebo1" >> ~/.bashrc
~~~

Repeat this process on any Linux machine that you plan to use with this Gazebo
instance.

# Software Update

There are two ways the HAPTIX software may be updated:

- Command-line
   1. Open a terminal
   1. Run the following commands

         ~~~
         sudo apt-get update
         sudo apt-get upgrade
         ~~~

- Using the desktop icon.
   1. **Note:** This requires the `haptixUpdate` desktop icon. For more information, see <a href='/tutorials?tut=haptix_unboxing&cat=haptix#Environmentoverview'>here</a>.
   1. Double click the `haptixUpate` icon on the desktop.

##Version numbers

There are two version numbers relevant to the HAPTIX software. One is the
version of the simulator; the other is the version of the client SDK.

###Simulator version

To check the simulator version: Run Gazebo by either double-clicking the
`haptixStart` desktop icon, or running the following command in a terminal.

~~~
gazebo worlds/arat.world
~~~

The simulator version (more specifically, the version of the `handsim`
package) is located at the bottom left of the on-screen GUI.

[[file:files/haptix_version.png|400px]]

In this case, the simulator version is 0.5.1.

From 0.8.0 version on, the version number in the menu located at the right
top corner of the hand GUI, after pressed the gear icon:

[[file:files/haptix_version2.png|400px]]

In this case, the simulator version is 0.8.1

###Client SDK version

####Windows
On Windows, the client SDK version is in the name of the `.zip` file and the
directory that you downloaded (and the resulting directory that the `.zip` file
extracts to).  E.g., if you downloaded `hx_gz_sdk-0.6.0-Release-win64.zip`,
then you have version 0.6.0 of the client SDK.

####Linux
On Linux, the client SDK version is the version of the `libhaptix-comm-dev`
package that is installed.  You can check it by running the following command in
a terminal:

~~~
dpkg -l libhaptix-comm-dev
~~~

You should see output similar to:

~~~
Desired=Unknown/Install/Remove/Purge/Hold
| Status=Not/Inst/Conf-files/Unpacked/halF-conf/Half-inst/trig-aWait/Trig-pend
|/ Err?=(none)/Reinst-required (Status,Err: uppercase=bad)
||/ Name                    Version          Architecture     Description
+++-=======================-================-================-====================================================
ii  libhaptix-comm-dev:amd6 0.6.0-1~trusty   amd64            Haptix project communication library - Development f
~~~

In this case, the client SDK version is 0.6.0.

##Version compatiblity

To ensure correct behavior between the Gazebo simulator and the client
library SDK, you should check that you are using compatible versions of the
two packages.  The following table summarizes compatible combinations:

Simulator Version : Client SDK Version

* 0.8.x : 0.7.y
* 0.7.x : 0.6.y
* 0.6.x : 0.5.y

