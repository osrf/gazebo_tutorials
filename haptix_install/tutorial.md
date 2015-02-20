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

# HAPTIX Client library SDK

The HAPTIX client library SDK is a stack of libraries that enables communication
between your control software and the simulated hand in Gazebo. The main library is
called `haptix-comm`. `haptix-comm` exposes the API that will allow your software to
request information about the hand (number of joints, motors, number of sensors,
etc.), as well as send new joint commands and receive the hand state.

## Installing the HAPTIX client library SDK in Windows

We currently support Windows 7, Visual Studio 2013, and Matlab R2014b.

**If you have a Matlab R2014b 64-bit version you must download the
 [HAPTIX 64-bit client SDK](
https://s3.amazonaws.com/osrf-distributions/haptix/hx_gz_sdk-latest-Release-win64.zip) (32-bit version should be available soon)**. You can
 determine the Matlab version by selecting `Help`->`About MATLAB` from the main
 menu.

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
`IGN_PARTITION`. Enter a name like `Gazebo1`. Repeat this process on any Windows
machine that you plan to use with this Gazebo instance.

## Installing the HAPTIX client library SDK in Linux

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

1. Install handsim.

        sudo apt-get install handsim

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

1. Command-line

    1. Open a terminal

    1. Run the following commands

        ~~~
        sudo apt-get update
        sudo apt-get upgrade
        ~~~

1. Using the desktop icon.

    1. **Note:** This requires the `haptixUpdate` desktop icon. For more information, see <a href='/tutorials?tut=haptix_unboxing&cat=haptix#Environmentoverview'>here</a>

    1. Double click the `haptixUpate` icon on the desktop.

##Version numbers

There are two version numbers relevant to the HAPTIX software. One is the version of Gazebo, the core simulation software. The second is the supplementary HAPTIX library.  Follow these steps to access both version numbers.

1. Run Gazebo by either double-clicking the `haptixStart` desktop icon, or running the following command in a terminal.

    ~~~
    gazebo worlds/arat.world
    ~~~

1. Use the `Help`->`About` menu option to display Gazebo's version.

    [[file:files/gazebo_version.png|400px]]

1. The HAPTIX client library version number is located at the bottom left of the on-screen GUI.

    [[file:files/haptix_version.png|400px]]
