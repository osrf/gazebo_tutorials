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
running Gazebo.**

Before installing Gazebo, you need a machine with Ubuntu 14.04 64-bit
installed. Once your Linux machine is ready, open up a terminal and run the
following command:

~~~
wget -O /tmp/haptix_gazebo_install.sh http://osrf-distributions.s3.amazonaws.com/haptix/haptix_gazebo_install.sh; sudo sh /tmp/haptix_gazebo_install.sh
~~~

If you are a HAPTIX participant you should have access to the
[`handsim-proprietary` deb package](http://gazebosim.org/haptix-proprietary).
Please, downloaded the package, open a new console and type the following
command to install the [Johns Hopkins APL arm](http://www.jhuapl.edu/prosthetics/):

~~~
sudo dpkg -i ~/Downloads/handsim-proprietary*.deb
~~~

You will experience some delay during the Gazebo start up phase if you do not
have the `handsim-proprietary` package installed. The APL arm will not appear in
the scene.

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
etc.), as well as request new joint commands and receive the hand state.

## Installing the HAPTIX client library SDK in Windows

We currently support Windows 7, Visual Studio 2013, and Matlab R2014b.

**If you have a Matlab R2014b 64-bit version you must download the
 [HAPTIX 64-bit client SDK](
https://s3.amazonaws.com/osrf-distributions/haptix/hx_gz_sdk-0.3.1-Release-win64.zip)**.
Otherwise, you must use the
 [HAPTIX 32-bit client SDK](
https://s3.amazonaws.com/osrf-distributions/haptix/hx_gz_sdk-0.3.1-Release-win32.zip). You can
 determine the Matlab version by selecting `Help`->`About MATLAB` from the main
 menu.

Unzip the zip file into your preferred HAPTIX folder. For example: `C:\Users\osrf\Desktop\haptix-ws`.

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

1. Install haptix-comm.

        sudo apt-get install libhaptix-comm-dev
