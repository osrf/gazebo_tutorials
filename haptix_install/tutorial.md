# Overview

The HAPTIX project simulates advanced robotic prosthetic limbs to allow
researchers and developers to test their control software before running on the
real hardware. The HAPTIX setup is composed by a Gazebo simulator, a client
library and a client controller. The Gazebo simulator runs on a Linux machine,
whereas the client library and the client controller can run in Linux and
Windows hosts. This tutorial will guide your through the installation of Gazebo
and the client library. Check out the rest of the HAPTIX tutorials for
instructions on how to create your own hand controller.

# Gazebo installation.

Before installing Gazebo, you need a machine with Ubuntu 14.04 64 bits
installed. Once your Linux machine is ready, open up a terminal and run the
following command:

~~~
wget -O /tmp/gazebo_haptix_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo_haptix_install.sh; sudo sh /tmp/gazebo_haptix_install.sh
~~~

You can test your Gazebo installation by running the next command on your
terminal:

~~~
gazebo
~~~

# Client library SDK.

The client library SDK is a stack of libraries that enables the communication
between programs and the simulated hand in Gazebo. Tha main public library is
called `haptix-comm`. `haptix-comm` exposes the API that will allow a program to
request information about the hand (number of joints, motors, number of sensors,
etc.), as well as request a new joint command and receive the new hand state.

## Installing the client library SDK in Windows.

Download the latest version of the client SDK from [here](http://osrf-distributions.s3.amazonaws.com/haptix/hx_gz_sdk-0.1.0.zip).

Unzip the zip file into your HAPTIX working folder. For example: `C:\Users\osrf\Desktop\haptix-ws`.

## Installing the client library SDK in Linux.

###  One-line install

~~~
wget -O /tmp/haptix_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/haptix_install.sh; sudo sh /tmp/haptix_install.sh
~~~

### Step-by-step install

1. Setup your computer to accept software from packages.osrfoundation.org.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'

1. Setup keys

        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        sudo apt-get update

1. Install Google Protobuf.

        sudo apt-get install libprotobuf-dev protobuf-compiler

1. Install ZeroMQ and cppzmq.

        sudo apt-get install libzmq3 libzmq3-dev

1. Install Ignition Transport.

        sudo apt-get install libignition-transport0 libignition-transport-dev

1. Install haptix-comm.

        sudo apt-get install libhaptix-comm libhaptix-comm-dev