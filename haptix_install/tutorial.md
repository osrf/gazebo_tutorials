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

%%%
[[file:files/ipconfig.png|600px]]
%%%

Click on the `Start button` and search for `environment variables`.
Click on `Edit the system environment variables`, and then, click on the button
`Environment variables`. Click on the `New` button from the `User variables`
section to create a new environment variable with name `IGN_IP`. **Enter the IPv4 Address mentioned above**.

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

%%%
eth0      Link encap:Ethernet  HWaddr 90:2b:34:d7:51:7a
          inet addr:172.23.2.37  Bcast:172.23.3.255  Mask:255.255.252.0
          inet6 addr: fe80::922b:34ff:fed7:517a/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:64111138 errors:0 dropped:0 overruns:0 frame:0
          TX packets:57702428 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:14401677278 (14.4 GB)  TX bytes:6048516431 (6.0 GB)
          Interrupt:20 Memory:f7500000-f7520000

lo        Link encap:Local Loopback
          inet addr:127.0.0.1  Mask:255.0.0.0
          inet6 addr: ::1/128 Scope:Host
          UP LOOPBACK RUNNING  MTU:65536  Metric:1
          RX packets:30882650 errors:0 dropped:0 overruns:0 frame:0
          TX packets:30882650 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:0
          RX bytes:12394703023 (12.3 GB)  TX bytes:12394703023 (12.3 GB)

vmnet1    Link encap:Ethernet  HWaddr 00:50:56:c0:00:01
          inet addr:172.16.100.1  Bcast:172.16.100.255  Mask:255.255.255.0
          inet6 addr: fe80::250:56ff:fec0:1/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:0 errors:0 dropped:0 overruns:0 frame:0
          TX packets:460 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:0 (0.0 B)  TX bytes:0 (0.0 B)

vmnet8    Link encap:Ethernet  HWaddr 00:50:56:c0:00:08
          inet addr:172.16.49.1  Bcast:172.16.49.255  Mask:255.255.255.0
          inet6 addr: fe80::250:56ff:fec0:8/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:173555 errors:0 dropped:0 overruns:0 frame:0
          TX packets:1315 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:0 (0.0 B)  TX bytes:0 (0.0 B)
%%%

Run the following command in the terminal, replacing the `IGN_IP` value with the
one mentioned above:

echo "export IGN_IP=172.23.2.37 >> ~/.bashrc"
