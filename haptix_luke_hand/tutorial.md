# Overview
This tutorial describes how to install the DEKA Luke hand model and how to start
the simulation environment with this limb.

We assume that you have already completed the
[installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix).

## Getting the DEKA Luke hand model and documentation

All performer team leads should have received a download link
  for the DEKA Luke hand model from DEKA webtransfer system.
If you believe you should have received a copy but did not,
  please contact `haptix@osrfoundation.org` for help.

The model file will be provided in a zipped file that looks like
   `Luke_Hand_Gazebo_vXX.X.zip` with corresponding documentation file in
   another zip file named `Luke_Hand_Gazebo_Doc_vXX.X.zip`.

Note that the generic haptix and simulation API documentation can be found
[here](http://gazebosim.org/haptix/api).

### Installing Gazebo DEKA Luke hand model for handsim

~~~
unzip Luke_Hand_Gazebo_vX.X.zip -d ~/.gazebo/models/
~~~

### Installing documentation for Gazebo DEKA Luke hand model

~~~
unzip Luke_Hand_Gazebo_Doc_vX.X.zip -d /tmp/
~~~

To view documentation, open the following link in a browser:

[file:///tmp/haptix_wiki/haptix/index.html](file:///tmp/haptix_wiki/haptix/index.html)

## Running handsim with DEKA Luke hand model

To start Gazebo with the DEKA Luke hand model, type the following command
  in a terminal:

~~~
gazebo --verbose worlds/luke_hand.world
~~~

## Installing documentation for Gazebo DEKA Luke hand model

~~~
unzip Luke_Hand_Gazebo_Doc_v1.0.zip -d /tmp/
~~~

To view documentation, open the following link in a browser:

[file:///tmp/haptix_wiki/haptix/index.html](file:///tmp/haptix_wiki/haptix/index.html)

# Running handsim with DEKA Luke hand model

To start Gazebo with the DEKA Luke hand model, type the following command
  in a terminal:

~~~
gazebo --verbose worlds/luke_hand.world
~~~

# Using the real DEKA Luke hand.

To use the real DEKA Luke hand with your own controller using the HAPTIX
protocol, you need to install the HAPTIX CANbus driver.

## Installing HAPTIX CANbus driver from Linux.

First, install the CAN dependencies:

~~~
sudo apt-get install libpcanbasic-dev
~~~

Then, download the haptix-luke-proprietary driver from here and type:

~~~
sudo dpkg -i ~/Downloads/haptix-luke-proprietary_0.0.1-1~trusty_amd64.deb
~~~

## Installing HAPTIX CANbus driver from Windows.

%%%
[[file:files/vide-visuals2.png|800px]]
%%%

Download the haptix-luke-proprietary driver from here and unzip it in your
preferred folder.

