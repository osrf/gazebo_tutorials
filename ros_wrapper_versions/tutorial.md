# Introduction 

This document provides an overview about the options to use different versions of ROS
 in combination with different versions of Gazebo.
It is recommended to read it before installing the Gazebo ROS wrappers.

# Short version for quick decision

If you are planning on using an specific version of ROS and don't have a reason
 to use an specific version of Gazebo version,
 you can go ahead and proceed with the
 [Installing gazebo\_ros\_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
 tutorial.

# Gazebo versions and ROS integration

Since ROS Hydro, Gazebo is considered a system package instead of ROS package,
 which means that one major version of gazebo (for example `gazebo2` for ROS Indigo)
 is selected at the beginning of the ROS release cycle
 and will be kept during the whole life of the ROS distribution.
Gazebo development is independent from ROS, so each new major version of Gazebo
 must be released before they being used in a ROS distribution.
The following sections cover how to use ROS with different versions of Gazebo.

## Gazebo Ubuntu packages

The easiest way of installing Gazebo is to use packages. There are two main repositories which host Gazebo packages: one is `packages.ros.org` and the other is `osrfoundation.packages.org`. At the time of writing:

 * ***packages.ros.org***
  *  Hydro: hosts gazebo version 1.x package.
  *  Indigo: host gazebo version 2.x package.
 * ***packages.osrfoundation.org*** 
  * gazebo 1.x series (package name `gazebo`)
  * gazebo 2.x series (package name `gazebo-current` or `gazebo2` in saucy/trusty)
  * gazebo 3.x series (package name `gazebo3`)
  * gazebo 4.x series (package name `gazebo4`)

This means that including the osrfoundation repository is not strictly needed to get the Gazebo Ubuntu package.
It can be installed from the ros repo.

## Using a specific ROS version and Gazebo 

For the users that need to run an specific version of ROS
 and want to use all the gazebo ROS related packages out-of-the-box,
 this is the recommended section:

### Indigo

ROS Indigo hosts the 2.x version of Gazebo.
For a fully-integrated ROS system, we recommend using the 2.x version of Gazebo.
The way to proceed is just to use the ROS repository (it will automatically install `gazebo2`)
 and do ***not*** use the osrfoundation repository.

### Hydro

ROS Hydro hosts the 1.9.x version of Gazebo.
For a fully-integrated ROS system, we recommend using the 1.9.x version of Gazebo.
The way to proceed is just to use the ROS repository (it will automatically install `gazebo`)
 and do ***not*** use the osrfoundation repository.

## Using a specific Gazebo version and ROS
***Warning!: Using this option,
 you won't be able to use any ROS Ubuntu package related to Gazebo from ROS deb repository.
The way to go is to build them from source.
Thanks to
 [catkin workspaces](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
 this is quite easy to do.*** 

There is a way of using any specific version of gazebo and ROS if really needed:

### Gazebo 4.x series

The OSRF repository provides `-gazebo4-` versions of ROS/Indigo and ROS/Hydro gazebo wrappers
 (`gazebo4_ros_pkgs`) which are built on top of the `gazebo4` package.
The steps to use them are:

 * Add the osrfoundation repository to your sources list
 * Install `ros-$distro-gazebo4-ros-pkgs` from the osrfoundation repository, which will install the `gazebo4` package.
 * Use catkin workspaces to compile the rest of the software used from source

### Gazebo 3.x series

The OSRF repository provides `-gazebo3-` versions of ROS/Hydro and ROS/Groovy gazebo wrappers
 (`gazebo3_ros_pkgs`) which are built on top of the `gazebo3` package.
The steps to use them are:

 * Add the osrfoundation repository to your sources list
 * Install `ros-$distro-gazebo3-ros-pkgs` from the osrfoundation repository, which will install the `gazebo3` package.
 * Use catkin workspaces to compile the rest of the software used from source

### Gazebo 2.x series

The OSRF repository provides `-current` versions of ROS/Hydro and ROS/Groovy gazebo wrappers
 (`gazebo_ros_pkgs-current`) which are built on top of the `gazebo-current` package.
The steps to use them are:

 * Add the osrfoundation repository to your sources list
 * Install `ros-$distro-gazebo-ros-pkgs-current` from the osrfoundation repository, which will install the `gazebo-current` package.
 * Use catkin workspaces to compile the rest of the software used from source

## FAQ 

#### I am a DARPA Robotics Challenge participant, which version should I use?

`drcsim-3.2` is built on top of `gazebo3`. Starting from `drcsim-4.0`, `gazebo4` is the one to use.

#### I am not using ROS at all, which version should I use?

If you don't need ROS support, the recommended version is the latest released version that can be
 [installed using the osrfoundation repo](http://gazebosim.org/tutorials/?tut=install).

#### I want to use the bullet/simbody/dart physics engine, which version of Gazebo should I use?

Starting from `gazebo4`, bullet and simbody support is built into the Ubuntu package,
 so please follow the above instructions to use `gazebo4` in combination with ROS.
Dart still requires gazebo installation from source (starting from `gazebo3`),
 so you can use `gazebo3` or above and follow the instructions above in this page to make it work with ROS.

#### Which version of Gazebo is going to work in ROS-J?

It is still undecided but [according with the schedule](http://gazebosim.org/#status), probably `gazebo5`.

#### I need to use gazebo4 and ROS Indigo, what can I do?
***Warning!: Using this option, you won't be able to use any ROS Indigo package related to Gazebo from ROS deb repository. The way to go is to build them from source. Thanks to catkin workspaces this is quite easy to do.***

If you need some features only present in version 4.x of Gazebo, there is a way of installing `gazebo4` and ROS Indigo. Please follow the [[#Gazebo 4.x series|instructions about how to use ROS with gazebo4 package]] which are in this page.

#### I need to use gazebo3 and ROS Indigo, what can I do?
***Warning!: Using this option, you won't be able to use any ROS Indigo package related to Gazebo from ROS deb repository. The way to go is to build them from source. Thanks to catkin workspaces this is quite easy to do.***

If you need some features only present in version 3.x of Gazebo, there is a way of installing `gazebo3` and ROS Indigo. Please follow the [[#Gazebo 3.x series|instructions about how to use ROS with gazebo3 package]] which are in this page.
