# Introduction

This document provides an overview about the options to use different versions of ROS
 in combination with different versions of Gazebo.
It is recommended to read it before installing the Gazebo ROS wrappers.

# Short version for quick decision

If you are planning on using a specific version of ROS and don't have a reason
 to use a specific version of Gazebo,
 you can proceed with the
 [Installing gazebo\_ros\_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
 tutorial.

# Gazebo versions and ROS integration

Since ROS Hydro, Gazebo is considered a system package instead of ROS package,
 which means that one major version of gazebo (for example `gazebo2` for ROS Indigo)
 is selected at the beginning of the ROS release cycle
 and will be kept during the whole life of the ROS distribution.
Gazebo development is independent from ROS, so each new major version of Gazebo
 must be released before being used in a ROS distribution.
The following sections cover how to use ROS with different versions of Gazebo.

Note that Gazebo ABI stability policy follows the
 [semantic versioning](http://semver.org/) philosophy, in which all versions that have the
 same major number (`gazebo_2.0.0`, `gazebo_2.1.0`, `gazebo_2.0.1`, ...)
 are binary compatible and thus interchangeable when using the same ROS distro.

## Installing Gazebo

### Gazebo Ubuntu packages

The easiest way of installing Gazebo is to use packages. There are two main repositories which host Gazebo packages: one is `packages.ros.org` and the other is `osrfoundation.packages.org`. At the time of writing:

 * ***packages.ros.org***
  *  Hydro: hosts gazebo version 1.x package.
  *  Indigo: host gazebo version 2.x package.
  *  Jade: host gazebo version 5.x package.
 * ***packages.osrfoundation.org***
  * gazebo 1.x series (package name `gazebo`)
  * gazebo 2.x series (package name `gazebo-current` or `gazebo2` in saucy/trusty)
  * gazebo 4.x series (package name `gazebo4`)
  * gazebo 5.x series (package name `gazebo5`)

This means that including the osrfoundation repository is not strictly needed to get the Gazebo Ubuntu package.
It can be installed from the ros repository.

### Gazebo built from source

If you have compiled a gazebo version from source, note that depending on the
repository branch used (`gazebo_4.0`,`gazebo_5.0`,...) your gazebo will be
binary compatible with the `gazebo_ros_pkgs` (and all other ROS packages compiled
on top of gazebo) only if the major version matches your local branch
repository and the gazebo version used in your ROS distro.  For example, if you
are compiling from gazebo branch `gazebo_2.0`, you can use the `gazebo_ros_pkgs`
present in Indigo (which uses gazebo2 series).

Note that if you are using `default` branch, you are probably not binary
compatible with any of the packages released, so you will need a catkin
workspace for getting a valid `gazebo_ros_pkgs`.

## Using the default Gazebo version for a ROS distribution

For the users that need to run a specific version of ROS
 and want to use all the gazebo ROS related packages out-of-the-box,
 this is the recommended section:

### Jade

ROS Jade hosts the 5.x version of Gazebo.
For a fully-integrated ROS system, we recommend using the 5.x version of
Gazebo.  The way to proceed is just to use the ROS repository (it will
automatically install `gazebo5`) and do ***not*** use the osrfoundation
repository.

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

## Using a specific Gazebo version with ROS
***Warning!: Using this option,
 you won't be able to use any ROS Ubuntu package related to Gazebo from ROS deb repository.
The equivalent of `gazebo_ros_pkgs` can be installed from debian packages,
 but all other software (such as [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo))
 must be built from source.
Thanks to
 [catkin workspaces](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
 this is quite easy to do.***

There is a way of using any specific version of gazebo and ROS if really needed:

### Gazebo 5.x series

The OSRF repository provides `-gazebo5-` versions of ROS/Indigo gazebo wrappers
 (`gazebo5_ros_pkgs`) which are built on top of the `gazebo5` package.
The steps to use them are:

 * Add the osrfoundation repository to your sources list.
 * Install `ros-indigo-gazebo5-ros-pkgs` from the osrfoundation repository, which will install the `gazebo5` package.
 * Use catkin workspaces to compile the rest of the software used from source.

### Gazebo 4.x series

The OSRF repository provides `-gazebo4-` versions of ROS/Indigo and ROS/Hydro gazebo wrappers
 (`gazebo4_ros_pkgs`) which are built on top of the `gazebo4` package.
The steps to use them are:

 * Add the osrfoundation repository to your sources list.
 * Install `ros-$distro-gazebo4-ros-pkgs` from the osrfoundation repository, which will install the `gazebo4` package.
 * Use catkin workspaces to compile the rest of the software used from source.

### Gazebo 2.x series

The OSRF repository provides `-current` versions of ROS/Hydro and ROS/Groovy gazebo wrappers
 (`gazebo_ros_pkgs-current`) which are built on top of the `gazebo-current` package.
The steps to use them are:

 * Add the osrfoundation repository to your sources list.
 * Install `ros-$distro-gazebo-ros-pkgs-current` from the osrfoundation repository, which will install the `gazebo-current` package.
 * Use catkin workspaces to compile the rest of the software used from source.

## FAQ

#### I am a DARPA Robotics Challenge participant, which version should I use?

Starting from `drcsim-4.0`, `gazebo4` is the one to use. The old `drcsim-3.2` is built
on top of `gazebo3`.

DRCSim package can not use `gazebo5` since Ubuntu Precise is one of the
supported platforms and lacks of `gazebo5` support. Participants using
Ubuntu Trusty can use `gazebo5` but `drcsim` needs to be compiled from
source.

#### I am not using ROS at all, which version should I use?

If you don't need ROS support, the recommended version is the latest released version that can be
 [installed using the osrfoundation repo](http://gazebosim.org/install).

#### I want to use the bullet/simbody/dart physics engine, which version of Gazebo should I use?

Starting from `gazebo4`, bullet and simbody support is built into the Ubuntu package,
 so please follow the above instructions to use `gazebo4` in combination with ROS.
Dart still requires gazebo installation from source (starting from `gazebo3`),
 so you can use `gazebo3` or above and follow the instructions above in this page to make it work with ROS.

#### I need to use gazebo4/gazebo5 and ROS Indigo, what can I do?
***Warning!: Using this option, you won't be able to use any ROS Indigo package related to Gazebo from ROS deb repository. The way to go is to build them from source. Thanks to catkin workspaces this is quite easy to do.***

If you need some features only present in version 4.x or 5.x of Gazebo, there
is a way of installing `gazebo4` or `gazebo5` and ROS Indigo. Please follow the
instructions about how to use ROS with gazebo4 package or gazebo5 which are in 
this same docuement.

#### I need to use gazebo3 and ROS Indigo, what can I do?
***Warning!: Using this option, you won't be able to use any ROS Indigo package related to Gazebo from ROS deb repository. The way to go is to build them from source. Thanks to catkin workspaces this is quite easy to do.***

If you need some features only present in version 3.x of Gazebo, there is a way of installing `gazebo3` and ROS Indigo. Please follow the [instructions about how to use ROS with gazebo3 package](#Gazebo 3.x series) which are in this page.
