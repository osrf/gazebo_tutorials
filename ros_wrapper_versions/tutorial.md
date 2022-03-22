# Introduction

This document provides an overview about the options to use different versions of ROS
 in combination with different versions of Gazebo.
It is recommended to read it before installing the Gazebo ROS wrappers.

# Important! simple analysis for a quick and correct decision

If you are planning on using a specific version of ROS and don't have a reason
to use a specific version of Gazebo, you should proceed with the
[Installing gazebo\_ros\_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
tutorial which explains how to install the fully supported version of gazebo
by ROS.

Warning: note that using a different gazebo version than the official version delivered from the ROS
repositories could end up in conflicts or other integration problems with ROS packages.

# Gazebo versions and ROS integration

Gazebo is an independent project like boost, ogre or
any other project used by ROS. Usually, the latest major version of gazebo
available at the beginning of every ROS release cycle (for example `gazebo11`
for ROS Noetic) is selected as the official one to be fully integrated and
supported and will be kept during the whole life of the ROS distribution.

Gazebo development is not synced with ROS, so each new major version of Gazebo
must be released before being used in a ROS distribution.  The following
sections cover how to use ROS with different versions of Gazebo.

Note that Gazebo ABI stability policy follows the
[semantic versioning](http://semver.org/) philosophy, in which all versions
that have the same major number (`gazebo_11.0.0`, `gazebo_11.1.0`,
`gazebo_11.0.1`, ...) are binary compatible and thus interchangeable when using
the same ROS distro.

## Installing Gazebo

### Gazebo Ubuntu packages

The easiest way of installing Gazebo is to use packages. There are two main repositories which host Gazebo packages: one is `packages.ros.org` and the other is `packages.osrfoundation.org`. At the time of writing:

 * ***packages.ros.org***
  *  ROS Melodic: Gazebo 9.x
  *  ROS Noetic: Gazebo 11.x
  *  ROS2 Foxy: Gazebo 11.x
  *  ROS2 Rolling: Gazebo 11.x
 * ***packages.osrfoundation.org***
  * gazebo 9.x series (package name `gazebo9`)
  * gazebo 11.x series (package name `gazebo11`)

This means that including the osrfoundation repository is not strictly needed to get the Gazebo Ubuntu package.
It can be installed from the ros repository.

### Gazebo built from source

If you have compiled a gazebo version from source, note that depending on the
repository branch used (`gazebo9` or `gazebo11`) your gazebo will be
binary compatible with the `gazebo_ros_pkgs` (and all other ROS packages compiled
on top of gazebo) only if the major version matches your local branch
repository and the gazebo version used in your ROS distro.  For example, if you
are compiling from gazebo branch `gazebo11`, you can use the `gazebo_ros_pkgs`
present in Noetic (which uses gazebo11 series).

Note that if you are using `default` branch, you are probably not binary
compatible with any of the packages released, so you will need a catkin
workspace for getting a valid `gazebo_ros_pkgs`.

## Using the default Gazebo version for a ROS distribution

For the users that need to run a specific version of ROS
 and want to use all the gazebo ROS related packages out-of-the-box,
 this is the recommended section:

### ROS2 Foxy and ROS2 Rolling
ROS2 Foxy and ROS2 Rolling host or use the 11.x version of Gazebo.
For a fully-integrated ROS system, we recommend using the 11.x version of
Gazebo.  The way to proceed is just to use the ROS repository (it will
automatically install `gazebo11`) and do ***not*** use the osrfoundation
repository.

### ROS1 Noetic
ROS Noetic hosts or use the 11.x version of Gazebo.
For a fully-integrated ROS system, we recommend using the 11.x version of
Gazebo.  The way to proceed is just to use the ROS repository (it will
automatically install `gazebo11`) and do ***not*** use the osrfoundation
repository.

### ROS1 Melodic
ROS Melodic hosts or use the 9.x version of Gazebo.
For a fully-integrated ROS system, we recommend using the 9.x version of
Gazebo.  The way to proceed is just to use the ROS repository (it will
automatically install `gazebo9`) and do ***not*** use the osrfoundation
repository.

## Using a specific Gazebo version with ROS
***Warning!: Using this option, you won't be able to use any ROS Ubuntu package
related to Gazebo from ROS deb repository.  The equivalent of `gazebo_ros_pkgs`
can be installed from debian packages, but all other software (such as
[turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)) must be built from
source.  Thanks to [catkin workspaces](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
or [colcon workspaces](https://docs.ros.org/en/rolling/Tutorials/Workspace/Creating-A-Workspace.html)
this is quite easy to do.***

There is a way of using any specific version of gazebo and ROS if really needed:

### Gazebo 11.x series

The OSRF repository provides `-gazebo11-` versions of ROS/Melodic or
ROS2/Eloquent gazebo wrappers (`gazebo11_ros_pkgs`) which are built on top of
the `gazebo11` package.  The steps to use them are:

 * Add the osrfoundation repository to your sources list.
 * Install `ros-$ROS_DISTRO-gazebo11-ros-pkgs` and/or `ros-$ROS_DISTRO-gazebo11-ros-control`
   from the osrfoundation repository, which will install the `gazebo11` package.
 * Use catkin workspaces to compile the rest of the software used from source.

## FAQ

#### I am not using ROS at all, which version should I use?

If you don't need ROS support, the recommended version is the latest released version that can be
 [installed using the osrfoundation repo](http://gazebosim.org/install).

#### I need to use gazebo11 and ROS Melodic what can I do?
***Warning!: Using this option, you won't be able to use any ROS Melodic package
related to Gazebo from ROS deb repository. The way to go is to build them from
source. Thanks to catkin workspaces this is quite easy to do.***

If you need some features only present in the version 11.x of Gazebo, there
is a way of installing `gazebo11`and ROS Melodic/Dashing/Eloquent. Please
follow the instructions about how to use ROS with gazebo11 package which
are in this same document.

#### Some ROS packages conflict with GazeboX ROS Wrappers!

Note that each ROS distribution is designed to be used with an specific version
of Gazebo (`gazebo9` in Melodic). When someone chooses to use a different version
of Gazebo than the one recommended in the ROS distribution, problems may appear
and some of them could be unsolvable.

If you a find a dependency conflict (for example with RVIZ) after trying to
install one of the versions described in this document, you will need to
probably install ROS or Gazebo from source.
