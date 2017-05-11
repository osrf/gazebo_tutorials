# Intro

Welcome to the Advanced Module! This module will guide you through the
process of making a contribution to Gazebo's codebase! We will cover
topics such as:

* Using Mercurial and Bitbucket to manage source files
* Understanding the source code structure
* Finding an issue to fix
* Testing and documenting your code
* Getting your code integrated into Gazebo!

Each tutorial builds upon the last, so we recommend following the tutorials in order.

These tutorials are intended for those with basic experience using Gazebo,
Ubuntu Linux, and writing C++ code.

> **Note**: It's possible to contribute to Gazebo if you're using a different
operating system, but for simplicity, this tutorial is focusing on Ubuntu users.

# Where is Gazebo's source code?

Gazebo is an open source project, and as such, its source code is publicly
available online. It is currently hosted on a mercurial repository at Bitbucket:

[https://bitbucket.org/osrf/gazebo/](https://bitbucket.org/osrf/gazebo/)

If none of that makes sense to you, let's go through some important concepts:

## Mercurial

[**Mercurial**](https://www.mercurial-scm.org/) is a version control software used to
keep track of changes made to source code. Some might be familiar with a similar
tool called **Git**.

This tutorial series won't be going into detail about how mercurial works, but
each mercurial command (`hg`) used will be explained.

> If you're completely new to version control, there's a lot of good information
available online, such as [this](https://www.youtube.com/watch?v=idd2fmPRRlU)
tutorial series.

## Bitbucket

[**Bitbucket**](https://bitbucket.org) is the website used to host the mercurial
repository so that the source code has a central place to live.

In order to contribute to Gazebo, you'll need to create an account there, by
following [this link](https://bitbucket.org/account/signup/).

# Where is the rest of the code?

You might be asking, is this it, is that all the code needed to bring Gazebo
to life? Well, not exactly. Gazebo, as most software, takes advantage of other
existing libraries to perform specific tasks. These are called _dependencies_.

For example, Gazebo uses [Ogre](http://www.ogre3d.org/) for rendering,
[Qt](https://www.qt.io/) for the graphical user interface and supports a few
physics engines, such as [ODE](http://www.ode.org/) and
[Bullet](http://bulletphysics.org/wordpress/). That's only to name a few.

Among Gazebo's dependencies, there are some dependencies which are maintained
by the Gazebo core team. Gazebo development is often tied to the development of
these libraries, so everything that will be discussed on this series also
applies to those libraries. All these libraries are hosted on Bitbucket using
mercurial. They are the following:

## SDFormat

SDF is an XML file format that describes worlds used by simulators
such as Gazebo. The SDF library is used to parse these files and provide a
C++ interface.

* Source code: [https://bitbucket.org/osrf/sdformat](https://bitbucket.org/osrf/sdformat)

## Ignition Math

Ignition Math is a small, fast, and high performance math library. This library
is a self-contained set of classes and functions suitable for robot applications.

* Source code: [https://bitbucket.org/ignitionrobotics/ign-math/](https://bitbucket.org/ignitionrobotics/ign-math/)

## Ignition Transport

Ignition Transport combines ZeroMQ with Protobufs to create a fast and
efficient message passing system. Asynchronous message publication and
subscription is provided along with service calls and discovery.

* Source code: [https://bitbucket.org/ignitionrobotics/ign-transport/](https://bitbucket.org/ignitionrobotics/ign-transport/)

## Ignition Messages

Ignition Messages provides a standard set of message definitions, used by
Ignition Transport, and other applications.

* Source code: [https://bitbucket.org/ignitionrobotics/ign-msgs/](https://bitbucket.org/ignitionrobotics/ign-msgs/)

