# Intro

Welcome to the Advanced Module! This module will guide you through the
process of making a contribution to Gazebo's codebase! We will cover
topics such as:

* Using Git and GitHub to manage source files
* Understanding the source code structure
* Finding an issue to fix
* Testing and documenting your code
* Getting your code integrated into Gazebo!

Each tutorial builds upon the last, so we recommend following the tutorials in order.

These tutorials are intended for those with basic experience using Gazebo,
Ubuntu Linux, and writing C++ code.

> **Note**: It's possible to contribute to Gazebo-classic if you're using a different
operating system, but for simplicity, this tutorial is focusing on Ubuntu users.

# Where is Gazebo's source code?

Gazebo-classic is an open source project, and as such, its source code is publicly
available online. It is currently hosted on a git repository at GitHub:

[https://github.com/osrf/gazebo/](https://github.com/osrf/gazebo/)

If none of that makes sense to you, let's go through some important concepts:

## Git

[**Git**](https://git-scm.org/) is a version control software used to
keep track of changes made to source code.

This tutorial series won't be going into detail about how git works, but
each `git` command used will be explained.

> If you're completely new to version control, there's a lot of good information
available online, such as [this](https://www.youtube.com/watch?v=SWYqp7iY_Tc)
tutorial series.

## GitHub

[**GitHub**](https://github.com) is the website used to host the git
repository so that the source code has a central place to live.

In order to contribute to Gazebo, you'll need to create an account there, by
following [this link](https://github.com/join).

# Where is the rest of the code?

You might be asking, is this it, is that all the code needed to bring Gazebo
to life? Well, not exactly. Gazebo, as most software, takes advantage of other
existing libraries to perform specific tasks. These are called _dependencies_.

For example, Gazebo-classic uses [Ogre](http://www.ogre3d.org/) for rendering,
[Qt](https://www.qt.io/) for the graphical user interface and supports a few
physics engines, such as [ODE](http://www.ode.org/) and
[Bullet](http://bulletphysics.org/wordpress/). That's only to name a few.

Among Gazebo's dependencies, there are some dependencies which are maintained
by the Gazebo-classic core team. Gazebo-classic development is often tied to the development of
these libraries, so everything that will be discussed on this series also
applies to those libraries. All these libraries are hosted on GitHub using
git. They are the following:

## SDFormat

SDF is an XML file format that describes worlds used by simulators
such as Gazebo. The SDF library is used to parse these files and provide a
C++ interface.

* Source code: [https://github.com/osrf/sdformat](https://github.com/osrf/sdformat)

## Ignition Math

Ignition Math is a small, fast, and high performance math library. This library
is a self-contained set of classes and functions suitable for robot applications.

* Source code: [https://github.com/ignitionrobotics/ign-math/](https://github.com/ignitionrobotics/ign-math/)

## Ignition Transport

Ignition Transport combines ZeroMQ with Protobufs to create a fast and
efficient message passing system. Asynchronous message publication and
subscription is provided along with service calls and discovery.

* Source code: [https://github.com/ignitionrobotics/ign-transport/](https://github.com/ignitionrobotics/ign-transport/)

## Ignition Messages

Ignition Messages provides a standard set of message definitions, used by
Ignition Transport, and other applications.

* Source code: [https://github.com/ignitionrobotics/ign-msgs/](https://github.com/ignitionrobotics/ign-msgs/)

