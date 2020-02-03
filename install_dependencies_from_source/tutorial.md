# Install dependencies from source on Ubuntu

This tutorial will go through the process of installing some of Gazebo's
dependencies from source. The dependencies listed here are all maintained by
the Gazebo team and often new features in Gazebo are tied to new features in
these libraries.

These libraries are:

* [SDFormat](http://sdformat.org/)
* [ignition-cmake](http://ignitionrobotics.org/libs/cmake)
* [ignition-common](http://ignitionrobotics.org/libs/common)
* [ignition-fuel-tools](http://ignitionrobotics.org/libs/fuel_tools)
* [ignition-math](http://ignitionrobotics.org/libs/math)
* [ignition-msgs](http://ignitionrobotics.org/libs/msgs)
* [ignition-transport](http://ignitionrobotics.org/libs/transport)

[[file:files/gazebo_dependency_tree.svg|400px]]

## A bit of history

All the libraries listed here are evolutions of libraries which were at some
point built within the Gazebo project itself. In an effort to make these
available for other projects and to make Gazebo more modular, they have been
extracted from Gazebo. Since March 2019, the Gazebo team has also been releasing
a [new simulator](https://ignitionrobotics.org/libs/gazebo) entirely based on
Ignition libraries.

### SDFormat

#### SDF protocol

Gazebo uses the Simulation Description Format (SDF) protocol to describe every
aspect of simulation. The SDF protocol is based on XML, you can take a look at
its specification [here](http://sdformat.org/spec). The protocol consists of a
series of (*.sdf) files.

Current protocol versions available are 1.4, 1.5, 1.6 and 1.7.

#### SDFormat C++ library

Gazebo uses the SDFormat C++ library to parse the SDF protocol.

> Both the SDF protocol and the SDFormat C++ parser are hosted in the same
> repository and will be installed at the same time when performing an
> installation from source.

> Please note that SDFormat library versions follow semantic versioning where
> major versions correspond to changes in ABI. Its versioning scheme has nothing
> to do with the SDF protocol supported.

#### Versions

Gazebo has had a dependency on the SDFormat library (which automatically handles
the SDF protocol supported) since early versions:

* Gazebo 1.9 - SDFormat > 1? (SDF protocol <= 1.4)
* Gazebo 2.2 - SDFormat > 1.4.7 and < 2.0 (SDF protocol <=  1.5)
* Gazebo 3 - SDFormat > 2.0.1 and < 3.0 (SDF protocol <=  1.5)
* Gazebo 4 - SDFormat > 2.0.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo 5 - SDFormat > 2.3.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo 6 - SDFormat > 3.1.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo 7 - SDFormat > 4.1.0 and < 5.0 (SDF protocol <=  1.6)
* Gazebo 8 - SDFormat 5.0 (SDF protocol <=  1.6)
* Gazebo 9 - SDFormat 6.0 (SDF protocol <= 1.6)
* Gazebo 10 - SDFormat > 6.0 and < 7.0 (SDF protocol <= 1.6)
* Gazebo 11 - SDFormat 9 (SDF protocol <= 1.7)

### Ignition Common

* Gazebo 9 - Ignition common 1
* Gazebo 10 - Ignition common 1
* Gazebo 11 - Ignition common 3

### Ignition Fuel Tools

* Gazebo 9 - Ignition fuel tools 1
* Gazebo 10 - Ignition fuel tools 1
* Gazebo 11 - Ignition fuel tools 4

### Ignition Math

Gazebo has a dependency on Ignition Math from version 6.

* Gazebo 6 - Ignition math 2.0
* Gazebo 7 - Ignition math 2.4
* Gazebo 8 - Ignition math 3 - The built-in gazebo::math library is completely deprecated.
* Gazebo 9 - Ignition math 4 - The built-in gazebo::math library is completely removed.
* Gazebo 10 - Ignition math 4
* Gazebo 11 - Ignition math 6

### Ignition Transport

Gazebo has a dependency on Ignition Transport from version 7.

* Gazebo 7 - Ignition transport 1 or 2
* Gazebo 8 - Ignition transport 3
* Gazebo 9 - Ignition transport 4
* Gazebo 10 - Ignition transport 4
* Gazebo 11 - Ignition transport 8

### Ignition Messages

Gazebo has a dependency on Ignition Messages from version 8.

* Gazebo 8 - Ignition msgs 0.4
* Gazebo 9 - Ignition msgs 1.0
* Gazebo 10 - Ignition msgs 1.0
* Gazebo 11 - Ignition msgs 5

## Remove packages to get a clean system

Be sure of removing any distribution package of the dependencies listed in this
document to get started from a clean system. For example, in .deb distributions
like Debian or Ubuntu this can easily done by running:

        sudo apt-get remove '.*sdformat.*' '.*ignition-.*'

## Build and install Ignition CMake from source

Many of the ignition packages are using the ignition cmake library.

1. Install required dependencies:

        sudo apt-get install build-essential cmake pkg-config

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-cmake /tmp/ign-cmake
        cd /tmp/ign-cmake

1. Checkout the corresponding branch for a target Gazebo version:
   (e.g. `ign-cmake0` for Gazebo 10)

        hg up ign-cmake0

1. Create a build directory and go there. Configure the build:

        mkdir build
        cd build
        cmake ../

    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

1. Build and install:

        make -j4
        sudo make install


## Build and install Ignition Math from source

SDFormat, Ignition Messages and Gazebo depend on the Ignition Math library.

1. Install required dependencies:

        sudo apt-get install build-essential \
                             cmake \
                             mercurial \
                             python

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-math /tmp/ign-math
        cd /tmp/ign-math

1. Checkout the corresponding branch for a target Gazebo version:
   (e.g. `ign-math4` for Gazebo 10)

        hg up ign-math4

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install

## Build and install Ignition Common from source

Gazebo and Ignition Fuel Tools depend on the Ignition Common library.

1. Install required dependencies (note that ignition-cmake and ignition-math are out):

        sudo apt-get install build-essential \
                             cmake \
			     libfreeimage-dev \
			     libtinyxml2-dev \
			     uuid-dev \
			     libgts-dev \
			     libavdevice-dev \
			     libavformat-dev \
			     libavcodec-dev \
			     libswscale-dev \
			     libavutil-dev \
			     libprotoc-dev \
	 	             libprotobuf-dev

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-common /tmp/ign-common
        cd /tmp/ign-common

1. Checkout the corresponding branch for a target Gazebo version:
   (e.g. `ign-common1` for Gazebo 10)

        hg up ign-common1

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install

## Build and install SDFormat from source

Gazebo depends on the SDFormat package.

1. Install required dependencies (note that ign-math was left out):

        sudo apt-get install build-essential \
                             cmake \
                             mercurial \
                             python \
                             libboost-system-dev \
                             libtinyxml-dev \
                             libxml2-utils \
                             ruby-dev \
                             ruby

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/osrf/sdformat /tmp/sdformat
        cd /tmp/sdformat

1. Checkout the corresponding branch for a target Gazebo version:
   (e.g. `sdf6` for Gazebo 10)

        hg up sdf6

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install

## Build and install Ignition Messages from source

Gazebo and Ignition Transport depend on the Ignition Messages package.

1. Install required dependencies:

        sudo apt-get install build-essential \
                             cmake \
                             mercurial \
                             libprotoc-dev \
                             libprotobuf-dev \
                             protobuf-compiler

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-msgs /tmp/ign-msgs
        cd /tmp/ign-msgs

1. Checkout the corresponding branch for a target Gazebo version:
   (e.g. `ign-msgs1` for Gazebo 10)

        hg up ign-msgs1

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../

    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install


## Build and install Ignition Fuel Tools

Gazebo depends optionally in the Ignition Fuel Tools

1. Install required dependencies (note that ignition-cmake and ignition-common are out):

        sudo apt-get install build-essential \
                             cmake \
			     libzip-dev \
			     libjsoncpp-dev \
			     libcurl4-openssl-dev \
			     libyaml-dev

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-fuel-tools /tmp/ign-fuel-tools
        cd /tmp/ign-fuel-tools

1. Checkout the corresponding branch for a target Gazebo version:
   (e.g. `ign-fuel-tools1` for Gazebo 10)

        hg up ign-fuel-tools1

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode (strictly speaking, `RelWithDebInfo`): This will generate
       optimized code, but will not have debug symbols. Use this mode if you
       don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch
    > between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. It will run
       slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

1. Build and install:

        make -j4
        sudo make install


## Build and install Ignition Transport from source

Gazebo depends on the Ignition Transport package.

Please follow the instructions on the Ignition Transport
[documents](https://ignitionrobotics.org/tutorials/transport/4.0/md__data_ignition_ign-transport_tutorials_installation.html).

When installing dependencies, make sure you only install the
`libignition-msgs-dev` package if you haven't installed Ignition Messages from
source.
