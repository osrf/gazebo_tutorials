# Install dependencies from source on Ubuntu

This tutorial will go through the process of installing some of Gazebo's
dependencies from source. The dependencies listed here are all maintained by
the Gazebo-classic team and often new features in Gazebo-classic are tied to new features in
these libraries.

These libraries are:

* [SDFormat](http://sdformat.org/)
* [ignition-math](http://ignitionrobotics.org/libraries/math)
* [ignition-transport](http://ignitionrobotics.org/libraries/transport)
* [ignition-msgs](http://ignitionrobotics.org/libraries/messages)

[[file:files/gazebo_dependency_tree.svg|400px]]

## A bit of history

All the libraries listed here are evolutions of libraries which were at some
point built within the Gazebo-classic project itself. In an effort to make these
available for other projects and to make Gazebo-classic more modular, they have been
extracted from Gazebo.

### SDFormat

#### SDF protocol

Gazebo-classic uses the Simulation Description Format (SDF) protocol to describe every
aspect of simulation. The SDF protocol is based on XML, you can take a look at
its specification [here](http://sdformat.org/spec). The protocol consists of a
series of (*.sdf) files.

Current protocol versions available are 1.4, 1.5 and 1.6.

#### SDFormat C++ library

Gazebo-classic uses the SDFormat C++ library to parse the SDF protocol.

> Both the SDF protocol and the SDFormat C++ parser are hosted in the same
> repository and will be installed at the same time when performing an
> installation from source.

> Please note that SDFormat library versions follow a semantic versioning where
> major versions correspond to changes in ABI. Its versioning scheme has nothing
> to do with the SDF protocol supported.

#### Versions

Gazebo-classic has had a dependency on the SDFormat library (which automatically handles
the SDF protocol supported) since early versions:

* Gazebo-classic 1.9 - SDFormat > 1? (SDF protocol <= 1.4)
* Gazebo-classic 2.2 - SDFormat > 1.4.7 and < 2.0 (SDF protocol <=  1.5)
* Gazebo-classic 3 - SDFormat > 2.0.1 and < 3.0 (SDF protocol <=  1.5)
* Gazebo-classic 4 - SDFormat > 2.0.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo-classic 5 - SDFormat > 2.3.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo-classic 6 - SDFormat > 3.1.1 and < 4.0 (SDF protocol <=  1.5)
* Gazebo-classic 7 - SDFormat > 4.1.0 and < 5.0 (SDF protocol <=  1.6)
* Gazebo-classic 8 - SDFormat 5.0 (SDF protocol <=  1.6)
* Gazebo-classic 9 - SDFormat TBD

### Ignition Math

Gazebo-classic has a dependency on Ignition Math from version 6.

* Gazebo-classic 6 - Ignition math 2.0
* Gazebo-classic 7 - Ignition math 2.4
* Gazebo-classic 8 - Ignition math 3 - The built-in gazebo::math library is completely deprecated.
* Gazebo-classic 9 - Ignition math TBD - The built-in gazebo::math library is completely removed.

### Ignition Transport

Gazebo-classic has a dependency on Ignition Transport from version 7.

* Gazebo-classic 7 - Ignition transport 1 or 2
* Gazebo-classic 8 - Ignition transport 3
* Gazebo-classic 9 - Ignition transport TBD

### Ignition Messages

Gazebo-classic has a dependency on Ignition Messages from version 8.

* Gazebo-classic 8 - Ignition msgs 0.4
* Gazebo-classic 9 - Ignition msgs TBD

## Remove packages to get a clean system

Be sure of removing any distribution package of the dependencies listed in this
document to get started from a clean system. For example, in .deb distributions
like Debian or Ubuntu this can easily done by running:

        sudo apt-get remove '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'

## Build and install Ignition Math from source

SDFormat, Ignition Messages and Gazebo-classic depend on the Ignition Math library.

1. Install required dependencies:

        sudo apt-get install build-essential \
                             cmake \
                             git

1. Clone the repository into a directory and go into it:

        git clone https://github.com/ignitionrobotics/ign-math /tmp/ign-math
        cd /tmp/ign-math

     **Note:** the `default` branch is the development branch where
     you'll find the bleeding edge code, your cloned repository should be on
     this branch by default but we recommend you switch to the `ign-math3`
     branch if you desire more stability (with the `git checkout ign-math3` command).

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

Gazebo-classic depends on the SDFormat package.

1. Install required dependencies (note that ign-math was left out):

        sudo apt-get install build-essential \
                             cmake \
                             git \
                             python \
                             libboost-system-dev \
                             libboost-filesystem-dev \
                             libboost-program-options-dev \
                             libboost-regex-dev \
                             libboost-iostreams-dev \
                             libtinyxml-dev \
                             libxml2-utils \
                             ruby-dev \
                             ruby

1. Clone the repository into a directory and go into it:

        git clone https://github.com/osrf/sdformat /tmp/sdformat
        cd /tmp/sdformat

     **Note:** the `master` branch is the development branch where you'll find
     the bleeding edge code, your cloned repository should be on this branch by
     default but we recommend you switch to branch `sdf4` if you desire more
     stability

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

Gazebo-classic and Ignition Transport depend on the Ignition Messages package.

1. Install required dependencies:

        sudo apt-get install build-essential \
                             cmake \
                             git \
                             libprotoc-dev \
                             libprotobuf-dev \
                             protobuf-compiler

1. Clone the repository into a directory and go into it:

        git clone https://github.com/ignitionrobotics/ign-msgs /tmp/ign-msgs
        cd /tmp/ign-msgs

     **Note:** You can use the `ign-msgs0` branch for version 0.

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

Gazebo-classic depends on the Ignition Transport package.

Please follow the instructions on the Ignition Transport
[documents](http://ignition-transport.readthedocs.io/en/latest/installation/installation.html#install-from-sources-ubuntu-linux).

When installing dependencies, make sure you only install the
`libignition-msgs-dev` package if you haven't installed Ignition Messages from
source.

