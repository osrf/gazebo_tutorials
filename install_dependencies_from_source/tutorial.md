# Install dependencies from source

This tutorial will go through the process of installing some of Gazebo's
dependencies from source. The dependencies listed here are all maintained by
the Gazebo team and often new features in Gazebo are tied to new features in
these libraries.

These libraries are:

* [SDFormat](http://sdformat.org/)
* [ignition-math](http://ignitionrobotics.org/libraries/math)
* [ignition-transport](http://ignitionrobotics.org/libraries/transport)
* [ignition-msgs](http://ignitionrobotics.org/libraries/messages)

[[file:files/gazebo_dependency_tree.svg|400px]]

## A bit of history

All the libraries listed here are evolutions of libraries which were at some
point built within the Gazebo project itself. In an effort to make these
available for other projects and to make Gazebo more modular, they have been
extracted from Gazebo.

### SDFormat

Gazebo has had a dependency on SDFormat since early versions.

* Gazebo 1.9 - SDFormat > 1?
* Gazebo 2.2 - SDFormat > 1.4.7 and < 2.0
* Gazebo 3 - SDFormat > 2.0.1 and < 3.0
* Gazebo 4 - SDFormat > 2.0.1 and < 4.0
* Gazebo 5 - SDFormat > 2.3.1 and < 4.0
* Gazebo 6 - SDFormat > 3.1.1 and < 4.0
* Gazebo 7 - SDFormat 4.1.0
* Gazebo 8 - SDFormat 4.1.0
* Gazebo 9 - SDFormat TBD

### Ignition Math

Gazebo has a dependency on Ignition Math from version 6.

* Gazebo 6 - Ignition math 2.0
* Gazebo 7 - Ignition math 2.4
* Gazebo 8 - Ignition math 3 - The built-in gazebo::math library is completely deprecated.
* Gazebo 9 - Ignition math TBD - The built-in gazebo::math library is completely removed.

### Ignition Transport

Gazebo has a dependency on Ignition Transport from version 7.

* Gazebo 7 - Ignition transport 1 or 2
* Gazebo 8 - Ignition transport 3
* Gazebo 9 - Ignition transport TBD

### Ignition Messages

Gazebo has a dependency on Ignition Messages from version 8.

* Gazebo 8 - Ignition msgs 0.4
* Gazebo 9 - Ignition msgs TBD

## Build and install Ignition Math from source

SDFormat, Ignition Messages and Gazebo depend on the Ignition Math library.

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-math /tmp/ign-math
        cd /tmp/ign-math

     **Note:** the `default` branch is the development branch where
you'll find the bleeding edge code, your cloned repository should be on this
branch by default but we recommend you switch to the `ign-math3` branch if you
desire more stability (with the `hg up ign-math3` command).

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode: This will generate optimized code, but will not have debug symbols. Use this mode if you don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. Gazebo will run slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

    >        cmake ../

1. Build and install:

        make -j4
        sudo make install

## Build and install SDFormat from source

Gazebo depends on the SDFormat package.

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/osrf/sdformat /tmp/sdformat
        cd /tmp/sdformat

     **Note:** the `default` branch is the development branch where you'll find
the bleeding edge code, your cloned repository should be on this branch by
default but we recommend you switch to branch `sdf4` if you desire more
stability

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode: This will generate optimized code, but will not have debug symbols. Use this mode if you don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. Gazebo will run slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

    >        cmake ../

1. Build and install:

        make -j4
        sudo make install

## Build and install Ignition Messages from source

Gazebo and Ignition Transport depend on the Ignition Messages package.

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-msgs /tmp/ign-msgs
        cd /tmp/ign-msgs

     **Note:** Ignition messages hasn't released version 1.0 yet. You can use the `default` branch for version 0.

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode: This will generate optimized code, but will not have debug symbols. Use this mode if you don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. Gazebo will run slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

    >        cmake ../

1. Build and install:

        make -j4
        sudo make install

## Build and install Ignition Transport from source

Gazebo depends on the Ignition Transport package.

1. Clone the repository into a directory and go into it:

        hg clone https://bitbucket.org/ignitionrobotics/ign-msgs /tmp/ign-msgs
        cd /tmp/ign-msgs

     **Note:** the `default` branch is the development branch where
you'll find the bleeding edge code, your cloned repository should be on this
branch by default but we recommend you switch to the `ign-transport3` branch if you
desire more stability (with the `hg up ign-transport3` command).

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure build (choose either method `a` or `b` below):

    a. Release mode: This will generate optimized code, but will not have debug symbols. Use this mode if you don't need to use GDB.

           cmake ../


    > Note: You can use a custom install path to make it easier to switch between source and debian installs:

    >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

    b. Debug mode: This will generate code with debug symbols. Gazebo will run slower, but you'll be able to use GDB.

           cmake -DCMAKE_BUILD_TYPE=Debug ../

    >        cmake ../

1. Build and install:

        make -j4
        sudo make install
