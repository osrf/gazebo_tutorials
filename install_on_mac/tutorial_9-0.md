# Install Gazebo on Mac (using homebrew)

Gazebo and several of its dependencies can be compiled on OS X with
[Homebrew](http://brew.sh) using the
[osrf/simulation tap](https://github.com/osrf/homebrew-simulation).
Gazebo is straightforward to install on Mac OS X 10.11 (El Capitan) or higher.
Installation on older versions requires changing the default standard library
and rebuilding dependencies due to the
[use of c++11](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-request/1340/).
For purposes of this tutorial, I will assume OS X 10.11 or greater is in use.

## Default installation: one-liner

1. Install

        curl -ssL http://get.gazebosim.org | sh

2. Run

        gazebo

## Alternative installation: step-by-step

1. Install [homebrew](http://brew.sh), which should also prompt you to install
the XCode command-line tools:

        ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

2. Install [XQuartz](http://xquartz.macosforge.org/landing/), which provides
X11 support and is required by Gazebo and OGRE

3. Run the following commands to install gazebo quickly with a precompiled binary:

        brew tap osrf/simulation
        brew install gazebo9
        gazebo

## Optional dependencies
The Gazebo formula has an optional dependency on the
[DART](http://dartsim.github.io) physics engine,
which must be installed using the following procedure
prior to installing Gazebo9 without using the precompiled binary:

        brew tap dartsim/dart
        brew install dartsim
        # Gazebo will be built with DART (autodetected), bullet and simbody
        brew install gazebo9 --build-from-source

## Versions
The formula currently installs version 9.0 of Gazebo. Version 8 can be
installed using the `gazebo8` formula and Gazebo 7
using `gazebo7`. To install the latest version of Gazebo's default branch:

        brew install gazebo9 --HEAD
