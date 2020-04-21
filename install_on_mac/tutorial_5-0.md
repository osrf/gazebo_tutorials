# Install Gazebo on Mac (using homebrew)

## Step-by-step Install

Gazebo and several of its dependencies can be compiled on OS X with
[Homebrew](http://brew.sh) using the
[osrf/simulation tap](https://github.com/osrf/homebrew-simulation).
Gazebo 5 is straightforward to install on Mac OS X 10.9 (Mavericks) or higher.
Installation on older versions requires changing the default standard library
and rebuilding dependencies due to the
[use of c++11](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-request/1340/).
For purposes of this tutorial, I will assume OS X 10.9 or greater is in use.
Here are the instructions:

1. Install [homebrew](http://brew.sh), which should also prompt you to install
the XCode command-line tools:

        ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

2. Install [XQuartz](http://xquartz.macosforge.org/landing/), which provides
X11 support and is required by Gazebo and OGRE

3. Run the following commands:

        brew tap osrf/simulation
        brew install gazebo5
        gazebo

## Optional dependencies
The Gazebo formula has several optional dependencies:
the [Bullet](https://code.google.com/p/bullet/)
and [Simbody](https://github.com/simbody/simbody) physics engines.
To install with these physics engines:

        brew install gazebo5 --with-bullet --with-simbody

The [DART](http://dartsim.github.io) physics engine can be used as well,
  though it must be installed using the following procedure
  prior to installing Gazebo.

        brew tap dartsim/dart
        brew install dartsim --core-only
        # Gazebo will be built with DART (autodetected), bullet and simbody
        brew install gazebo5 --with-bullet --with-simbody

## Versions
The formula currently installs version 5.0 of Gazebo.
Version 1.9 can be installed using the `gazebo` formula,
Gazebo 2.2 using `gazebo2`, Gazebo 3 using `gazebo3`, and Gazebo 4 using
`gazebo4`.
To install the latest version of Gazebo's default branch:

        brew install gazebo5 --HEAD
