# Install Gazebo on Mac (using homebrew)

Gazebo and several of its dependencies can be compiled on OS X with
[Homebrew](http://brew.sh) using the
[osrf/simulation tap](https://github.com/osrf/homebrew-simulation).
Gazebo is straightforward to install on Mac OS X 10.11 (El Capitan) or higher.
Installation on older versions requires changing the default standard library
and rebuilding dependencies due to the
[use of c++11](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1340/).
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
