# Install Gazebo from source (Ubuntu and Mac)

## Install Gazebo from source on Ubuntu

### Prerequisites

For compiling the latest version of gazebo you will need an Ubuntu distribution
equal to 16.04 (Xenial) or newer.

Make sure you have removed the Ubuntu pre-compiled binaries before installing
from source:

    sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'

If you have previously installed from source, be sure you are installing to the
same path location or that you have removed the previous installation from
source version manually.

As a side note, default install locations:

  1. Pre-compiled Ubuntu Binaries : `/usr/bin/gazebo`

  2. Default source install : `/usr/local/bin/gazebo`

#### ROS Users

When building Gazebo, we recommend you do not have your */opt/ros/\*/setup.sh*
file sourced, as it has been seen to add the wrong libraries to the Gazebo
build.

### Install Required Dependencies

In a clean Ubuntu installation you can install pre-compiled versions of all dependencies:

1. Setup your computer to accept software from packages.osrfoundation.org.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

1. Setup keys and update

        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        sudo apt-get update

1. Install prerequisites. A clean Ubuntu system will need the following (replace `version` with the major version of gazebo you intend to build, eg: 7, 8, 9. And if using ROS, replace `dummy` with your ROS version, eg: indigo, jade,
    kinetic...):

        wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
        GAZEBO_MAJOR_VERSION=version ROS_DISTRO=dummy . /tmp/dependencies.sh
        echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\\' | xargs sudo apt-get -y install

### Optional Physics Engines

**Release Note:** in order to use DART, a full compilation of Gazebo from
source is needed (as detailed in this document). The .deb packages are
shipping the ODE, Bullet, and Simbody physics engines.

#### DART Support

Support for [DART](http://dartsim.github.io/) version 6 is integrated into
the default branch. In an Ubuntu system, several Personal Package Archives
(PPA's) can be used to install the proper package and dependencies. Note that
adding these PPA's may cause conflicts with ROS.

    # Main repository
    sudo apt-add-repository ppa:dartsim
    sudo apt-get update
    sudo apt-get install libdart6-dev

    # Optional DART utilities
    sudo apt-get install libdart6-utils-urdf-dev

### Optional Dependencies

#### GUI test Support

To correctly parse the results of GUI regression tests, the xsltproc package is needed.

    sudo apt-get install xsltproc

#### Man Page Support

To generate man-pages for the Gazebo executables, the ruby-ronn package is needed.

    sudo apt-get install ruby-ronn

#### Player Support

    sudo apt-get install robot-player-dev*


### Dependencies managed by OSRF

Gazebo development is tightly linked to the development of a few other libraries:

* [SDFormat](http://sdformat.org/)
* [ignition-math](http://ignitionrobotics.org/libraries/math)
* [ignition-transport](http://ignitionrobotics.org/libraries/transport)
* [ignition-msgs](http://ignitionrobotics.org/libraries/messages)

If you've installed all required dependencies following the instructions above, you
already have the latest release of these libraries installed in your system.
However, when working on new features, you often need to build these libraries
from source.

If you don't need a special version of these libraries, **you can skip this
section**. If you're not sure if you need to build these from source, you can
ask for guidance at [Gazebo Answers](https://answers.gazebosim.org), explaining
your specific use case.

To build these libraries from source, first go through the
[Build dependencies from source](http://gazebosim.org/tutorials?tut=install_dependencies_from_source)
tutorial and then come back here.

### Build And Install Gazebo

1. Clone the repository into a directory and go into it:

        git clone https://github.com/osrf/gazebo /tmp/gazebo
        cd /tmp/gazebo

     **Note:** the `master` branch is the development branch where
you'll find the bleeding edge code, your cloned repository should be on this
branch by default but we recommend you switch to the `gazebo6` branch if you
desire more stability

1. Create a build directory and go there:

        mkdir build
        cd build

1. Configure Gazebo (choose either method `a` or `b` below):

    a. Release mode: This will generate optimized code, but will not have debug symbols. Use this mode if you don't need to use GDB.

        cmake ../


    > Note: You can use a custom install path to make it easier to switch between source and debian installs. We recommend using `/home/$USER/local` as the value for `<install_path>`:

    >     cmake -DCMAKE_INSTALL_PREFIX=<install_path> ../

    b. Debug mode: This will generate code with debug symbols. Gazebo will run slower, but you'll be able to use GDB.

        cmake -DCMAKE_BUILD_TYPE=Debug ../

    >     cmake ../

1. The output from `cmake ../` may generate a number of errors and warnings about missing packages. You must install the missing packages that have errors and re-run `cmake ../`. Make sure all the build errors are resolved before continuing (they should be there from the earlier step in which you installed prerequisites). Warnings alert of optional packages that are missing.

1. Make note of your install path, which is output from `cmake` and will either be the default install location or the one specified as `<install_path>` above, e.g.:

        -- Install path: /home/$USER/local

1. Build Gazebo:

        make -j4

1. Install Gazebo:

        sudo make install

1. Setup environment variables

#### Optional tests suite compilation

The generic call to `make` won't compile any of the different types of tests
present in Gazebo. While this saves a lot of time in compilations, there are
many reasons to compile and run the testing suite: submitting changes to
gazebo repository, packaging for linux distributions, etc. In order to compile
the whole gazebo test suite you'll need to run:

    make tests

#### Local Install

If you decided to install gazebo in a local directory you'll need to modify some of your PATHs:

    echo "export LD_LIBRARY_PATH=<install_path>/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
    echo "export PATH=<install_path>/bin:$PATH" >> ~/.bashrc
    echo "export PKG_CONFIG_PATH=<install_path>/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
    source ~/.bashrc

Now try running gazebo:

    gazebo

***If Gazebo runs successfully, you're done!.***

If Gazebo was installed to `/usr/local/` and running gazebo throws an error similar to:

    gazebo: error while loading shared libraries: libgazebo_common.so.1: cannot open shared object file: No such file or directory

  , then `/usr/local/lib` is not in load path (default behavior for Ubuntu). Run the following commands and then try running gazebo again:

    echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
    sudo ldconfig

1. If you are interested in using Gazebo with [ROS](http://www.ros.org),
see [Installing gazebo\_ros\_pkgs](http://gazebosim.org/tutorials?cat=connect_ros).

#### Install in a catkin workspace

Another method for installing to a local directory is to use a
[catkin workspace](http://wiki.ros.org/catkin/workspaces),
which supports plain cmake packages as well as catkin packages.
This allows multiple versions of gazebo to be installed side-by-side.
Environment variables do not need to be added to the `~/.bashrc`;
rather they are set by sourcing the appropriate setup script.
Using catkin requires the some extra python packages to be installed.
If you are using Ubuntu and have configured your system
to use the ROS package repository
([see here for instructions](http://wiki.ros.org/jade/Installation/Ubuntu#Installation)),
then you can use the following `apt-get` commands:

~~~
sudo apt-get install python-catkin-pkg python-catkin-tools
~~~

For other platforms, you can use `pip`
(you can also use `pip` with Ubuntu, but `apt-get` is recommended):

~~~
sudo pip install catkin-pkg catkin-tools
~~~

Here is an example for building gazebo against custom versions
of sdformat, [bullet](https://github.com/bulletphysics/bullet3), and
[DART](https://github.com/dartsim/dart).

First, create a workspace folder.
Since it is easy to use multiple catkin workspaces,
it is convenient to place them in a single folder, such as `~/ws`.
For this tutorial, the bash variable `WS` will be used to refer
to the absolute path of the workspace folder.
In this case, we will name the folder `gazebo_dart`:

~~~
export WS=$HOME/ws/gazebo_dart
mkdir -p ${WS}/src
~~~

Clone catkin and the packages you want to build into this folder:

~~~
cd ${WS}/src
git clone https://github.com/ros/catkin.git
git clone https://github.com/bulletphysics/bullet3.git
git clone https://github.com/dartsim/dart.git
git clone https://github.com/osrf/sdformat
git clone https://github.com/osrf/gazebo
~~~

Checkout the appropriate branch for each repository.
For example, gazebo5 doesn't support dart5.

~~~
cd ${WS}/src/gazebo
git checkout master
cd ${WS}/src/dart
git checkout release-6.2
~~~

Add [package.xml](http://wiki.ros.org/catkin/package.xml)
files for the plain cmake packages:

~~~
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_bullet.xml    > ${WS}/src/bullet3/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_dart-core.xml > ${WS}/src/dart/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml    > ${WS}/src/gazebo/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml  > ${WS}/src/sdformat/package.xml
~~~

Initialize the catkin workspace:

~~~
cd ${WS}
catkin init
~~~

Then build the workspace using `catkin build`.
Note that bullet and DART have several important cmake options.
Using bullet with gazebo requires `BUILD_SHARED_LIBS=ON`
and has better accuracy if `USE_DOUBLE_PRECISION=ON`.
Using DART with Gazebo is compatible with `BUILD_CORE_ONLY=ON`,
which requires many fewer dependencies to be installed.
For now, these options do not overlap, so they can be sent to
all the packages:

~~~
cd ${WS}
catkin build -vi --cmake-args \
  -DBUILD_CORE_ONLY=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DUSE_DOUBLE_PRECISION=ON
~~~

This will build all the packages in order with verbose output.
Omit the `-vi` option to see less console output.

Once the build has completed,
source the setup file located in `${WS}/devel/setup.bash`
to run gazebo:

~~~
. ${WS}/devel/setup.bash
gazebo -e bullet
gazebo -e dart
~~~

Other packages can be added to the catkin workspace
as long as they have a package.xml that
lists their dependencies.

### Uninstalling Source-based Install

If you need to uninstall Gazebo or switch back to a debian-based install of Gazebo when you currently have installed Gazebo from source, navigate to your source code directory's build folders and run make uninstall:

    cd ~/gazebo/build
    sudo make uninstall
    cd ~/sdformat/build
    sudo make uninstall

## Compiling From Source (Mac OS X)

Gazebo and several of its dependencies can be compiled on OS X with [Homebrew](http://brew.sh) using the [osrf/simulation tap](https://github.com/osrf/homebrew-simulation). Here are the instructions:

1. Install [homebrew](http://brew.sh): `ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`

2. Install [XQuartz](http://xquartz.macosforge.org/landing/), which provides X11 support and is required by Gazebo and OGRE

3. For 10.8 and earlier, install [Xcode command-line tools](http://stackoverflow.com/questions/9329243/xcode-4-4-and-later-install-command-line-tools) by downloading them from Apple. For 10.9 and later, they should prompt you to install them when you install Homebrew in step 1.

4. Run the following commands:

        brew tap osrf/simulation
        brew install gazebo11
        gazebo

### Optional dependencies ###
The gazebo formula has two optional dependencies: the [Bullet](https://code.google.com/p/bullet/) and [Simbody](https://github.com/simbody/simbody) physics engines. To install with these physics engines:

    brew install gazebo11 --with-bullet --with-simbody
