#Download and Install Gazebo

### Important Notes

**New in Gazebo 1.9:** the parsers for the Simulated Description Format (SDF) and the Universal Robotic Description Format (URDF) have been moved into a seperate package named SDFormat.

1. ROS Users:
 1. Only install Gazebo from here, and only follow tutorials from this website. Documentation on ros.org for Gazebo is old and not actively maintained.
 1. ROS Groovy is the last release of ROS to pull in its own version of Gazebo. As of ROS Hydro, ROS uses the standalone (system-install) version of Gazebo.
1. If you are new to Gazebo or just want to use Gazebo as a stand-alone application in Ubuntu, select the [[#Pre-compiled binaries | Pre-compiled binaries]] instructions below.
1. If you're still reading, select [[#Compiling From Source | Compiling From Source]]

# Gazebo/ROS integration

With the release of gazebo 2.0 version (as gazebo_current package) there are some [important aspects of gazebo integration into ROS](http://gazebosim.org/tutorials?cat=connect_ros) that must be read before continue.

# Pre-compiled binaries

### Ubuntu Debians

1. Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the [Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.  (Note: These are enabled by default In Ubuntu 9.04 (Jaunty) and later.)

1. Setup your computer to accept software from packages.osrfoundation.org.

  **Ubuntu Linux 12.04 (precise)**

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'

  **Ubuntu Linux 12.10 (quantal)**

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu quantal main" > /etc/apt/sources.list.d/gazebo-latest.list'

  **Ubuntu Linux 13.04 (raring)**

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu raring main" > /etc/apt/sources.list.d/gazebo-latest.list'

  **Ubuntu Linux 13.10 (saucy)**

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu saucy main" > /etc/apt/sources.list.d/gazebo-latest.list'

1. Retrieve and install the keys for the Gazebo repositories.

        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

1. Update apt-get and install Gazebo.

        sudo apt-get update
        sudo apt-get install gazebo

1. Check your installation

        gzserver

   **Note: As of Gazebo 1.9 you no longer need to setup your .bashrc to always source Gazebo's setup.sh to export necessary paths.**

1. The first time `gzserver` is executed requires the download of some models and it could take some time, please be patient. Wait until you see a message like `Publicized address: ...` and then execute a `gazebo client`:

        gzclient

1. If you are interested in using Gazebo with [ROS](http://www.ros.org), see [Installing gazebo_ros_pkgs](http://gazebosim.org/tutorials?cat=connect_ros).

# Compiling From Source (Ubuntu)

## Prerequisites

Make sure you have removed the Ubuntu pre-compiled binaries before installing from source

    sudo apt-get remove gazebo gazebo-prerelease gazebo-nightly sdformat sdformat-nightly

We also recommend for ROS users that your remove older ROS versions of Gazebo:

    sudo apt-get remove ros-fuerte-simulator-gazebo ros-groovy-simulator-gazebo

If you have previously installed from source, be sure you are installing to the same path location or that you have removed the previous installation from source version manually.

As a side note, default install locations:
; Pre-compiled Ubuntu Binaries : /usr/bin/gazebo
; Default source install : /usr/local/bin/gazebo

## ROS Users

When building Gazebo, we recommend you do not have your `/opt/ros/*/setup.sh` file sourced, as it has been seen to add the wrong libraries to the Gazebo build.


## Install Dependencies

1. Install prerequisites.  A clean Ubuntu system will need:

        sudo apt-get install build-essential libtinyxml-dev libtbb-dev libxml2-dev libqt4-dev pkg-config  libprotoc-dev libfreeimage-dev \
                             libprotobuf-dev protobuf-compiler libboost-all-dev freeglut3-dev cmake libogre-dev libtar-dev \
                             libcurl4-openssl-dev libcegui-mk2-dev libopenal-dev git

   **Bullet Support** (Optional) Bullet version 2.81 is needed for this. In an Ubuntu system (precise or quantal) the OSRF repo can be used to install the proper package. Be sure to follow Step 2 in the [[1.9/install#Ubuntu_Debians |Ubuntu Debians section above]] to configure your computer to accept software from packages.osrfoundation.org

        sudo apt-get update
        sudo apt-get install libbullet-dev

   **GUI test Support** (Optional) To correctly parse the results of GUI regression tests, the xsltproc package is needed.

        sudo apt-get install xsltproc

To install from source, you should first install the SDFormat package, then build Gazebo off of that:

## Build And Install SDFormat ##

1. Clone the repository into a directory in your home folder:

        mkdir ~/gazebo_source
        cd ~/gazebo_source/
        git clone https://github.com/osrf/sdformat

1. Change directory into the sdformat repository and switch to the 1.4 branch

        cd sdformat
        git checkout sdf_1.4

   *Note: the `default` branch is the development branch where you'll find the bleeding edge code, your cloned repository should be on this branch by default but we recommend you switch to the 1.4 branch if you desire more stability*

1. Create a build directory and go there

        mkdir build
        cd build

1. Build and install

        cmake ../
        make -j4
        sudo make install

## Build And Install Gazebo

1. Clone the repository into a directory in your home folder:

        cd ~/gazebo_source/
        git clone https://github.com/osrf/gazebo

1. Change directory in the Gazebo repository and switch to the 1.9 branch

        cd gazebo
        git checkout gazebo_1.9

   *Note: the `default` branch is the development branch where you'll find the bleeding edge code, your cloned repository should be on this branch by default but we recommend you switch to the 1.9 branch if you desire more stability*

1. Create a build directory and go there

        mkdir build
        cd build

1. Configure Gazebo (choose either method `a` or `b` below)

  > a. Release mode: This will generate optimized code, but will not have debug symbols. Use this mode if you don't need to use GDB.

  >        cmake ../

  > Note: A big part of the compilation is the test suite. If it is useful to temporary disable it during the development, you can use:

  >        cmake ../ -DENABLE_TESTS_COMPILATION:BOOL=False

  > b. Debug mode: This will generate code with debug symbols. Gazebo will run slower, but you'll be able to use GDB.

  >        cmake -DCMAKE_BUILD_TYPE=Debug ../

1. The output from `cmake ../` may generate a number of errors and warnings about missing packages. You must install the missing packages that have errors and re-run `cmake ../`. Make sure all the build errors are resolved before continuing (they should be there from the earlier step in which you installed prerequisites). Warnings alert of optional packages that are missing.

1. Make note of your install path, which is output from `cmake` and should look like:

          -- Install path: /home/$USER/local

  You can specify the the install path on the command line by defining the CMAKE_INSTALL_PREFIX :
  >        cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

1. Build Gazebo

        make
  > Note: You can decrease compile time by using more cores

  >         make -jX

  > Where X is the number of cores you want to use.

1. Install Gazebo

        sudo make install

   *Note: As of Gazebo 1.9 you no longer need to setup your .bashrc to always source Gazebo's setup.sh to export necessary paths.*

1. Setup environment variables

  **Local Install**: If you decide to install gazebo in a local directory you'll need to modify some of your PATHs

        echo "export LD_LIBRARY_PATH=<install_path>/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
        echo "export PATH=<install_path>/local/bin:$PATH" >> ~/.bashrc
        echo "export PKG_CONFIG_PATH=<install_path>/local/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
        source ~/.bashrc

1. Now try running gazebo:

        gazebo

  **If Gazebo runs successfully, you're done!.** If Gazebo was installed to `/usr/local/` and running gazebo throws an error similar to:

        gazebo: error while loading shared libraries: libgazebo_common.so.1: cannot open shared object file: No such file or directory

  , then `/usr/local/lib` is not in load path (default behavior for Ubuntu). Run the following commands and then try running gazebo again:

        echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
        sudo ldconfig

1. If you are interested in using Gazebo with [ROS](http://www.ros.org), see (Installing gazebo_ros_pkgs](http://gazebosim.org/tutorials?cat=connect_ros).

## Uninstalling Source-based Install

If you need to uninstall Gazebo or switch back to a debian-based install of Gazebo when you currently have installed Gazebo from source, navigate to your source code directory's build folders and run make uninstall:

        cd ~/gazebo_source/gazebo/build
        sudo make uninstall
        cd ~/gazebo_source/sdformat/build
        sudo make uninstall

# Compiling From Source (Mac OS X)

Coming soon...

