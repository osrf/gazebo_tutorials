<a id="Ubuntu"></a>
## Gazebo in different deb packages

Gazebo 3.0 version is the first one that ships different Ubuntu debian packages following the [official packaging guidelines](https://www.debian.org/doc/manuals/maint-guide/). This changes brings an option about how to install gazebo:

 * Use gazebo as an application: for the users that just run gazebo simulator with the provided plugins and models and do not plan on developing on top of gazebo its own custom software. In this case, please install the package called ***gazebo3***.
 * Use gazebo also to develop software using gazebo libraries: for users that develop plugins or any other kind of software that needs gazebo headers and libraries. In this case, together with gazebo3 package, please install ***libgazebo-dev***. 

## Gazebo3 in ROS ##

Gazebo3 will be use by ROS-J (gazebo2 is the one used in ROS Indigo). All the details are in the [Gazebo/ROS integration page](http://gazebosim.org/tutorials?cat=connect_ros).

## Gazebo3 multi physics engines support ##

Gazebo3 is able to use different physics engines to perform the simulation. ODE is the one used by default, but support is in place for Bullet, Simbody and DART. For those users that wan't to try a different physics engine than ODE, a from source installation of gazebo is needed, so please do not use the .deb packages. For those that just need ODE, .deb packages are just fine.

#Download and Install Gazebo#

### Ubuntu Debians ###

1. Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the [https://help.ubuntu.com/community/Repositories/Ubuntu Ubuntu guide] for instructions on doing this.

1. Setup your computer to accept software from packages.osrfoundation.org.

***Note:*** there is a list of [available mirrors](https://bitbucket.org/osrf/gazebo/wiki/gazebo_mirrors) for this repository which could improve the download speed.

  **Ubuntu Linux 12.04 (precise)**

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'

  **Ubuntu Linux 13.04 (raring)**

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu raring main" > /etc/apt/sources.list.d/gazebo-latest.list'

  **Ubuntu Linux 13.10 (saucy)**

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu saucy main" > /etc/apt/sources.list.d/gazebo-latest.list'

1. Retrieve and install the keys for the Gazebo repositories.

        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

1. Update apt-get and install Gazebo.

        sudo apt-get update
        sudo apt-get install gazebo3
        # For developers that works on top of gazebo, one extra package
        sudo apt-get install libgazebo-dev
    **There is available the old gazebo version 2 under the package name of: gazebo-current (or gazebo2 in saucy/trusty)**


1. Check your installation

        gzserver

   **Note: As of Gazebo 1.9 you no longer need to setup your .bashrc to always source Gazebo's setup.sh to export necessary paths.**

1. The first time `gzserver` is executed requires the download of some models and it could take some time, please be patient. Wait until you see a message like `Publicized address: ...` and then execute a `gazebo client`:

        gzclient

1. If you are interested in using Gazebo with [ROS](http://www.ros.org), see [Installing gazebo_ros_pkgs](http://gazebosim.org/tutorials?cat=connect_ros).

# Compiling From Source (Ubuntu)

## Prerequisites

Make sure you have removed the Ubuntu pre-compiled binaries before installing from source

    sudo apt-get remove 'gazebo-*' 'sdformat-*'

We also recommend for ROS users that your remove older ROS versions of Gazebo:

    sudo apt-get remove ros-fuerte-simulator-gazebo ros-groovy-simulator-gazebo

If you have previously installed from source, be sure you are installing to the same path location or that you have removed the previous installation from source version manually.

As a side note, default install locations:
; Pre-compiled Ubuntu Binaries : /usr/bin/gazebo
; Default source install : /usr/local/bin/gazebo

## ROS Users

When building Gazebo, we recommend you do not have your <tt>/opt/ros/*/setup.sh</tt> file sourced, as it has been seen to add the wrong libraries to the Gazebo build.

## Install Required Dependencies

Install prerequisites.  A clean Ubuntu system will need:
<!-- Ogre doesn't install on Ubuntu 13.10, so use ogre-1.8 which does -->

        sudo apt-get install build-essential libtinyxml-dev libboost-all-dev cmake mercurial pkg-config \
                     libprotoc-dev libprotobuf-dev protobuf-compiler libqt4-dev libtar-dev \
                     libcurl4-openssl-dev libcegui-mk2-dev libopenal-dev libtbb-dev \
                     libswscale-dev libavformat-dev libavcodec-dev libogre-1.8-dev libgts-dev libltdl3-dev \
                     playerc++ libxml2-dev libfreeimage-dev freeglut3-dev

**Note: ** in Precise replace libogre-1.8-dev by libogre-dev

## Optional Physics Engines

**Release Note: in order to use a physics engine different than ODE, a full compilation of gazebo from source is needed (as detailed in this document). The .deb packages are only shipping the ODE physics engine.**

Gazebo supports multiple physics engines in addition to the modified version of ODE that is used internally.

   ***Bullet Support*** [Bullet](http://code.google.com/p/bullet/) version 2.82 is needed for Gazebo 3.0. In an Ubuntu system (precise - saucy) the OSRF repo can be used to install the proper package. Be sure to follow Step 2 in the [Ubuntu Debians section above](http://gazebosim.org/tutorials?tut=install_old_versions&ver=3.0&cat=install#Ubuntu) to configure your computer to accept software from packages.osrfoundation.org

        sudo apt-get update        
        sudo apt-get install libbullet2.82-dev

   ***Simbody Support*** [Simbody](https://simtk.org/home/simbody/) version 3.3 is supported for Gazebo version 2.0.0 and later. In an Ubuntu system (precise - saucy) the OSRF repo can be used to install the proper package. Be sure to follow Step 2 in the [Ubuntu Debians section above](http://gazebosim.org/tutorials?tut=install_old_versions&ver=3.0&cat=install#Ubuntu) to configure your computer to accept software from packages.osrfoundation.org

        sudo apt-get update
        sudo apt-get install libsimbody-dev

   ***DART Support*** Support for [DART](http://dartsim.github.io/) version 3.0 is being integrated into Gazebo version 2.2 (pending [pull request 850](https://bitbucket.org/osrf/gazebo/pull-request/850/dart-physics-engine-support/diff)). In an Ubuntu system (precise - saucy) several Personal Package Archives (PPA's) can be used to install the proper package and dependencies. Note that adding these PPA's may cause conflicts with ROS.

        sudo apt-add-repository ppa:libccd-debs
        sudo apt-add-repository ppa:fcl-debs
        sudo apt-add-repository ppa:dartsim
        sudo apt-get update
        sudo apt-get install libdart-core3-dev

## Optional Dependencies ##

   ***GUI test Support*** (Optional) To correctly parse the results of GUI regression tests, the xsltproc package is needed.

        sudo apt-get install xsltproc

   ***Man Page Support*** (Optional) To generate man-pages for the Gazebo executables, the ruby-ronn package is needed.

        sudo apt-get install ruby-ronn

   ***Player Support*** (Optional) To use Player/Stage:

        sudo apt-get install libplayer*

## Build And Install SDFormat ##

To install from source, you should first install the SDFormat package, then build Gazebo off of that:

1. Clone the repository into a directory in your home folder:

        mkdir ~/gazebo_source
        cd ~/gazebo_source/
        hg clone https://bitbucket.org/osrf/sdformat

1. Change directory into the sdformat repository and switch to the 2.0 branch

        cd sdformat
        hg up sdf_2.0

   **Note: the <tt>default</tt> branch is the development branch where you'll find the bleeding edge code, your cloned repository should be on this branch by default but we recommend you switch to the 2.0 branch if you desire more stability**

1. Create a build directory and go there

        mkdir build
        cd build

1. Build and install

        cmake .. -DCMAKE_INSTALL_PREFIX=/usr
        make -j4
        sudo make install

## Build And Install Gazebo ##

1. Clone the repository into a directory in your home folder:

        cd ~/gazebo_source/
        hg clone https://bitbucket.org/osrf/gazebo

1. Change directory in the Gazebo repository and switch to the 3.0 branch

        cd gazebo
        hg up gazebo_3.0

   **Note: the <tt>default</tt> branch is the development branch where you'll find the bleeding edge code, your cloned repository should be on this branch by default but we recommend you switch to the 3.0 branch if you desire more stability**

1. Create a build directory and go there

        mkdir build
        cd build

1. Configure Gazebo (choose either method `a` or `b` below)
   
  > a. Release mode: This will generate optimized code, but will not have debug symbols. Use this mode if you don't need to use GDB.
  
  >        cmake ../
  
  > Note: A big part of the compilation is the test suite. If it is useful to temporary disable it during the developemnt, you can use:

  >        cmake ../ -DENABLE_TESTS_COMPILATION:BOOL=False

  > Note: You can use a custom install path to make it easier to switch between source and debian installs:

  >        cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../
  
  > b. Debug mode: This will generate code with debug symbols. Gazebo will run slower, but you'll be able to use GDB.
  
  >        cmake -DCMAKE_BUILD_TYPE=Debug ../
  
1. The output from `cmake ../` may generate a number of errors and warnings about missing packages. You must install the missing packages that have errors and re-run `cmake ../`. Make sure all the build errors are resolved before continuing (they should be there from the earlier step in which you installed prerequisites). Warnings alert of optional packages that are missing.

1. Make note of your install path, which is output from `cmake` and should look something like:

          -- Install path: /home/$USER/local     

1. Build Gazebo

        make -j4

1. Install Gazebo

        sudo make install

   **Note: As of Gazebo 1.9 you no longer need to setup your .bashrc to always source Gazebo's setup.sh to export necessary paths.**

1. Setup environment variables

  ***Local Install***: If you decide to install gazebo in a local directory you'll need to modify some of your PATHs

        echo "export LD_LIBRARY_PATH=<install_path>/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
        echo "export PATH=<install_path>/local/bin:$PATH" >> ~/.bashrc
        echo "export PKG_CONFIG_PATH=<install_path>/local/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
        source ~/.bashrc

1. Now try running gazebo:

        gazebo

  ***If Gazebo runs successfully, you're done!.*** If Gazebo was installed to `/usr/local/` and running gazebo throws an error similar to:

        gazebo: error while loading shared libraries: libgazebo_common.so.1: cannot open shared object file: No such file or directory

  , then `/usr/local/lib` is not in load path (default behavior for Ubuntu). Run the following commands and then try running gazebo again:

        echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf 
        sudo ldconfig

1. If you are interested in using Gazebo with [ROS](http://www.ros.org), see [Installing gazebo_ros_pkgs](http://gazebosim.org/tutorials?cat=connect_ros).

## Uninstalling Source-based Install ##

If you need to uninstall Gazebo or switch back to a debian-based install of Gazebo when you currently have installed Gazebo from source, navigate to your source code directory's build folders and run make uninstall:

        cd ~/gazebo_source/gazebo/build
        sudo make uninstall
        cd ~/gazebo_source/sdformat/build
        sudo make uninstall

# Compiling From Source (Mac OS X) #

Gazebo and several of its dependencies can be compiled on OS X with [Homebrew](http://brew.sh) using the [osrf/simulation tap](https://github.com/osrf/homebrew-simulation). Here are the instructions:

1. Install [homebrew](http://brew.sh): `ruby -e "$(curl -fsSL https://raw.github.com/mxcl/homebrew/go/install)"`

2. Install [XQuartz](http://xquartz.macosforge.org/landing/), which provides X11 support and is required by Gazebo and OGRE

3. For 10.8 and earlier, install [Xcode command-line tools](http://stackoverflow.com/questions/9329243/xcode-4-4-and-later-install-command-line-tools) by downloading them from Apple. For 10.9 and later, they should prompt you to install them when you install Homebrew in step 1.

4. Run the following commands:

        brew tap osrf/simulation
        brew install gazebo3
        gazebo

## Optional dependencies ##
The gazebo formula has two optional dependencies: the [Bullet](https://code.google.com/p/bullet/) and [Simbody](https://github.com/simbody/simbody) physics engines. To install with these physics engines:

        brew install gazebo3 --with-bullet --with-simbody

## Versions ##
The formula currently installs version 3.0 of gazebo.
Version 1.9 can be installed using the gazebo formula, or gazebo 2.2 using gazebo2.
To install the latest version of gazebo's default branch:

        brew install gazebo3 --HEAD

