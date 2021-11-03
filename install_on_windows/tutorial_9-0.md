# Install on Windows

This documentation describes how to set up a workspace for trying to compile
Gazebo on Windows. The support for the Gazebo Windows compilation has been
completed since version 9.

> **Important note**: These Windows instructions may not work for all users.
> The community is looking for a better alternative to use Gazebo on Windows.
> Please [checkout the issue tracker](https://github.com/osrf/gazebo/issues/2901)
> to know about the different alternatives.

## Important requirement: 30Gb free disk space

It is recommended to have at least 30 Gigabytes of disk space to host Gazebo
sources together with all the dependencies and compilation artifacts.

## Supported compilers

At this moment, compilation has been tested on Windows 8.1 and 10, supported when
using [Visual Studio 2017](https://www.visualstudio.com/downloads/).
Patches for other versions are welcome.

## Installation

This installation procedure uses pre-compiled binaries in a local workspace.  To
make things easier, use a MinGW shell for your editing work (such as the [Git Bash Shell](https://msysgit.github.io/)), and only use the
Windows `cmd` for configuring and building.  You might also need to
[disable the Windows firewall](http://windows.microsoft.com/en-us/windows/turn-windows-firewall-on-off#turn-windows-firewall-on-off=windows-7).

1. Make a directory to work in, e.g.:

        mkdir gz-ws
        cd gz-ws

1. Download the following dependencies into that directory:
    1. [libcurl HEAD](https://s3.amazonaws.com/osrf-distributions/win32/deps/curl-7.57.0-vc15-x64-dll-MD.zip)

    1. [libyaml]( https://s3.amazonaws.com/osrf-distributions/win32/deps/libyaml-0.1.7-vc15-x64-md.zip)

    1. [dlfcn-win32](https://s3.amazonaws.com/osrf-distributions/win32/deps/dlfcn-win32-vc15-x64-dll-MD.zip)

    1. [jsoncpp](https://s3.amazonaws.com/osrf-distributions/win32/deps/jsoncpp-1.8.4-vc15-x64-dll-MD.zip)

    1. [protobuf 3.4.1](https://s3.amazonaws.com/osrf-distributions/win32/deps/protobuf-3.4.1-vc15-x64-dll-MD.zip)

    1. [zlib](https://s3.amazonaws.com/osrf-distributions/win32/deps/libzip-1.4.0_zlip-1.2.11_vc15-x64-dll-MD.zip)

    1. [zziplib 0.13.62](https://s3.amazonaws.com/osrf-distributions/win32/deps/zziplib-0.13.62-vc12-x64-release-debug.zip)

    1. [freeImage 3.x](https://s3.amazonaws.com/osrf-distributions/win32/deps/FreeImage3180Win32Win64.zip)

    1. [boost 1.67.0](https://s3.amazonaws.com/osrf-distributions/win32/deps/boost_1_67_0.zip)

    1. [OGRE 1.10.12 rc1](https://s3.amazonaws.com/osrf-distributions/win32/deps/ogre-sdk-1.10.12-vc15-x64.zip)

    1. [bzip2 1.0.6](https://s3.amazonaws.com/osrf-distributions/win32/deps/bzip2-1.0.6-vc12-x64-release-debug.zip)

    1. [TBB 4.3](https://s3.amazonaws.com/osrf-distributions/win32/deps/tbb43_20141023oss_win.zip)

    1. [Qt 5.7.0](https://s3.amazonaws.com/osrf-distributions/win32/deps/qt-opensource-windows-x86-msvc2015_64-5.7.0.zip)

    1. [QWT 6.1.22](https://s3.amazonaws.com/osrf-distributions/win32/deps/qwt_6.1.2~osrf_qt5.zip)

    1. [ZeroMQ 4.2.3](https://s3.amazonaws.com/osrf-distributions/win32/deps/libzmq-4.2.3_cppzmq-4.2.2_vc15-x64-dll-MD.zip)

1. Unzip each of them in gz-ws.

1. Install cmake, make sure to select the "Add CMake to system path for all users" option in the install dialog box

    > [Cmake](http://www.cmake.org/download/)

1. Install Ruby 1.9 or greater. During the install process make sure add Ruby to your system paths.

    > [Ruby](http://rubyinstaller.org/downloads/)

1. Clone Ignition CMake, Common, Fuel Tools, Math, Transport, Sdformat, and Gazebo:

        git clone https://github.com/ignitionrobotics/ign-cmake -b ign-cmake0
        git clone https://github.com/ignitionrobotics/ign-common -b ign-common1
        git clone https://github.com/ignitionrobotics/ign-fuel-tools -b ign-fuel-tools1
        git clone https://github.com/ignitionrobotics/ign-math -b ign-math4
        git clone https://github.com/ignitionrobotics/ign-msgs -b ign-msgs1
        git clone https://github.com/ignitionrobotics/ign-transport -b ign-transport4
        git clone https://github.com/osrf/sdformat -b sdf6
        git clone https://github.com/osrf/gazebo -b gazebo9

1. Open a regular Windows shell (Start->Run->"cmd"->enter), and load your compiler setup by copying and pasting the following line:

        "C:\Program Files\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
   or
        "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64

Note:   Replace 2017 with 2019 in the path if running VS 2019

        "C:\Program Files\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64

   or
        "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64

1. In a Windows shell, configure and build Ignition CMake

        cd ign-cmake
        # if you want debug, run configure Debug
        .\configure
        nmake
        nmake install

    You should now have an installation of Ignition CMake in gz-ws/ign-cmake/build/install/Release.

1. In a Windows shell, configure and build Ignition Math

        cd ign-math
        # if you want debug, run configure Debug
        .\configure
        nmake
        nmake install

    You should now have an installation of Ignition Math in gz-ws/ign-math/build/install/Release.

1. In a Windows shell, configure and build Ignition Common

        cd ign-common
        # if you want debug, run configure Debug
        .\configure
        nmake
        nmake install

    You should now have an installation of Ignition Common in gz-ws/ign-common/build/install/Release.

1. In a Windows shell, configure and build Ignition Fuel Tools

        cd ign-fuel-tools
        # if you want debug, run configure Debug
        .\configure
        nmake
        nmake install

    You should now have an installation of Ignition Fuel Tools in gz-ws/ign-fuel-tools/build/install/Release.

1. In the same Windows shell, configure and build Ignition Msgs

        cd ..\..\ign-msgs
        mkdir build
        cd build
        # if you want debug, run ..\configure Debug
        ..\configure
        nmake
        nmake install

    You should now have an installation of Ignition Msgs in gz-ws/ign-msgs/build/install/Release


1. In the same Windows shell, configure and build Ignition Transport

        cd ..\..\ign-transport
        mkdir build
        cd build
        # if you want debug, run ..\configure Debug
        ..\configure
        nmake
        nmake install

    You should now have an installation of Ignition Transport in gz-ws/ign-transport/build/install/Release

1. In the same Windows shell, configure and build Sdformat

        cd ..\..\sdformat
        mkdir build
        cd build
        # if you want debug, run ..\configure Debug
        ..\configure
        nmake
        nmake install

    You should now have an installation of Sdformat in gz-ws/sdformat/build/install/Release or
    gz-ws/sdformat/build/install/Debug.

1. In the same Windows shell, configure and build Gazebo:

        cd ..\..\gazebo
        mkdir build
        cd build
        # if you want debug, run ..\configure Debug
        ..\configure
        nmake gzclient
        nmake gzserver
        nmake install

    Once this all works you should now have an installation of Gazebo in gz-ws/gazebo/build/install/Release or
    gz-ws/gazebo/build/install/Debug.

## Running

### gzserver

1. Adjust all paths to load dll

    1. if in Debug

         cd gz-ws\gazebo\build
         ..\win_addpath.bat Debug

    2. if in Release

         cd gz-ws\gazebo\build
         ..\win_addpath.bat Release

1. Create an ogre plugins.cfg file

    1. `cd gz-ws\gazebo\build\gazebo`

    1. If in Debug: Copy in the following into `plugins.cfg and replace MYUSERNAME with your actual username`

            # Define plugin folder
            PluginFolder=C:\Users\MYUSERNAME\gz-ws\ogre-sdk-1.10.12-vc15-x64\build\install\Debug\bin\Debug

            # Define plugins
            Plugin=RenderSystem_GL_d
            Plugin=Plugin_ParticleFX_d
            Plugin=Plugin_BSPSceneManager_d
            Plugin=Plugin_PCZSceneManager_d
            Plugin=Plugin_OctreeZone_d
            Plugin=Plugin_OctreeSceneManager_d

    1. If in Release: Copy in the following into `plugins.cfg`

            # Define plugin folder
            PluginFolder=C:\Users\MYUSERNAME\gz-ws\ogre-sdk-1.10.12-vc15-x64\build\install\Release\bin\Release

            # Define plugins
            Plugin=RenderSystem_GL
            Plugin=Plugin_ParticleFX
            Plugin=Plugin_BSPSceneManager
            Plugin=Plugin_PCZSceneManager
            Plugin=Plugin_OctreeZone
            Plugin=Plugin_OctreeSceneManager

    1. Copy this file into the `gui` directory

            copy plugins.cfg gui\


1. Run gzserver

        gzserver.exe ..\..\worlds\empty.world

## Debugging

Just in case that you need to debug problems on Gazebo

### Running gzserver

If you run into issues, use --verbose to get more information.

### Running gzclient

If you run into issues, use --verbose to get more information.
A known issue is that it does not run on VirtualBox 3.4, with Ubuntu 15.04 Host.
The current theory is that it does not support off-screen frame buffering.
It has been confirmed to work on VMWare Player with windws 7 guest and Ubuntu 14.04 Host.
More details will be added as testing continues.


### Building Ogre Examples

1. Download OIS

       http://sunet.dl.sourceforge.net/project/wgois/Source%20Release/1.3/ois-v1-3.zip

1. Compile OIS in Visual Studio
   Use the project in Win32/ folder

1. Place OIS headers and libs into

       ogre-...\Dependencies\include
       ogre-...\Dependencies\lib
       ogre-...\Dependencies\bin

1. Patch configure.bat inside ogre-1.8 to use

       `-DOGRE_BUILD_SAMPLES:BOOL=TRUE ..`

1. Compile as usual

        ..\configure.bat
        nmake

1. Run the demo browser using:

       # copy OIS_*.dll into the bin directory
       ogre-...\build\bin\SampleBrowser.exe
