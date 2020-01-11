# Install on Windows

This documentation describes how to set up a workspace for trying to compile
Gazebo on Windows. The support for the Gazebo Windows compilation has been
completed since version 9.

## Important requirement: 30Gb free disk space

It is recommended to have at least 30 Gigabytes of disk space to host Gazebo
sources together with all the dependencies and compilation artifacts.

## Supported compilers

At this moment, compilation has been tested on Windows 8.1 and 10, supported when
using [Visual Studio 2017](https://www.visualstudio.com/downloads/).
Patches for other versions are welcome.

## Installation

This installation procedure uses pre-compiled binaries in a local workspace.  To
make things easier, use a MinGW shell for your editing work (such as the [Git Bash Shell](https://msysgit.github.io/) with [Mercurial](http://tortoisehg.bitbucket.org/download/index.html)), and only use the
Windows `cmd` for configuring and building.  You might also need to
[disable the Windows firewall](http://windows.microsoft.com/en-us/windows/turn-windows-firewall-on-off#turn-windows-firewall-on-off=windows-7).

1. Make a directory to work in, e.g.:

        md gz-ws
        cd gz-ws

1. Download the following dependencies into that directory:
    1. [libcurl HEAD](https://s3.amazonaws.com/osrf-distributions/win32/deps/curl-7.57.0-vc15-x64-dll-MD.zip)

    1. [dlfcn-win32](https://s3.amazonaws.com/osrf-distributions/win32/deps/dlfcn-win32-vc15-x64-dll-MD.zip)

    1. [protobuf 3.4.1](https://s3.amazonaws.com/osrf-distributions/win32/protobuf-3.4.1-vc15-x64-dll-MD.zip)

    1. [zlib](https://s3.amazonaws.com/osrf-distributions/win32/deps/libzip-1.4_zlip-1.2.11_vc15-x64-dll-MD.zip)

    1. [zziplib 0.13.62](https://s3.amazonaws.com/osrf-distributions/win32/deps/zziplib-0.13.62-vc12-x64-release-debug.zip)

    1. [freeImage 3.x](https://s3.amazonaws.com/osrf-distributions/win32/deps/FreeImage3180Win32Win64.zip)

    1. [boost 1.67.0](https://s3.amazonaws.com/osrf-distributions/win32/deps/boost_1_67_0.zip)

    1. [OGRE 1.10.12 rc1](https://s3.amazonaws.com/osrf-distributions/win32/deps/ogre-sdk-1.10.12-vc15-x64.zip)

    1. [bzip2 1.0.6](https://s3.amazonaws.com/osrf-distributions/win32/deps/bzip2-1.0.6-vc12-x64-release-debug.zip)

    1. [TBB 4.3](https://s3.amazonaws.com/osrf-distributions/win32/deps/tbb43_20141023oss_win.zip)

    1. [Qt 5.7.0](https://s3.amazonaws.com/osrf-distributions/win32/deps/qt-opensource-windows-x86-msvc2015_64-5.7.0.zip)

    1. [QWT 6.1.22](https://s3.amazonaws.com/osrf-distributions/win32/deps/qwt_6.1.2~osrf_qt5.zip)
    
    1. [Eigen 3.3.4](https://s3.amazonaws.com/osrf-distributions/win32/deps/eigen3-3.3.4.zip)
    
    1. [libZMQ 4.2.3](https://s3.amazonaws.com/osrf-distributions/win32/deps/libzmq-4.2.3_cppzmq-4.2.2_vc15-x64-dll-MD.zip)
    
    1. [JOM 1.1.3](http://ftp.fau.de/qtproject/official_releases/jom/jom_1_1_3.zip)

1. Unzip each of them in gz-ws.

1. Install cmake, make sure to select the "Add CMake to system path for all users" option in the install dialog box

    > [Cmake](http://www.cmake.org/download/)

1. Install Ruby 1.9 or greater. During the install process make sure add Ruby to your system paths.

    > [Ruby](http://rubyinstaller.org/downloads/)

1. Clone Ignition CMake, Math, Msgs, Transport, Sdformat, and Gazebo:

        hg clone https://bitbucket.org/ignitionrobotics/ign-cmake -r ign-cmake0
        hg clone https://bitbucket.org/ignitionrobotics/ign-math -r ign-math4
        hg clone https://bitbucket.org/ignitionrobotics/ign-msgs -r ign-msgs1
        hg clone https://bitbucket.org/ignitionrobotics/ign-transport -r ign-transport4
        hg clone https://bitbucket.org/osrf/sdformat -r sdf6
        hg clone https://bitbucket.org/osrf/gazebo -r gazebo9

1. Open a regular Windows shell (Start->Run->"cmd"->enter), and load your compiler setup by copying and pasting the following line:

        "C:\Program Files\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
   or
   
        "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64


1. Still in gz-ws, set up the build process. Decide how many CPU cores to use for the build. Example for using 4 cores is:
  
        set PATH=%cd%\jom_1_1_3;%PATH%
        set NMAKE=jom -j4
        set CMAKE=cmake .. -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX="install\Release" -DCMAKE_BUILD_TYPE="Release" -DBUILD_TESTING:BOOL=False

1. In a Windows shell, configure and build Ignition CMake

        cd ign-cmake
        md build
        cd build
        %CMAKE%
        %NMAKE%
        %NMAKE% install
        set CMAKE_PREFIX_PATH=%cd%\install\Release\lib\cmake\ignition-cmake0

    You should now have an installation of Ignition CMake in gz-ws/ign-cmake/build/install/Release.

1. In a Windows shell, configure and build Ignition Math

        cd ..\..\ign-math
        md build
        cd build
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\..\..\eigen3-3.3.4\share\eigen3\cmake
        %CMAKE%
        %NMAKE%
        %NMAKE% install
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\install\Release\lib\cmake\ignition-math4

    You should now have an installation of Ignition Math in gz-ws/ign-math/build/install/Release.

1. In a Windows shell, configure and build Ignition Msgs

        cd ..\..\ign-msgs
        md build
        cd build
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\..\..\protobuf-3.4.1-vc15-x64-dll-MD\cmake
        set PATH=%PATH%;%cd%\..\..\protobuf-3.4.1-vc15-x64-dll-MD\bin
        %CMAKE%
        %NMAKE%
        %NMAKE% install
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\install\Release\lib\cmake\ignition-msgs1

    You should now have an installation of Ignition Msgs in gz-ws/ign-math/build/install/Release.

1. In the same Windows shell, configure and build Ignition Transport

        cd ..\..\ign-transport
        md build
        cd build
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\..\..\libzmq-4.2.3_cppzmq-4.2.2_vc15-x64-dll-MD\CMake
        set CMAKE=%CMAKE% -DCPPZMQ_HEADER_PATH=%cd%\..\..\libzmq-4.2.3_cppzmq-4.2.2_vc15-x64-dll-MD\include
        %CMAKE%
        %NMAKE%
        %NMAKE% install
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\install\Release\lib\cmake\ignition-transport4

    You should now have an installation of Ignition Trasport in gz-ws/ign-transport/build/install/Release

1. In the same Windows shell, configure and build Sdformat

        cd ..\..\sdformat
        md build
        cd build
        set BOOST_ROOT=%cd%\..\..\boost_1_67_0
        %CMAKE%
        %NMAKE%
        %NMAKE% install
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\install\Release\lib\cmake\sdformat

    You should now have an installation of Sdformat in gz-ws/sdformat/build/install/Release

1. In the same Windows shell, configure and build Gazebo:

        cd ..\..\gazebo
        md build
        cd build
        set LD_LIBRARY_PATH=%cd%\..\..\tbb43_20141023oss\lib\intel64\vc12
        set LIB=%LIB%;%cd%\..\..\boost_1_67_0\lib64-msvc-14.1
        set CL=/I%cd%\..\..\dlfcn-win32-vc15-x64-dll-MD\include /I%cd%\..\..\tbb43_20141023oss\include /wd4251 /wd4275 /wd4146
        set OGRE_PATH=%cd%\..\..\ogre-sdk-1.10.12-vc15-x64
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\..\..\ogre-sdk-1.10.12-vc15-x64\CMake
        set CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%;%cd%\..\..\qt-opensource-windows-x86-msvc2015_64-5.7.0\lib\cmake\Qt5
        
        set CMAKE=%CMAKE% -Dfreeimage_LIBRARIES=%cd%\..\..\FreeImage\Dist\x64\FreeImage.lib -Dfreeimage_INCLUDE_DIRS=%cd%\..\..\FreeImage\Dist\x64\
        set CMAKE=%CMAKE% -DQWT_WIN_INCLUDE_DIR=%cd%\..\..\qwt_6.1.2~osrf_qt5\include -DQWT_WIN_LIBRARY_DIR=%cd%\..\..\qwt_6.1.2~osrf_qt5\Release\qwt-6.1.2-vc12-x64
        set CMAKE=%CMAKE% -DCURL_LIBRARY=%cd%\..\..\curl-7.57.0-vc15-x64-dll-MD\lib\libcurl_imp.lib -DCURL_INCLUDE_DIR=%cd%\..\..\curl-7.57.0-vc15-x64-dll-MD\include
        set CMAKE=%CMAKE% -DOGRE_VERSION=1.10.12 -DOGRE_FOUND=1 -DOGRE-RTShaderSystem_FOUND=1 -DOGRE-Terrain_FOUND=1 -DOGRE-Overlay_FOUND=1
        set CMAKE=%CMAKE% -DOGRE_PLUGINDIR=%OGRE_PATH%\bin -DOGRE_INCLUDE_DIRS=%OGRE_PATH%\include;%OGRE_PATH%\include\OGRE;%OGRE_PATH%\include\OGRE\RTShaderSystem;%OGRE_PATH%\include\OGRE\Terrain;%OGRE_PATH%\include\OGRE\Paging -DOGRE_LIBRARIES=%OGRE_PATH%\lib\OgreMain.lib;%OGRE_PATH%\lib\OgreRTShaderSystem.lib;%OGRE_PATH%\lib\OgreTerrain.lib;%OGRE_PATH%\lib\OgrePaging.lib;%OGRE_PATH%\lib\OgreOverlay.lib
        set CMAKE=%CMAKE% -Dlibdl_include_dir=%cd%\..\..\dlfcn-win32-vc15-x64-dll-MD\include -Dlibdl_library=%cd%\..\..\dlfcn-win32-vc15-x64-dll-MD\lib\dl.lib
        set CMAKE=%CMAKE% -DTBB_INCLUDEDIR=%cd%\..\..\tbb43_20141023oss\include
        
        %CMAKE%
        %NMAKE%
        
1. If you want, you can run the `install` target to get all the required files in a nice filesystem structure and with working `setup.bat` scripts
        
        %NMAKE% install
        
    Sometimes, the install target copies only a few files (not hundreds). In that case, run
    
        cmake -P cmake_install.cmake

    Once this all works you should now have an installation of Gazebo in gz-ws/gazebo/build/install/Release

## Running

### gzserver

1. Adjust all paths to load dll

         cd gz-ws\gazebo
         win_addpath.bat

1. If you want to run server-side rendering (e.g. cameras), start up some X server (e.g.  [Xming](https://sourceforge.net/projects/xming/)) and set the DISPLAY variable to the display it provides (e.g. `:0` in Xming default settings)

         set DISPLAY=:0


1. Run gzserver

        gzserver.exe empty.world
        
### gzclient

1. The steps are the same as for gzserver. Setting the `DISPLAY` variable and running X server is required.

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
