#Overview

This tutorial will create a source file that is a system plugin designed to save images into the director `/tmp/gazebo_frames`.

# Source code
#
We'll start with the source file. Create a file called `system_gui.cc` with the following contents:

~~~
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
    public: virtual ~SystemGUI()
    {
      if (this->userCam)
        this->userCam->EnableSaveFrame(false);
    }

    public: virtual void Load(int /*_argc*/, char ** /*_argv*/)
    {
    }

    private: virtual void Init()
    {
      // Get a pointer to the active user camera
      this->userCam = gui::get_active_camera();

      // Enable saving frames
      this->userCam->EnableSaveFrame(true);

      // Specify the path to save frames into
      this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
    }

    private: rendering::UserCameraPtr userCam;
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}
~~~

Both the `Load` and `Init` functions must not block. The `Load` function is called at startup, before Gazebo-classic is loaded. The `Init` function is called after Gazebo-classic has been loaded.

In our `Init` function, we get a pointer to the user camera (the camera used in the graphical interface) and enable saving of frames.

1.  Get the user camera

        this->userCam = gui::get_active_camera();
2.  Enable save frames

        this->userCam->EnableSaveFrame(true);
3.  Set the location to save frames

        this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");

# Compiling Camera Plugin

Create a CMakeLists.txt file with the following:

~~~
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

add_library(system_gui SHARED system_gui.cc)
target_link_libraries(system_gui ${GAZEBO_LIBRARIES} ${OGRE_LIBRARIES})
~~~

Note for Gazebo-classic 1.10 and later, you will need to depend on `OGRE-Terrain` as well by adding these lines to you `CMakeLists.txt`:

~~~
pkg_check_modules(OGRE-Terrain OGRE-Terrain)
include_directories(${OGRE-Terrain_INCLUDE_DIRS})
~~~

Make a build directory, run cmake, and compile. You should end up with a libsystem_gui.so library.

Create a `build` directory

~~~
mkdir build; cd build
~~~

Compile the plugin

~~~
cmake ../; make
~~~

Make sure the location of the plugin is in your $GAZEBO_PLUGIN_PATH

~~~
cd build
export GAZEBO_PLUGIN_PATH=$PWD:$GAZEBO_PLUGIN_PATH
~~~

# Running Plugin

First start gzserver in the background:

~~~
gzserver &
~~~

Run the client with plugin:

~~~
gzclient -g libsystem_gui.so
~~~

Inside `/tmp/gazebo_frames` you should see many saved images from the current plugin.


Note: Remember to also terminate the background server process after you quit the client. In the same terminal, bring the process to foreground:

~~~
$ fg
~~~

and press `Ctrl-C` to abort the process. Alternatively, just kill the `gzserver` process:

~~~
$ killall gzserver
~~~
