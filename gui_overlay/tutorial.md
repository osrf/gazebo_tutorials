# Overview

The Gazebo GUI overlay can be thought of as a transparent 2D layer that sits
on top of the render window. QT widgets can be added to this layer through
a plugin interface. This tutorial describes how to create and use GUI
overlay plugins to create custom interfaces for Gazebo.

Two examples will be used to demonstrate the GUI Overlay functionality. The
first example creates a button that spawns a sphere, and the second displays
the current simulation time. These two exmamples show how to send data to
Gazebo and receive data from Gazebo.

# Example 1: Spawn spheres

The source code for this example is found [here](https://bitbucket.org/osrf/gazebo/src/default/examples/plugins/gui_overlay_plugin_spawn).

1. Start by creating a working directory

    ~~~
    mkdir ~/gazebo_gui_spawn
    cd ~/gazebo_gui_spawn
    ~~~

1. Download the source code for the GUI overlay plugin

    ~~~
    wget https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh
    wget https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc
    wget https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/CMakeLists.txt
    ~~~

1. Take a look at the header file.

    ~~~
    gedit GUIExampleSpawnWidget.hh
    ~~~

    > A GUI overlay plugin must inherit from the GUIPlugin class, and use Qt's `Q_OBJECT` macro.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh' from='/.*class GAZEBO_VISIBLE/' to='/.*Q_OBJECT/' />

    > The rest of the plugin may contain any code that is required to make the plugin meet your needs. In this example, we will use a QT slot to receive button presses:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh' from='/.*protected slots/' to='OnButton\(\);' />

    > We will also use Gazebo's factory functionality to send SDF spawn messages to gzserver:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh' from='/.*private: transport/' to='/factoryPub;/' />

1. Take a look at the source file.

    ~~~
    gedit GUIExampleSpawnWidget.cc
    ~~~

    > The constructor in this file uses QT to create a button and attach it to our `OnButton` callback:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*QPushButton/' to='/OnButton\(\)\)\);/' />

    > The constructor also connects to Gazebo's transport mechanism and creates a factory publisher:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*this->node/' to='/factory\"\);/' />

    > The `OnButton` callback creates a new sphere SDF string:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*std::ostringstream/' to='/\/sdf>\";/' />

    > and sends the string to Gazebo:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*msgs::Factory msg/' to='/Publish\(msg\);/'/>

1. Compile the plugin

    ~~~
    cd ~/gazebo_gui_spawn
    mkdir build
    cd build
    cmake ../
    make
    ~~~

1. Now we need to make sure Gazebo can find the plugin. You can do this by modifying the `GAZEBO_PLUGIN_PATH` environment variable.

    ~~~
    export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
    ~~~

1. We also need to tell Gazebo that it should load the overlay plugin.

    ~~~
    gedit ~/.gazebo/gui.ini
    ~~~

    Add the following lines

    ~~~
    [overlay_plugins]
    filenames=libgui_example_spawn_widget.so
    ~~~

1. Now when Gazebo is run, a button should appear in the upper left of the render window.

    ~~~
    gazebo
    ~~~

    [[file:files/spawn_button.png|640px]]

1. Click on the button to spawn spheres.

    [[file:files/spawned_sphere.png|640px]]


# Example 2: Display Simulation Time

The source code for this example is found [here](https://bitbucket.org/osrf/gazebo/src/default/examples/plugins/gui_overlay_plugin_time).

1. Start by creating a working directory

    ~~~
    mkdir ~/gazebo_gui_time
    cd ~/gazebo_gui_time
    ~~~

1. Download the source code for the GUI overlay plugin

    ~~~
    wget https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh
    wget https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc
    wget https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/CMakeLists.txt
    ~~~

1. Take a look at the header file.

    ~~~
    gedit GUIExampleTimeWidget.hh
    ~~~

    > Just as in the first example, this plugin inherits from the GUIPlugin class, and use Qt's `Q_OBJECT` macro.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*class GAZEBO_VISIBLE/' to='/.*Q_OBJECT/' />

    > We use `SetSimTime` signal as a thread safe mechanism to update the displayed simulation time.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*signals: void SetSimTime/' to='/.*_string\);/' />

    > An `OnStats` callback is used to receive information from Gazebo.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*protected: void/' to='/_msg\);/' />

    > We will also use Gazebo's transport mechanism to receive messages from Gazebo.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*private: transport/' to='/statsSub;/' />

1. Take a look at the source file.

    ~~~
    gedit GUIExampleTimeWidget.cc
    ~~~

    > In the constructor, we create a QLabel to display the time, and connect it to the `SetSimeTime` signal.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc' from='/.*QLabel \*timeLabel/' to='/QueuedConnection\);/' />

    > The constructor also connects to Gazebo's `~/world_stats` topic.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc' from='/.*this->node/' to='/this\);/' />

    > When a message is received, the `OnStats` function is called and the displayed time is updated.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gui_plugins/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc' from='/void GUIExampleTimeWidget::OnStats/' to='/\)\)\);/' />

1. Compile the plugin

    ~~~
    cd ~/gazebo_gui_time
    mkdir build
    cd build
    cmake ../
    make
    ~~~

1. Tell Gazebo where to find the plugin.

    ~~~
    export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
    ~~~

1. Add this plugin to `gui.ini`

    ~~~
    gedit ~/.gazebo/gui.ini
    ~~~

    Change the `[overlay_plugins]` section to be:

    ~~~
    [overlay_plugins]
    filenames=libgui_example_spawn_widget.so:libgui_example_time_widget.so
    ~~~

    This will load both the spawn sphere plugin from the previous example and the time plugin from this example.

1. When Gazebo is run, a new text box to the right of the spawn button should show the simulation time.

    ~~~
    gazebo
    ~~~

    [[file:files/time.png|640px]]

