# Overview

The Gazebo GUI overlay can be thought of as a transparent 2D layer that sits
on top of the render window. QT widgets can be added to this layer through
a plugin interface. You can show or hide all GUI overlays by clicking on
`View->GUI Overlays` on the main Gazebo menu bar. This tutorial describes how to
create and use GUI overlay plugins to create custom interfaces for Gazebo.

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
    wget https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh
    wget https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc
    wget https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/CMakeLists.txt
    ~~~

1. Take a look at the header file.

    ~~~
    gedit GUIExampleSpawnWidget.hh
    ~~~

    > A GUI overlay plugin must inherit from the GUIPlugin class, and use Qt's `Q_OBJECT` macro.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh' from='/.*class GAZEBO_VISIBLE/' to='/.*Q_OBJECT/' />

    > The rest of the plugin may contain any code that is required to make the plugin meet your needs. In this example, we will use a QT slot to receive button presses:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh' from='/.*\\\\\\ \\brief Callback triggered/' to='OnButton\(\);' />

    > We will also use Gazebo's factory functionality to send SDF spawn messages to gzserver:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh' from='/.*\\\\\\ \\brief Node used/' to='/factoryPub;/' />

1. Take a look at the source file.

    ~~~
    gedit GUIExampleSpawnWidget.cc
    ~~~

    > The constructor in this file uses QT to create a button and attach it to our `OnButton` callback:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*\\\\ Create a push/' to='/OnButton\(\)\)\);/' />

    > The constructor also connects to Gazebo's transport mechanism and creates a factory publisher:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*\\\\ Create a node/' to='/factory\"\);/' />

    > The `OnButton` callback creates a new sphere SDF string:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*std::ostringstream/' to='/\/sdf>\";/' />

    > and sends the string to Gazebo:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc' from='/.*msgs::Factory msg/' to='/Publish\(msg\);/'/>

1. Compile the plugin

    ~~~
    cd ~/gazebo_gui_spawn
    mkdir build
    cd build
    cmake ../
    make
    ~~~

1. Now we need to make sure Gazebo can find the plugin. You can do this by
appending the `build` directory the `GAZEBO_PLUGIN_PATH` environment variable:

    ~~~
    export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
    ~~~

    Note that the command above works only for the current shell. To make sure the plugin will work when opening new terminals, install the plugin into a common search path, such as `/usr/local/lib`, or into one of the paths specified by the `GAZEBO_PLUGIN_PATH` library.

1. We also need to tell Gazebo that it should load the overlay plugin.

    There are two methods to accomplish this.

    1. **SDF world file:** Modify a world SDF file to contain the GUI plugin. For example:

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/spawn_widget_example.world'/>

        Tip: Download the world file above:

            ~~~
            cd ~/gazebo_gui_spawn
            wget https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_spawn/spawn_widget_example.world
            ~~~

    1. **GUI INI file:** Modify the ~/.gazebo/gui.ini file:

        ~~~
        gedit ~/.gazebo/gui.ini
        ~~~

        Add the following lines

        ~~~
        [overlay_plugins]
        filenames=libgui_example_spawn_widget.so
        ~~~

1. Now when Gazebo is run, a button should appear in the upper left of the render window.

    If you created a custom SDF world file with with GUI plugin:

    ~~~
    gazebo spawn_widget_example.world
    ~~~

    or if you modified `~/.gazebo/gui.ini`

    ~~~
    gazebo
    ~~~

    [[file:files/spawn_button_6-0.png|640px]]

1. Click on the button to spawn spheres.

    [[file:files/spawn_sphere_6-0.png|640px]]


# Example 2: Display Simulation Time

The source code for this example is found [here](https://bitbucket.org/osrf/gazebo/src/default/examples/plugins/gui_overlay_plugin_time).

1. Start by creating a working directory

    ~~~
    mkdir ~/gazebo_gui_time
    cd ~/gazebo_gui_time
    ~~~

1. Download the source code for the GUI overlay plugin

    ~~~
    wget https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh
    wget https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc
    wget https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/CMakeLists.txt
    ~~~

1. Take a look at the header file.

    ~~~
    gedit GUIExampleTimeWidget.hh
    ~~~

    > Just as in the first example, this plugin inherits from the GUIPlugin class, and use Qt's `Q_OBJECT` macro.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*class GAZEBO_VISIBLE/' to='/.*Q_OBJECT/' />

    > We use `SetSimTime` signal as a thread safe mechanism to update the displayed simulation time.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*\\\\\\ \\brief A signal used/' to='/.*_string\);/' />

    > An `OnStats` callback is used to receive information from Gazebo.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*\\\\\\ \\brief Callback that/' to='/_msg\);/' />

    > We will also use Gazebo's transport mechanism to receive messages from Gazebo.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh' from='/.*\\\\\\ \\brief Node used to/' to='/statsSub;/' />

1. Take a look at the source file.

    ~~~
    gedit GUIExampleTimeWidget.cc
    ~~~

    > In the constructor, we create a QLabel to display the time, and connect it to the `SetSimeTime` signal.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc' from='/.*\\\\ Create a time label/' to='/QueuedConnection\);/' />

    > The constructor also connects to Gazebo's `~/world_stats` topic.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc' from='/.*\\\\ Create a node for/' to='/this\);/' />

    > When a message is received, the `OnStats` function is called and the displayed time is updated.

    > <include src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc' from='/void GUIExampleTimeWidget::OnStats/' to='/\)\)\);/' />

1. Follow the same steps as the previous tutorial to compile the plugin,
tell Gazebo where to find it and load it via `gui.ini` or an SDF  world file.

    > **Tip:** You can add both plugins to `gui.ini` as follows:

    ~~~
    gedit ~/.gazebo/gui.ini
    ~~~

    > Change the `[overlay_plugins]` section to be:

    ~~~
    [overlay_plugins]
    filenames=libgui_example_spawn_widget.so:libgui_example_time_widget.so
    ~~~

    > This will load both the spawn sphere plugin from the previous example and the time plugin from this example.

1. When Gazebo is run, a new text box to the right of the spawn button should show the simulation time.

    ~~~
    gazebo
    ~~~

    [[file:files/time_6-0.png|640px]]

