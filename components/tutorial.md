This page describes each of the items involved in running a Gazebo simulation.

# World Files

The world description file contains all the elements in a simulation, including robots, lights, sensors, and static objects. This file is formatted using [SDF (Simulation Description Format)](http://gazebosim.org/sdf.html), and typically has a `.world` extension.

The Gazebo server (`gzserver`) reads this file to generate and populate a world.

A number of example worlds are shipped with Gazebo. These worlds are installed in `<install_path>/share/gazebo-<version>/worlds`;
you can also see them in the [source code](https://github.com/osrf/gazebo/blob/master/worlds/).

# Model Files

A model file uses the same [SDF](http://gazebosim.org/sdf.html) format as world files, but should only contain a single `<model> ... </model>`. The purpose of these files is to facilitate model reuse, and simplify world files. Once a model file is created, it can be included in a world file using the following SDF syntax:

~~~
<include>
  <uri>model://model_file_name</uri>
</include>
~~~

A number of models are provided in the [online model database](http://github.com/osrf/gazebo_models) (in previous versions, some example models were shipped with Gazebo).  Assuming that you have an Internet connection when running Gazebo, you can insert any model from the database and the necessary content will be downloaded at runtime.

Read more about model files [here](http://gazebosim.org/tutorials?tut=build_model).

# Environment Variables

Gazebo uses a number of environment variables to locate files, and set up
communications between the server and clients.  Default values that work for
most cases are compiled in. This means you don't need to set any variables.

Here are the variables:

`GAZEBO_MODEL_PATH`: colon-separated set of directories where Gazebo will search for models

`GAZEBO_RESOURCE_PATH`: colon-separated set of directories where Gazebo will search for other resources such as world and media files.

`GAZEBO_MASTER_URI`: URI of the Gazebo master. This specifies the IP and port where the server will be started and tells the clients where to connect to.

`GAZEBO_PLUGIN_PATH`: colon-separated set of directories where Gazebo will search for the plugin shared libraries at runtime.

`GAZEBO_MODEL_DATABASE_URI`: URI of the online model database where Gazebo will download models from.

These defaults are also included in a shell script:

~~~
source <install_path>/share/gazebo/setup.sh
~~~

If you want to modify Gazebo's behavior, e.g., by extending the path it searches for models, you should first source the shell script listed above, then modify the variables that it sets.

## New in Gazebo 8

Parts of Gazebo transitioned to use the [Ignition Transport](https://ignitionrobotics.org/libs/transport)
library for inter-process communication instead of the built-in
Gazebo Transport library. Some features such as
[markers](https://github.com/osrf/gazebo/blob/gazebo8/examples/stand_alone/marker/)
and the plotting utility are using Ignition Transport and may be affected by
the following environment variables:

`IGN_PARTITION`: Partition name for all Ignition Transport nodes.

`IGN_IP`: Similar to `GAZEBO_MASTER_URI`, but for Ignition Transport.

`IGN_VERBOSE`: Show debug information from Ignition Transport.

Read more about Ignition Transport environment variables
[here](https://ignitionrobotics.org/tutorials/transport/4.0/md__data_ignition_ign-transport_tutorials_20_env_variables.html).

# Gazebo Server

The server is the workhorse of Gazebo. It parses a world description file given on the command line, and then simulates the world using a physics and sensor engine.

The server can be started using the following command.  Note that the server does not include any graphical interface; it's meant to run headless.

~~~
gzserver <world_filename>
~~~

The `<world_filename>` can be:

1. relative to the current directory,

2. an absolute path, or

3. relative to a path component in `GAZEBO_RESOURCE_PATH`.

4. `worlds/<world_name>`, where `<world_name>` is a world that is installed with Gazebo

For example, to use the `empty_sky.world` which is shipped with Gazebo, use the following command

~~~
gzserver worlds/empty_sky.world
~~~

# Graphical Client

The graphical client connects to a running `gzserver` and visualizes the elements. This is also a tool which allows you to modify the running simulation.

The graphical client is run using:

~~~
gzclient
~~~

# Server + Graphical Client in one

The `gazebo` command combines server and client in one executable.  Instead of running `gzserver worlds/empty.world` and then `gzclient`, you can do this:

~~~
gazebo worlds/empty_sky.world
~~~

# Plugins

Plugins provide a simple and convenient mechanism to interface with Gazebo.
Plugins can either be loaded on the command line, or specified in an SDF file
(see the [SDF](http://gazebosim.org/sdf.html) format).

Plugins specified on the command line are loaded first, then plugins specified
in the SDF files are loaded. Some plugins are loaded by the server, such as
plugins which affect physics properties, while other plugins are loaded by the
graphical client to facilitate custom GUI generation.

Example of loading a system plugin via the command line:

~~~
gzserver -s <plugin_filename>
~~~

The `-s` flag indicates it is a system plugin, and `<plugin_filename>` is the
name of a shared library found in `GAZEBO_PLUGIN_PATH`.
For example, to load the `RestWebPlugin` that ships with Gazebo:

~~~
gzserver --verbose -s libRestWebPlugin.so
~~~

The same mechanism is used by the graphical client, the supported command line
flags are the following:

* Gazebo 7 and earlier: Use `-g` to load a GUI plugin
* Gazebo 8 and later: Use `--gui-client-plugin` to load a GUI plugin

For example, to load the `TimerGUIPlugin`:

~~~
gzclient --gui-client-plugin libTimerGUIPlugin.so
~~~

For more information refer to the [plugins overview](http://classic.gazebosim.org/tutorials/?tut=plugins_hello_world) page.
