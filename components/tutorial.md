#Tutorial: Components#

This page describes each of the item involved in running a Gazebo simulation.

### World Description File ###

The world description file contains all the elements in a simulation, including robots, lights, sensors, and static objects. This file is formatted using [SDF (Simulation Description Format)](http://gazebosim.org/wiki/sdf_format), and typically has a `.world` extension.

The Gazebo server (`gzserver`) reads this file to generate and populate a world.

A number of example worlds are shipped with Gazebo. These worlds are located in `<install_path>/share/gazebo-<version>/worlds`.

### Model Files ###

A model file uses the same [SDF](http://gazebosim.org/wiki/sdf_format) format as world files, but should only contain a single `<model> ... </model>`. The purpose of these files is to facilitate model reuse, and simplify world files. Once a model file is created, it can be included in a world file using the following SDF syntax:

~~~
<include filename="model_file_name"/>
~~~

A number of models are provided in the (online model database)[http://gazebosim.org/user_guide/started__models__database.html] (in previous versions, some example models were shipped with Gazebo).  Assuming that you have an Internet connection when running Gazebo, you can insert any model from the database and the necessary content will be downloaded at runtime.

### Environment Variables ###

Gazebo uses a number of (environment variables)[http://gazebosim.org/user_guide/started__components__env.html] to locate files, and setup communications between the server and clients.

Starting with Gazebo 1.9.0, default values that work for most cases are compiled in, so that if you don't need to set any variables.  These defaults are also included in a shell script:

~~~
source <install_path>/share/gazebo/setup.sh
~~~

If you want to modify Gazebo's behavior, e.g., by extending the path it searches for models, you should first source the shell script listed above, then modify the variables that it sets.
 
### Gazebo Server ###
 
The server is the workhorse of Gazebo. It parses a world description file given on the command line, and then simulates the world using a physics and sensor engine. 
 
The server can be started using the following command.  Note that the server does not include any graphics; it's meant to run headless.

~~~ 
gzserver <world_filename>
~~~
 
The `<world_filename>` can be: (i) relative to the current directory, (ii) an absolute path, or (iii) relative to a path component in `GAZEBO_RESOURCE_PATH`. Worlds that are shipped with Gazebo are located in `<install_path>/share/gazebo-<version_number>/worlds`.

For example, to use the `empty.world` which is shipped with Gazebo, use the following command 

~~~
gzserver worlds/empty.world
~~~

### Graphical Client ###

The graphical client connects to a running `gzserver` and visualizes the elements. This is also a tool which allows you to modify the running simulation.

The graphical client is run using:

~~~
gzclient
~~~

### Server + Graphical Client in one ###

The `gazebo` command combines server and client in one executable.  Instead of running `gzserver worlds/empty.world` and then `gzclient`, you can do this:

~~~
gazebo worlds/empty.world
~~~

### Plugins ###

Plugins provide a simple and convenient mechanism to interface with Gazebo. Plugins can either be loaded on the command line, or specified in a world/model file (see the [SDF](http://gazebosim.org/wiki/sdf_format ) format). Plugins specified on the command line are loaded first, then plugins specified in the world/model files are loaded. Most plugins are loaded by the server; however, plugins can also be loaded by the graphical client to facilitate custom GUI generation.

Example of loading a plugin on the command line:

~~~
gzserver -p <plugin_filename> <world_file>
~~~

The same mechanism is used by the graphical client:

~~~
gzclient -p <plugin_filename>
~~~

For more information refer to the [plugins overview](http://gazebosim.org/wiki/tutorials/1.5/plugins/overview) page.

