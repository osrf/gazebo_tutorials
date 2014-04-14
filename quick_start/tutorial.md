#Tutorial: Quick Start 2.0

### Run an empty world

First make sure Gazebo is [installed](http://gazebosim.org/#download).

Next, open a terminal.

Then start gazebo with an empty world, by entering the following at the command prompt.

~~~
gazebo
~~~

### Run a world with a robot

Let's simulate something a bit more interesting by loading a world with a pioneer2dx.

At a terminal prompt, enter the following command.

~~~
gazebo worlds/pioneer2d.world
~~~

#### Where are the worlds located?

You may have noticed the mysterious `worlds/pioneer2dx.world` argument in the above command.
This instructs gazebo to find the `pioneer2dx.world` file, and load it on start.

These world files are located in a versioned system directory, for example `/usr/share/gazebo-3.0` on Ubuntu.
If you have Gazebo 3.0 installed on Ubuntu, in a terminal type the following to see a complete list of worlds.

~~~
ls /usr/share/gazebo-3.0/worlds
~~~

For a Gazebo 3.0 installation on OS X using homebrew, type the following to see a complete list of worlds.

~~~
ls /usr/local/share/gazebo-3.0/worlds
~~~

### Client and server separation

The `gazebo` command actually run two different executables for you. The
first is called `gzserver`, and the second `gzclient`.

The `gzserver` executable runs the physics update-loop and sensor data
generation. This is core of Gazebo, and can be use independent of any
graphical interface. You may see the phrase "run headless" thrown about in
the forums. This phrase equates to running only the `gzserver`. An example
use case would involve running `gzserver` on a cloud computer where a user
interface is not needed.

The `gzclient` executable runs the [QT](http://qt-project.org) based user
interface. This application provides a nice visualization of simulation, and
convenient controls over various simulation properties.

Try running each of these executables. Open a terminal and run the server:

~~~
gzserver
~~~

Open another terminal and run the graphical client:

~~~
gzclient
~~~

At this point you should see the Gazebo user interface. You restart the
`gzclient` application as often as you want, and even run multiple
interfaces.
