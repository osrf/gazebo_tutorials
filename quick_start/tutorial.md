# Run Gazebo

These three steps will run Gazebo with a default world.

1. [Install]( http://gazebosim.org/tutorials?cat=installation) Gazebo.

2. Open a terminal. On most Ubuntu systems you can press `CTRL+ALT+t`

3. Start Gazebo by entering the following at the command prompt.

    ~~~
    gazebo
    ~~~

# Run Gazebo with a robot

Let's simulate something a bit more interesting by loading a world with a pioneer2dx.

1. Open a terminal and enter the following command.

    ~~~
    gazebo worlds/pioneer2dx.world
    ~~~

## Where are the worlds located?

You may have noticed the mysterious `worlds/pioneer2dx.world` argument in the above command. This instructs gazebo to find the `pioneer2dx.world` file, and load it on start.

World files are located in a versioned system directory, for example `/usr/share/gazebo-4.0` on Ubuntu.  If you have Gazebo 4.0 installed on Ubuntu, in a terminal type the following to see a complete list of worlds.

~~~
ls /usr/share/gazebo-4.0/worlds
~~~

For a Gazebo 4.0 installation on OS X using Homebrew, type the following to see a complete list of worlds.

~~~
ls /usr/local/share/gazebo-4.0/worlds
~~~

# Client and server separation

The `gazebo` command actually run two different executables for you. The
first is called `gzserver`, and the second `gzclient`.

The `gzserver` executable runs the physics update-loop and sensor data
generation. This is core of Gazebo, and can be used independently of any
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
