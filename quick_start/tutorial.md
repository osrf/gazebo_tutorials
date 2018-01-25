# Run Gazebo

These three steps will run Gazebo with a default world.

1. [Install]( http://gazebosim.org/tutorials?cat=install) Gazebo.

2. Open a terminal. On most Ubuntu systems you can press `CTRL+ALT+t`

3. Start Gazebo by entering the following at the command prompt.

    ~~~
    gazebo
    ~~~

    > Note: The first time you launch gazebo, it will try to download a couple of models so this process may take some time.

# Run Gazebo with a robot

Let's simulate something a bit more interesting by loading a world with a pioneer2dx.

1. Open a terminal and enter the following command.

    ~~~
    gazebo worlds/pioneer2dx.world
    ~~~

    > Note: If you don't have the pioneer2dx model already, Gazebo will download it from the online model database which may take some time.

## Where are the worlds located?

You may have noticed the mysterious `worlds/pioneer2dx.world` argument in the above command. This instructs gazebo to find the `pioneer2dx.world` file, and load it on start.

World files are located in a versioned system directory, for example `/usr/share/gazebo-7` on Ubuntu.  If you have Gazebo 7.0 installed on Ubuntu, in a terminal type the following to see a complete list of worlds.

~~~
ls /usr/share/gazebo-7/worlds
~~~

For a Gazebo 7.0 installation on OS X using Homebrew, type the following to see a complete list of worlds.

~~~
ls /usr/local/share/gazebo-7/worlds
~~~

# Client and server separation

The `gazebo` command actually runs two different executables for you. The
first is called `gzserver`, and the second `gzclient`.

The `gzserver` executable runs the physics update-loop and sensor data
generation. This is the core of Gazebo, and can be used independently of a
graphical interface. You may see the phrase "run headless" thrown about. 
This phrase equates to running only the `gzserver`. An example
use case would involve running `gzserver` on a cloud computer where a user
interface is not needed.

The `gzclient` executable runs a [QT](http://qt-project.org) based user
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
