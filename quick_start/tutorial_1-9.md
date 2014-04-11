#Tutorial: Quick Start 1.9

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

### Where are the worlds located?

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
