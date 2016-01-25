# Overview

This tutorial will explain how to use the Gazebo logging capabilities for
recording your simulation and being able to reproduce it afterwards.

# Gazebo log files

Gazebo log files are compressed `.log` files which contain an initial full
description of the whole world, followed by a series of "world states".

The initial description contains complete information about everything in the
world. Each state following that contains information about changes in the
world, such as:

* Simulation stats such as the current simulation time and the number of physics
iterations.

* Current state of each model in the scene, as well as the state of each link
and joint in the model. It includes information such as instantaneous pose,
velocity, acceleration and forces.

* Current pose of each light in the world.

> **Tip**: You can find the whole spec for the world state [here](http://sdformat.org/spec?ver=1.5&elem=state).

In this tutorial we will record a few log files and then take a peak inside
them in the end.

# Record a log

## Logging from the GUI

1. Start your simulation. Here, as an example, we have a simple world with a
double pendulum.

1. Click on the logging icon on the top right to bring up the Data Logger.

[[file:files/data_logger.png|800px]]

1. You can choose the directory where your log file will be saved by clicking
the `Browse` button. By default, log files go to the `~/.gazebo/log` directory.

1. Click on the red button to start recording. You should see the number of
bytes in your log file increasing on the right.

> **Note**: For efficiency, only models and lights which move over time are
logged. If your scene is static, the number of bytes in your log file will not
increase.

1. Click on the red button again to stop logging.

1. Expand `Recordings` to see the path to the `state.log` file which was
generated. It will be inside a time-stamped directory.

[[file:files/recordings.png|800px]]

## Logging from the command line

From the command line, it is possible to log the whole simulation from the
moment Gazebo starts running until it stops, or to trigger logging from an
arbritary time.

### Logging the whole simulation

As an example, we will record the `random_velocity.world` as follows:

    gazebo -r --record_path ~/logs/random_velocity worlds/random_velocity.world

The log file will only be terminated when Gazebo is closed.

Here are some logging options. You can check what options there are running
`gazebo --help`.

* **-p [--play] arg**: Play a log file.

* **-r [ --record ]**: Record a log from the moment Gazebo is opened until it is
closed.

* **--record_encoding arg**: Compression encoding format for log data. The
options are `zlib` (default), `bz2` and `txt`.

### Logging part of the simulation

Gazebo also provides the `gz log` tool, which can be used to trigger logging at
any moment. While Gazebo is running, open another terminal and run the following
to start recording:

    gz log -d 1

And to stop:

    gz log -d 0

Check out `gz log --help` for other options.

# Playback a log file

Once you have a log file, you can replay it visually or introspect it in several
ways.

## Visualize in GUI

Currently, it is not possible to open a log file from the GUI, so playback must
be started from the command line. Simply start Gazebo using the `-p` option
to specify a log file, such as:

    gazebo -u -p ~/logs/pendulum/2016-01-25T13\:55\:16.923263/gzserver/state.log

> **Tip**: The `-u` option starts the log paused.

Gazebo will open in playback mode. You can play, pause, rewind and step through
the playback. It's also possible to input a specific time to jump to.

[[file:files/playback_gui.png|800px]]

## Command line tools

As mentioned above, the `gz log` tool provides several options for introspecting
your log file. Check out
[this](http://gazebosim.org/tutorials?tut=log_filtering&cat=tools_utilities)
tutorial for log filtering.

Here, let's quickly go over how you would take a look at the recorded states.

We'll use `-s` to step through a recorded files, like this:

    gz log -s -f ~/.gazebo/log/2016-01-25T14\:23\:13.042235/gzserver/state.log

You'll see the full initial SDF representation of the world, something like
this:




As you press `space`, you will step through the subsequent states. You'll note
that the states are more compact and only contain information about what has
changed in the world.







