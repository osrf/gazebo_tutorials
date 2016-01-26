# Overview

This tutorial will explain how to use the Gazebo logging capabilities to
record your simulation and then reproduce it afterwards, using either the GUI
or the command line.

# Gazebo log files

Gazebo log files are compressed `.log` files which contain an initial full
description of the whole world, followed by a series of "world states".

The initial description contains complete information about everything in the
world, from the scene to the entities present.

After that, every time something moves in simulation, a new world state is
recorded. World states are much simpler, as they only contain information about
what changed, such as:

* Simulation stats such as the current simulation time and the number of physics
iterations.

* Current state of each model in the scene, as well as the state of each link
and joint in the model. This includes information such as instantaneous pose,
velocity, acceleration and forces.

* Current pose of each light in the world.

> **Tip**: You can find the whole spec for the world state
[here](http://sdformat.org/spec?ver=1.6&elem=state).

In this tutorial we will record a few log files and then take a peak inside
them in the end.

# Record a log

## Logging from the GUI

1. Start your simulation. Here, as an example, we have a simple world with a
double pendulum.

1. Click on the logging icon on the top right, or hit `Ctrl+D` to bring up the
Data Logger.

    [[file:files/data_logger.png|800px]]

1. You can choose the directory where your log file will be saved by clicking
the `Browse` button. By default, log files go to the `~/.gazebo/log` directory.
In this example, we will save it in the `~/logs/double_pendulum/` directory.

1. Click on the red button to start recording. You should see the number of
bytes in your log file increasing on the right.

    > **Note**: For efficiency, only models and lights which move over time are
    logged. If your scene is static, the number of bytes in your log file will
    not increase.

1. Click on the red button again to stop logging.

1. Expand `Recordings` to see the path to the `state.log` file which was
generated. It will be inside a time-stamped directory.

    [[file:files/recordings.png|400px]]

## Logging from the command line

From the command line, it is possible to log the whole simulation from the
moment Gazebo starts running until it stops, or to trigger logging from an
arbitary time.

### Logging the whole simulation

As an example, you can record the `random_velocity.world` as follows:

    gazebo -r --record_path ~/logs/random_velocity worlds/random_velocity.world

> Here are some logging options. You can check what options there are running
> `gazebo --help`.

> * **-p [--play] arg**: Play a log file.

> * **-r [ --record ]**: Record a log from the moment Gazebo is opened until it
is closed.

> * **--record_encoding arg**: Compression encoding format for log data. The
options are `zlib` (default), `bz2` and `txt`.

The log file will only be terminated when Gazebo is closed. You can check the
file was created by looking into the path given:

    $ ls ~/logs/random_velocity/
    state.log

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
to specify a log file, such as the one we recorded earlier:

    gazebo -u -p ~/logs/double_pendulum/2016-01-25T15\:09\:49.677400/gzserver/state.log

> **Tip**: The `-u` option starts the log paused.

Gazebo will open in playback mode. You can play, pause, rewind and step through
the playback.

[[file:files/playback_gui.png|800px]]

* Use `Play` / `Pause` to stop the playback.

* Use `Rewind` / `Forward` to skip to the beginning / end of the file.

* Use `Step back` / `Step forward` to skip iterations. The number of iterations
skipped each time you press a step button can be changed in the box below.

* Drag the current time marker and drop it to skip through the log.

* Input a current time on the right to skip to that sample.

## Command line tools

As mentioned above, the `gz log` tool provides several options for introspecting
your log file. Check out
[this](http://gazebosim.org/tutorials?tut=log_filtering&cat=tools_utilities)
tutorial for log filtering, for example.

Here, let's quickly go over how you would take a look at the recorded states.

We'll use `-s` to step through a recorded file, like this:

    gz log -s -f ~/logs/double_pendulum/2016-01-25T15\:09\:49.677400/gzserver/state.log

You'll see the full initial SDF representation of the world, something like
this:

    <?xml version='1.0'?>
    <gazebo_log>
    <header>
    <log_version>1.0</log_version>
    <gazebo_version>7.0.0~pre1</gazebo_version>
    <rand_seed>10622214</rand_seed>
    <log_start>43 380000000</log_start>
    <log_end>69 651000000</log_end>
    </header>

    <chunk encoding='txt'><![CDATA[
    <sdf version ='1.6'>
    <world name='default'>
      (...)
      <light name='sun' type='directional'>
        (...)
      </light>
      <model name='ground_plane'>
        (...)
      </model>
      <model name='double_pendulum_with_base'>
        (...)
      </model>
    </world>
    </sdf>]]></chunk>

    --- Press space to continue, 'q' to quit ---

As you press `space`, you will step through the subsequent states. You'll note
that the states are more compact and only contain information about what has
changed in the world. Here's an example of a state:

    <chunk encoding='txt'><![CDATA[
    <sdf version='1.6'>
    <state world_name='default'>
    <sim_time>43 380000000</sim_time>
    <real_time>43 478499228</real_time>
    <wall_time>1453763389 677873530</wall_time>
    <iterations>43380</iterations>
    <model name='double_pendulum_with_base'><pose>1.140 -1.074 -0.000 0.000 -0.000 0.000 </pose><scale>1.000 1.000 1.000</scale><link name='base'><pose>1.13998 -1.07367 -0.00000 0.00000 0.00000 -0.00042 </pose><velocity>-0.0000 0.0000 -0.0005 0.0004 0.0030 0.0001 </velocity></link><link name='lower_link'><pose>1.38969 -1.79815 1.41059 -2.45351 0.00000 -0.00042 </pose><velocity>0.0042 -0.2557 0.2659 1.9694 0.0048 0.0001 </velocity></link><link name='upper_link'><pose>1.13999 -1.07367 2.10000 2.33144 -0.00000 -0.00042 </pose><velocity>0.0063 -0.0008 -0.0005 -0.3739 0.0032 0.0001 </velocity></link></model><model name='ground_plane'><pose>0.000 0.000 0.000 0.000 -0.000 0.000 </pose><scale>1.000 1.000 1.000</scale><link name='link'><pose>0.00000 0.00000 0.00000 0.00000 -0.00000 0.00000 </pose><velocity>0.0000 0.0000 0.0000 0.0000 -0.0000 0.0000 </velocity></link></model></state></sdf>
    ]]></chunk>

    --- Press space to continue, 'q' to quit ---

Note that there's no information for the `sun` or the `ground_plane`, since they
are not moving.

