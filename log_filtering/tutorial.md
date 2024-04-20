# Tutorial: State Log Filtering

## Introduction

State logs are recordings of world state information from Gazebo. State includes pose, velocity, acceleration, and forces applied to all links of all models. Gazebo-classic will only record state information for models that change over time. A state log file contains a header, the initial world description, and a time series of state.

## Step 1: Create a state log file

Start by removing old log files

~~~
rm -rf ~/.gazebo/log/*
~~~

We will use the PR2 world to create a state log file.

Start by running the Gazebo-classic server with the `-r` command line option

~~~
gzserver -r worlds/pr2.world
~~~

After a few seconds, stop the server using ctrl-c.

A new time stamped directory should exist in `~/.gazebo/log` with one subdirectory and a `state.log` file. Here is an example

~~~
~/.gazebo/log/2013-07-25T07\:29\:05.122275/gzserver/state.log
~~~

You can verify this log file by replaying it in Gazebo.

~~~
gazebo -p ~/.gazebo/log/*/gzserver/state.log
~~~

## Step 2: Filter a state log file

The `gzlog`
command line tool provides mechanisms for stepping through a log file and echoing the contents of a log file to screen.
The echo to screen feature can be combined with a filter to produce a log file
that contains specific information such as just the pose of models and links.

Try echoing the recorded state log file to screen.

~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log
~~~

You should see a lot of information scroll by.

Now let's remove all velocity, acceleration, and force information from the log file. This will leave just pose information.

~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log --filter *.pose/*.pose
~~~

The `--filter` option is a flexible command line argument to extract information from a log file.

It is also possible to filter based on simulation time using a Hz filter.
For example, we can output state information at 30 Hz using:

~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log -z 30
~~~

These filters can be combined and piped to a file for playback. This may take some time depending on the size of the state.log.

~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log -z 30 --filter *.pose/*.pose > /tmp/filtered_state.log
~~~

This log file can then be replayed in Gazebo

~~~
gazebo -p /tmp/filtered_state.log
~~~
