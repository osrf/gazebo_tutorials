# Introduction

State logs are recordings of world state information from Gazebo. State includes pose, velocity, acceleration, and forces applied to all links of all models. Gazebo will only record state information for models that change over time. A state log file contains a header, the initial world description, and a time series of state.

# Gazebo Log Command line tool

Gazebo ships with a logging utility that is accessed via the `gz log` command.

View the help information using:

~~~
gz help log
~~~

or

~~~
gz log -h
~~~

# Example Usage

> **Tip**: Check out the
[tutorial](/tutorials?tut=logging_playback)
on logging and playback for an overview of ways to record a log.

## Step 1: Create a state log file

Start by removing old log files

~~~
rm -rf ~/.gazebo/log/*
~~~

We will use the PR2 world to create a state log file.

Start by running the Gazebo server with the `-r` command line option

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

The `gz log` command line tool provides mechanisms for stepping through a log file and echoing the contents of a log file to screen. The echo to screen feature can be combined with a filter to produce a log file that contains specific information such as just the pose of models and links.

Try echoing the recorded state log file to screen.

~~~
gz log -e -f ~/.gazebo/log/*/gzserver/state.log
~~~

You should see a lot of information scroll by.

Now let's remove all velocity, acceleration, and force information from the log file. This will leave just pose information.

~~~
gz log -e -f ~/.gazebo/log/*/gzserver/state.log --filter *.pose/*.pose
~~~

The `--filter` option is a flexible command line argument to extract information from a log file.

Some guidelines for using the `--filter` flag:

* Use `/` to separate entities in a hierarchy, for example `<model_name>/<link_name>`

* Use `.` to access an entity's property, for example `<model_name>.pose.y` or `<model_name>/<link_name>.velocity`

  Some available properties are:

  **Model**: pose

  **Link**: pose, velocity, acceleration, wrench

  **Pose, velocity, acceleration or wrench**: x, y, z, r (roll), p (pitch), a (yaw)

* Use `*` as a wildcard, for example `*.pose` or `<model_name>/right_*`

* Use `[]` to match any of the lower level properties, for example `<model_name>/pose.[x,y,a]`

It is also possible to filter based on simulation time using a Hz filter.
For example, we can output state information at 30 Hz using:

~~~
gz log -e -f ~/.gazebo/log/*/gzserver/state.log -z 30
~~~

These filters can be combined and piped to a file for playback. This may take some time depending on the size of the state.log.

~~~
gz log -e -f ~/.gazebo/log/*/gzserver/state.log -z 30 --filter *.pose/*.pose > /tmp/filtered_state.log
~~~

This log file can then be replayed in Gazebo

~~~
gazebo -p /tmp/filtered_state.log
~~~




