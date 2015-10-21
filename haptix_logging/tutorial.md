# Overview

This tutorial will explain how to use the Gazebo logging capabilities for
saving your simulation episode on disk and being able to reproduce it
afterwards.

We assume that you have already done the
[installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix)
and you are also familiar with the Swarm API.

# A Gazebo log file

A Gazebo log file is a collection of State logs. State logs are recordings of
world state information from Gazebo. State includes pose, velocity, acceleration,
and forces applied to all links of all models. Gazebo will only record state
information for models that change over time. A state log file contains a header,
the initial world description, and a time series of state.

# Enable Gazebo logging from the GUI

Start Gazebo and click on the top right icon of the menu bar when you are ready
to start your logging.

%%%
[[file:files/logging_1.png|800px]]
%%%

A new pop-up window will appear. Click on the red record button to start the log.
Click on the same icon again when needed for stop logging.

%%%
[[file:files/logging_2.png|800px]]
%%%

A new time stamped directory should exist in `~/.gazebo/log` with one
subdirectory and a `state.log` file. Here is an example:

~~~
~/.gazebo/log/2013-07-25T07\:29\:05.122275/gzserver/state.log
~~~

# Enable Gazebo logging from your C++ program

Follow the steps described in the
[HAPTIX C-API tutorial](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_comm)
for compiling a program using your favorite operating system. Copy the code from
[**here**](http://bitbucket.org/osrf/haptix-comm/raw/default/example/hx_controller.c)
and paste it in your current project.

Uncomment the following blocks for enable logging:

~~~
// Uncomment this block for start logging.
// if (hxs_start_logging("/tmp") != hxOK)
//   printf("hxs_start_logging(): error.\n");
~~~

and:

~~~
// Uncomment this block for stop logging.
// if (hxs_stop_logging() != hxOK)
//   printf("hxs_stop_logging(): error.\n");
~~~

When executed, this example should create a `/tmp/log/` directory containing a
`state.log` file.

# Enable Gazebo logging from your MATLAB/Octave program

Follow the
[HAPTIX Matlab and Octave API tutorial](http://gazebosim.org/tutorials?cat=haptix&tut=haptix_matlab)
and load the `hx_matlab_controller` in Octave or MATLAB.

Uncomment the following blocks for enable logging:

~~~
% Uncomment this block for start logging.
% hxs_start_logging('/tmp/log/')
~~~

and:

~~~
% Uncomment this block for stop logging.
% hxs_stop_logging()
~~~

When executed, this example should create a `/tmp/log/` directory containing a
`state.log` file.

# Playback a log file

Follow the next steps for replaying your previously recorded log file in Gazebo:

Open a new terminal and run:

~~~
gazebo -p /tmp/log/state.log
~~~
