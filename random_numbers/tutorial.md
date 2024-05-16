# Introduction

Gazebo-classic makes use of a random number generator. By default the seed is set to the PID of the process running Gazebo. It is possible to manually set the random number seed. The advantage of this feature is to obtain a deterministic sequence of random numbers, which is good for test repeatability.

# Command line

Gazebo-classic can be initialized with a random number seed on the command line using the `--seed` argument:

~~~
gazebo --seed <integer>
~~~

# Message

Gazebo-classic listens to the `~/world_control` topic, which requires messages of type [msgs::WorldControl](http://osrf-distributions.s3.amazonaws.com/gazebo/msg-api/dev/world__control_8proto.html). The world control message may contain a random number seed.
