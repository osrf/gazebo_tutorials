#Tutorial: Random numbers#

Gazebo makes use of a random number generator. By default the seed is set to the PID of the process running Gazebo. It is possible to manually set the random number seed. The advantage of this feature is to obtain a deterministic sequence of random numbers, which is good for test repeatability.

## Command line

Gazebo can be initialized with a random number seed on the command line using the `--seed` argument:

~~~
gazebo --seed <integer>
~~~

## Message

Gazebo listens to the `~/world_control` topic, which requires messages of type [msgs::WorldControl](http://gazebosim.org/msgs/1.9.0/world__control_8proto.html). The world control message may contain a random number seed.
