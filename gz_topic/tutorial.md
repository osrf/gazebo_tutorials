# Overview

Gazebo provides the `gz` command-line interface which allows introspection
and interaction with a running simulation.
This tutorial describes the functionality of the `gz topic` tool which
focuses on topics published using the `gazebo-transport` interface.
To interact with topics published using `ignition-transport`,
please see the [documentation for `ign topic`](https://github.com/ignitionrobotics/ign-transport/blob/ign-transport9/tutorials/10_logging.md#running-the-examples).

# `gz topic --list`

To list the topics currently being published in a gazebo simulation,
use the `--list` or `-l` parameter.

With a `gzserver` running in one terminal (with the empty.world by default),
run the following in another terminal:

~~~
gz topic --list
~~~

which produces the following output:

~~~
/gazebo/default/atmosphere
/gazebo/default/diagnostics
/gazebo/default/factory
/gazebo/default/factory/light
/gazebo/default/gui
/gazebo/default/joint
/gazebo/default/light/modify
/gazebo/default/log/control
/gazebo/default/log/status
/gazebo/default/model/info
/gazebo/default/model/modify
/gazebo/default/physics
/gazebo/default/physics/contacts
/gazebo/default/playback_control
/gazebo/default/pose/info
/gazebo/default/pose/local/info
/gazebo/default/pose/modify
/gazebo/default/request
/gazebo/default/response
/gazebo/default/undo_redo
/gazebo/default/user_cmd
/gazebo/default/user_cmd_stats
/gazebo/default/visual
/gazebo/default/wind
/gazebo/default/world_control
/gazebo/default/world_stats
/gazebo/server/control
/gazebo/world/modify
~~~

# `gz topic --info`

To get more information about a particular gazebo-transport topic, use the
`--info` or `-i` parameter.

~~~
gz topic -i /gazebo/default/world_stats
~~~

which prints the type of message published on the topic and information
about publishers and subscribers of that topic.

~~~
Type: gazebo.msgs.WorldStatistics

Publishers:
	192.168.0.197:38057

Subscribers:

~~~

# `gz topic --echo`

To view the contents of messages published on a particular gazebo-transport
topic, use the `--echo` or `-e` parameter. To echo messages for a finite
duration of time, use the `--duration` or `-d` parameter as well.

~~~
gz topic --echo /gazebo/default/world_stats --duration 1
~~~

~~~
sim_time {
  sec: 656
  nsec: 31000000
}
pause_time {
  sec: 0
  nsec: 0
}
real_time {
  sec: 657
  nsec: 883386244
}
paused: false
iterations: 656031

sim_time {
  sec: 656
  nsec: 230000000
}
pause_time {
  sec: 0
  nsec: 0
}
real_time {
  sec: 658
  nsec: 83766729
}
paused: false
iterations: 656230
~~~

# `gz topic --hz` and `gz topic --bw`

The `--echo` command can be quite verbose. If you want to see only the
frequency with which messages are published on a topic, use the `--hz`
or `-z` parameter. If you want to see the bandwidth used by the topic,
use `--bw` or `-b`.

~~~
gz topic --hz /gazebo/default/world_stats
~~~

~~~
Hz:   4.98
Hz:   4.96
Hz:   5.01
Hz:   4.99
Hz:   5.00
Hz:   4.98
Hz:   4.98
Hz:   4.99
~~~

~~~
gz topic --bw /gazebo/default/world_stats
~~~

~~~
Total[182.85 B/s] Mean[33.36 B] Min[33 B] Max[34 B] Messages[11]
Total[185.19 B/s] Mean[33.40 B] Min[33 B] Max[34 B] Messages[10]
Total[185.08 B/s] Mean[33.40 B] Min[33 B] Max[34 B] Messages[10]
Total[185.16 B/s] Mean[33.40 B] Min[33 B] Max[34 B] Messages[10]
Total[185.02 B/s] Mean[33.40 B] Min[33 B] Max[34 B] Messages[10]
~~~

# `gz topic --pub`

Added in `gazebo10` by [Bitbucket pull request 2951](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2951/page/1),
a message can be published from the command-line by using the `--pub`
or `-p` parameter. This command requires encoding the message as a string
argument to the `--msg` or `-m` parameter. An easy way to see the syntax
for these messages is to use `gz topic --echo` to display a message of
similar type. In general, fields with numeric or string values are placed
on their own line of a multi-line string with as `name: value`, while
container messages use curly braces `{}`. Recall the content of a
[WorldStatics](http://osrf-distributions.s3.amazonaws.com/gazebo/msg-api/9.0.0/world__stats_8proto.html)
essage from the example for `gz topic --echo`:

~~~
sim_time {
  sec: 656
  nsec: 31000000
}
pause_time {
  sec: 0
  nsec: 0
}
real_time {
  sec: 657
  nsec: 883386244
}
paused: false
iterations: 656031
~~~

The message type is automatically inferred from the topic, so a publisher
or subscriber for that topic must already exist in order to use this command.

Example of using this command are given in the following subsections.

## Pause and unpause

As a simple example, we can replicate the functionality of `gz world --pause`
by publishing to the `~/world_control` topic:

In terminal A: open `gz stats` to see whether the simulation is running
or paused.

In terminal B: use `gz topic --echo /gazebo/default/world_control` to
watch for published messages.

In terminal C: use `gz world --pause 1` to pause the simulation and confirm
that `gz stats` pauses in terminal A. This should show the following in
terminal B:

~~~
pause: true

~~~

In terminal D: use `gz topic --pub` to unpause the simulation by publishing
the opposite message to `/gazebo/default/world_control`.

~~~
gz topic --pub /gazebo/default/world_control --msg 'pause: false'
~~~

## Spawn a model with orientation specified by a quaternion

~~~
gz topic -p /gazebo/default/factory \
         -m 'sdf_filename: "model://double_pendulum_with_base"
pose {
  position {
    x: 1.23
    y: 2.34
    z: 3.45
  }
  orientation {
    x: 0.5
    y: 0.5
    z: 0.5
    w: 0.5
  }
}'
~~~

## Change the gzclient user camera pose

~~~
gz topic -p /gazebo/default/gui \
         -m 'camera {
  name: "this_is_required_but_not_parsed"
  pose {
    position {
      x: 1.23
      y: 2.34
      z: 3.45
    }
    orientation {
      x: 0.5
      y: 0.5
      z: 0.5
      w: 0.5
    }
  }
}'
~~~
