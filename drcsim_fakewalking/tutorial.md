# Overview

This tutorial will explain how to drive simulated Atlas around as if it were a wheeled robot (i.e., without walking or balancing).

## Setup

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install).

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

## Background

Note that this tutorial does not use the [walking controller](http://gazebosim.org/tutorials/?tut=drcsim_walking&cat=drcsim). It simply spawns Atlas with position controllers enabled that keep it standing upright, as shown in the following image:

[[file:files/Gazebo_with_drc_robot.png|640px]]

Note that in drcsim 4.x.x, the robot starts with a standing pose:

[[file:files/Gazebo_with_drc_robot_drcsim4.png|640px]]

Note that all of the robot's joints, including the legs, are physically simulated and actively controlled.

So the simulated robot in this tutorial can't walk, but we still want to move it around in the world.  Fortunately, the simulated robot accepts velocity commands via ROS to translate and rotate in the plane, as if it were a wheeled robot.

## Moving Atlas

1. Start the simulator:

    ~~~
    VRC_CHEATS_ENABLED=1 roslaunch drcsim_gazebo atlas.launch
    ~~~

    Note: Setting the variable `VRC_CHEATS_ENABLED=1` exposes several development aid topics including `/atlas/cmd_vel`, which are by default disabled for the VRC competition.


    The simulated robot is awaiting [ROS Twist](http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html) messages, which specify 6-D velocities, on the `atlas/cmd_vel` topic.  Check that with [rostopic](http://ros.org/wiki/rostopic):

    ~~~
    rostopic info atlas/cmd_vel
    ~~~

    You should see something like:

    %%%
    Type: geometry_msgs/Twist
    Publishers: None
    Subscribers:
     * /gazebo (http://osrf-Latitude-E6420:35339/)
    %%%

    You can publish ROS Twist messages from anywhere, including from the command line, using rostopic.  First, let's see what's in a Twist message, using [rosmsg](http://ros.org/wiki/rosmsg):

    ~~~
    rosmsg show Twist
    ~~~

    You should see:

    %%%
    [geometry_msgs/Twist]:
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
    %%%

    It's a 6-D velocity: 3 linear velocities (X, Y, and Z) and 3 angular velocities (rotations about X, Y, Z, also called roll, pitch, and yaw). Our robot is constrained to move in the plane, so we only care about X, Y, and yaw (rotation about Z).

1. Place the robot in a *stand* position:

    ~~~
    rostopic pub --once /atlas/mode std_msgs/String "pid_stand"
    ~~~

1. *Pin* the robot for keeping its feet off the ground:

    ~~~
    rostopic pub --once /atlas/mode std_msgs/String "pinned"
    ~~~

1. Make the robot drive counter-clockwise in a circle:

    ~~~
    rostopic pub -r 10 atlas/cmd_vel geometry_msgs/Twist '{ linear: { x: 0.5, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.5 } }'
    ~~~

<iframe src="//player.vimeo.com/video/110497452" width="500" height="370" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe> <p><a href="http://vimeo.com/110497452">Gazebo with drc robot drcsim4</a> from <a href="http://vimeo.com/osrfoundation">OSRF</a> on <a href="https://vimeo.com">Vimeo</a>.</p>

Every `cmd_vel` sent has a lifetime associated. By default, the velocity command is applied to Atlas for 0.1 seconds. After that, the robot will stop. If you look at the previous `rostopic` command that you typed, we included a `-r 10` option argument to publish the same message at 10 Hz., to guarantee that the robot does not stop.

It's also possible to modify the value of the velocity command timeout:

1. By adding a new ROS parameter in the [`atlas.launch`](https://github.com/osrf/drcsim/raw/default/drcsim_gazebo/launch/atlas.launch) file:

    ~~~
    <param name="/atlas/cmd_vel_timeout" type="double" value="0.2"/>
    ~~~

1. By executing [`rosparam`](http://wiki.ros.org/rosparam) before launching drcsim:

    ~~~
    rosparam set /atlas/cmd_vel_timeout 0.2
    ~~~

You can verify the commands being sent with the command:

~~~
rostopic echo atlas/cmd_vel
~~~

To stop the robot, press CTRL-C to cancel the previous command.

From here, you're ready to write code that moves the robot around the world.
