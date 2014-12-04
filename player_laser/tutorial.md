# Introduction

[Player](http://playerstage.sourceforge.net) is a robot control framework,
please visit their [website](http://playerstage.sourceforge.net) for more
infomartion.

This tutorial covers connecting the laser Player interfaces to Gazebo. 

Create a working directory

~~~
cd; mkdir gazebo_laser; cd gazebo_laser
~~~

# Player Config

Copy the config script below into a file called `laser.cfg`

~~~
driver
(
  name "gazebo"
  provides ["simulation:0"]
  plugin "libgazebo_player"

  # The name of a running Gazebo world, specified in a .world file
  world_name "default"
)

driver
(
  name "gazebo"
  provides ["laser:0"]

  # The fully scoped laser sensor name.
  laser_name "pioneer2dx::hokuyo::link::laser"
)
~~~

# Run

Run Gazebo

~~~
gazebo worlds/pioneer2dx_laser.world
~~~

Run Player

~~~
player laser.cfg
~~~

Run playerv

~~~
playerv
~~~

You can now visualize the laser inside playerv.
