# Introduction

[Player](http://playerstage.sourceforge.net) is a robot control framework,
please visit their [website](http://playerstage.sourceforge.net) for more
information.

This tutorial covers connecting the position2d Player interfaces to Gazebo. 

Create a working directory

~~~
cd; mkdir gazebo_position2d; cd gazebo_position2d
~~~

# Player Config

Copy the config script below into a file called `position.cfg`

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
  provides ["position2d:0"]

  # This name must match the name of a model in the "default" world
  model_name "pioneer2dx"
)
~~~

# Run

Run Gazebo

~~~
gazebo worlds/pioneer2dx.world
~~~

Run Player

~~~
player position.cfg
~~~

Run playerv

~~~
playerv
~~~

You can now drive the pioneer2dx using playerv.
