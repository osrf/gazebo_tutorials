# Introduction

[Player](http://playerstage.sourceforge.net] is a robot control framework,
please visit their [webiste](http://playerstage.sourceforge.net) for more
infomartion.

This tutorial covers connecting the position2d, laser, and camera Player interfaces to Gazebo. 

Create a working directory

~~~
cd; mkdir gazebo_position2d; cd gazebo_position2d
~~~

# World File

Copy the SDF code below into a file called `position.world`

~~~
<?xml version="1.0"?>
<gazebo version="1.0">
  <world name="default">
    <!-- Ground -->
    <include filename="ground_plane.model"/>

    <!-- Pioneer2dx model -->
    <include filename="pioneer2dx.model"/>

    <!-- A global light source -->
    <include filename="sun.light"/>
  </world>
</gazebo>
~~~

# Player Config

Copy the config script below into a file called `position.cfg`

~~~
driver
(
  name "gazebo"
  provides ["simulation:0"]
  plugin "libgazebo_player"

  # The name of a runnign Gazebo world, specified in a .world file
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

Run the Gazebo server

<pre>
gzserver position.world
</pre>

Run Gazebo GUI

<pre>
gzclient
</pre>

Run Player

<pre>
player position.cfg
</pre>

Run playerv

<pre>
playerv
</pre>

You can now drive the pioneer2dx using playerv.
