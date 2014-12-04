# Introduction

[Player](http://playerstage.sourceforge.net) is a robot control framework,
please visit their [website](http://playerstage.sourceforge.net) for more
infomation.

This tutorial covers connecting the camera Player interfaces to Gazebo. 

Create a working directory

~~~
cd; mkdir gazebo_camera; cd gazebo_camera
~~~

# Player Config

Copy the config script below into a file called `camera.cfg`

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
  provides ["camera:0"]

  # The fully scoped camera sensor name.
  camera_name "pioneer2dx::camera::link::camera"
)
~~~

# Run

Run Gazebo

~~~
gazebo worlds/pioneer2dx_camera.world
~~~

Run Player

~~~
player camera.cfg
~~~

Run playerv

~~~
playerv
~~~

You can now visualize the camera inside playerv.
