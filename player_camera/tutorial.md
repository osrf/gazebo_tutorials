# Introduction

[Player](http://playerstage.sourceforge.net) is a robot control framework,
please visit their [website](http://playerstage.sourceforge.net) for more
information.

This tutorial covers connecting the camera Player interfaces to Gazebo. 

Create a working directory

~~~
cd; mkdir gazebo_camera; cd gazebo_camera
~~~

# Player Config

Copy the config script below into a file called `camera.cfg`. The following script is also accessible via this [link](https://github.com/osrf/gazebo/raw/default/examples/player/camera/camera.cfg).

<include
src='https://github.com/osrf/gazebo/raw/default/examples/player/camera/camera.cfg'/>

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
