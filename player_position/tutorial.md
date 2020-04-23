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

Copy the config script below into a file called `position2d.cfg`. The following script is also accessible via this [link](https://github.com/osrf/gazebo/raw/master/examples/player/position2d/position2d.cfg).

<include
src='https://github.com/osrf/gazebo/raw/master/examples/player/position2d/position2d.cfg'/>

# Run

Run Gazebo

~~~
gazebo worlds/pioneer2dx.world
~~~

Run Player

~~~
player position2d.cfg
~~~

Run playerv

~~~
playerv
~~~

You can now drive the pioneer2dx using playerv.
