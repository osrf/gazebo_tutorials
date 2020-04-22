# Introduction

[Player](http://playerstage.sourceforge.net) is a robot control framework,
please visit their [website](http://playerstage.sourceforge.net) for more
information.

This tutorial covers connecting the laser Player interfaces to Gazebo.

Create a working directory

~~~
cd; mkdir gazebo_laser; cd gazebo_laser
~~~

# Player Config

Copy the config script below into a file called `laser.cfg`. The following script is also accessible via this [link](https://github.com/osrf/gazebo/raw/master/examples/player/laser/laser.cfg).

<include
src='https://github.com/osrf/gazebo/raw/master/examples/player/laser/laser.cfg'/>

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
