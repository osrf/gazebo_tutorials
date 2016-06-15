# Overview

This tutorial explains how to use Gazebo's "actors" to create scripted animations.

Animations are useful if you want to have entities moving around in simulation
following scripted motions without being affected by the physics engine.
This means they won't fall due to gravity or collide with other objects, for
example. They will, however, have a 3D visualization which can be seen by RGB
cameras and 3D meshes which can be detected by GPU based depth sensors.

> Tip: Physics-engine-based sensors would require collsion information, read
more about sensors [here](is there any material on this?).

# Actors

In Gazebo, an animated model is called an `actor`. Actors extend models adding
animation capabilities.

There are two types of animations which can be used separately or combined
together:

* Motion along a trajectory, defined by a series of keyframes with time and
pose. This is a high level motion which carries all the actor's links as one
group.

    [[file:files/traj_full.gif|300px]]

* Skeleton motion, which is relative motion between links in one model. These
can be described in DAE and BVH formats.

    [[file:files/skel_full.gif|300px]]

* And both motions can be combined and used together:

    [[file:files/skel_traj_full.gif|300px]]

# Scripted motion

This is the high level animation of actors, which consists of specifying
poses which should be reached at specific times, and Gazebo takes care of
interpolating the motion between them.

Let's take a look at an example and then explain how it works.
Let's create a world which has an animated box in it.
try to script a simple motion for a box. Create a world file and copy
the following contents into it:

    gedit animated_box.world

    <?xml version="1.0" ?>
    <sdf version="1.6">
       <world name="default">
          <include>
             <uri>model://ground_plane</uri>
          </include>
          <!-- A global light source -->
          <include>
             <uri>model://sun</uri>
          </include>
          <actor name="actor">
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>.2 .2 .2</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>.2 .2 .2</size>
                  </box>
                </geometry>
              </visual>
            </link>
            <script>
              <loop>true</loop>
              <delay_start>0.000000</delay_start>
              <auto_start>true</auto_start>
              <trajectory id="0" type="walking">
                 <waypoint>
                    <time>0.0</time>
                    <pose>-1 -1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>0.5</time>
                    <pose>-1 1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>1.0</time>
                    <pose>1 1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>1.5</time>
                    <pose>1 -1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>2.0</time>
                    <pose>-1 -1 1 0 0 0</pose>
                 </waypoint>
              </trajectory>
            </script>
          </actor>
       </world>
    </sdf>


Let's take a look at what it does, and then explain how it works.

    gazebo animated_box.world

So what's happening?






