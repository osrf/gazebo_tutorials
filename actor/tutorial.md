# Overview

This tutorial explains how to use Gazebo's "actors" to create scripted animations.

Animations are useful if you want to have entities moving around in simulation
following scripted motions without being affected by the physics engine.
This means they won't fall due to gravity or collide with other objects, for
example. They will, however, have a 3D visualization which can be seen by RGB
cameras and 3D meshes which can be detected by GPU based depth sensors.

> **Tip**: Physics-engine-based sensors would require collsion information, read
more about sensors [here](is there any material on this?).

# Actors in Gazebo

In Gazebo, an animated model is called an `actor`. Actors extend models adding
animation capabilities.

There are two types of animations which can be used separately or combined
together:

* Motion along a **trajectory**, defined by a series of keyframes. This is a
high level motion which carries all of the actor's links around the world,
as one group:

    [[file:files/traj_full.gif|300px]]

* **Skeleton** motion, which is relative motion between links in one model:

    [[file:files/skel_full.gif|300px]]

* Both types of motions can be **combined** to achieve an animation which moves
in the world:

    [[file:files/skel_traj_full.gif|300px]]

# Actors

Gazebo's actors are just like
[models](http://gazebosim.org/tutorials?tut=build_model),
so you can put links and joints inside it as usual. The main differences are:

* Actors are always static (i.e. no forces are applied on them, be it from
gravity or contact)

* Actors can have their motions scripted directly in the SDF, with support for
different skeleton descriptions and open loop trajectories.

* There can't be models nested inside it, so we're limited to links and joints.

You can see the full specification for the actor element in SDF in this
[link](http://sdformat.org/spec?ver=1.6&elem=actor),
we will explain some of them below.

# Scripted trajectories

This is the high level animation of actors, which consists of specifying
a series of poses to be reached at specific times. Gazebo takes care of
interpolating the motion between them so the movement is fluid.

Let's take a look at a simple example by creating a world which has an
animated box in it. First create a world file:

    gedit animated_box.world

Then copy the following SDF description into it and save it. The code will be
explained in detail soon.

    <?xml version="1.0" ?>
    <sdf version="1.6">
       <world name="default">
          <!-- A ground plane -->
          <include>
             <uri>model://ground_plane</uri>
          </include>
          <!-- A global light source -->
          <include>
             <uri>model://sun</uri>
          </include>
          <!-- An actor -->
          <actor name="animated_box">
            <link name="link">
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
              <trajectory id="0" type="square">
                 <waypoint>
                    <time>0.0</time>
                    <pose>-1 -1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>1.0</time>
                    <pose>-1 1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>2.0</time>
                    <pose>1 1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>3.0</time>
                    <pose>1 -1 1 0 0 0</pose>
                 </waypoint>
                 <waypoint>
                    <time>4.0</time>
                    <pose>-1 -1 1 0 0 0</pose>
                 </waypoint>
              </trajectory>
            </script>
          </actor>
       </world>
    </sdf>


Let's open it in Gazebo to see what it does:

    gazebo animated_box.world

You'll see a floating box moving in a square trajectory again and again. The
trajectory goes through four points in the world (`[-1, -1, 1]`, `[-1, 1, 1]`,
`[1, 1, 1]` and `[1, -1, 1]`) and takes 1 s in between them.

    [[file:files/box_square_full.gif|300px]]

## Script

The actor in the example has a simple link with a box visual. The interesting
part here is the `<script>` tag, used to script global trajectories. The
parameters available are the following:

* **`loop`**: Set this to true for the script to be repeated in a loop. For a
fluid continuous motion, make sure the last waypoint matches the first one,
like in the previous example.

* **`delay_start`**: This is the time to wait before starting the script. If
running in a loop, this time will be waited before starting each cycle.

* **`auto_start`**: Set to true if the animation should start as soon as the
simulation starts playing. It is useful to set this to false if the animation
should only start playing only when triggered by a plugin, for example.

* **`trajectory`**: Within this tag, it's possible to describe a series of
keyframes to be followed. It has two attributes: a unique `id` and a `type`. The
type will be useful when we explain skeleton animations below.

    * **`waypoint`**: There can be any number of waypoints in a trajectory. Each
      waypoint consists of a `time` and a `pose`:

        * **`time`**: The time in seconds, counted from the beginning of the
          script, when the pose should be reached.

        * **`pose`**: The pose which should be reached

> **Tip**: The order in which waypoints are defined is not important, they will
follow the given times.

> **Note**: The trajectory is smoothed as a whole. This means that you'll get a
fluid motion, but the exact poses contained in the waypoints might not be reached.

> **Tip**: Non-actor models can also follow scripted trajectories, but that
requires the use of plugins. See
[this](http://gazebosim.org/tutorials?tut=animated_box)
tutorial to learn how.

Now it's your turn! Try out different trejctory descriptions before moving on to
the next section!

# Skeleton

These can be described in DAE and BVH formats.
