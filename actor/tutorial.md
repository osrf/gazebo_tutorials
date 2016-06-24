# Overview

This tutorial explains how to use Gazebo's "actors" to create scripted animations.

Animations are useful if you want to have entities following predefined paths
in simulation without being affected by the physics engine. This means they won't
fall due to gravity or collide with other objects, for example. They will,
however, have a 3D visualization which can be seen by RGB cameras, and 3D meshes
which can be detected by GPU based depth sensors.

The tutorial explains in detail how to create open-loop trajectories which don't
interact with the rest of the simulation. Towards the end, we will take a quick
look at an example plugin which controls animations based on feedback from the
environment.

# Actors

In Gazebo, an animated model is called an `actor`. Actors extend
[common models](http://gazebosim.org/tutorials?tut=build_model&cat=build_robot),
adding animation capabilities.

There are two types of animations which can be used separately or combined
together:

* **Skeleton** animation, which is relative motion between links in one model:

    [[file:files/skel_full.gif|300px]]

* Motion along a **trajectory**, which carries all of the actor's links around
the world, as one group:

    [[file:files/traj_full.gif|300px]]

* Both types of motions can be **combined** to achieve a skeleton animation which
moves in the world:

    [[file:files/skel_traj_full.gif|300px]]

Gazebo's actors are just like
[models](http://gazebosim.org/tutorials?tut=build_model),
so you can put links and joints inside them as usual. The main differences are:

* Actors are always static (i.e. no forces are applied on them, be it from
gravity or contact or anything else)

* Actors support skeleton animation imported from COLLADA and BVH files.

* Actors can have trajectories scripted directly in SDF.

* There can't be models nested inside actors, so we're limited to animated
meshes, links and joints.

> **Tip**: Check out the full specification for the `<actor>` SDF element
[here](http://sdformat.org/spec?ver=1.6&elem=actor).

# Scripted trajectories

This is the high level animation of actors, which consists of specifying
a series of poses to be reached at specific times. Gazebo takes care of
interpolating the motion between them so the movement is fluid.

## Example world

Let's take a look at a simple example world which comes with Gazebo:

    gazebo worlds/animated_box.world

You'll see a floating box moving in a square trajectory again and again. The
trajectory goes through four points in the world (`[-1, -1, 1]`, `[-1, 1, 1]`,
`[1, 1, 1]` and `[1, -1, 1]`) and takes 1 s in between them.

[[file:files/box_square_full.gif|300px]]

## World explained

You can see the whole world description
[here](http://bitbucket.org/osrf/gazebo/raw/actor_2/worlds/animated_box.world).
Let's go through it by parts.

We start by defining a world with a ground plane and a sun.

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

We create an actor called `animated_box` and give it a simple link with a box
visual:

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

Now comes the part which is special to actors, the `<script>` tag. We begin by
telling it to loop forever and start playing as soon as the world is loaded.

            <script>
              <loop>true</loop>
              <delay_start>0.000000</delay_start>
              <auto_start>true</auto_start>

The following parameters are available:

* **`loop`**: Set this to true for the script to be repeated in a loop. For a
fluid continuous motion, make sure the last waypoint matches the first one,
as we'll see below.

* **`delay_start`**: This is the time in seconds to wait before starting the
script. If running in a loop, this time will be waited before starting each
cycle.

* **`auto_start`**: Set to true if the animation should start as soon as the
simulation starts playing. It is useful to set this to false if the animation
should only start playing only when triggered by a plugin, for example.

Finally, we define a trajectory with a sequence of waypoints:

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

Within the `<trajectory>` tag, it's possible to describe a series of keyframes
to be followed. It has two attributes: a unique `id` and a `type`. The type will
be useful when we explain skeleton animations in the next section. The
trajectory parameters are as follows:

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

Gazebo supports two different skeleton animation file formats:
[COLLADA (.dae)](https://www.khronos.org/collada/wiki/Main_page) and
[Biovision Hierarchy (.bvh)](http://research.cs.wisc.edu/graphics/Courses/cs-838-1999/Jeff/BVH.html).

Let's try a simple example file that comes with Gazebo. First, create a
new world file:

    gedit walk.world

And paste the following SDF, it has a sun and an actor using the
[walk.dae](https://bitbucket.org/osrf/gazebo/src/default/media/models/walk.dae)
as the skin:

    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="default">
        <include>
          <uri>model://sun</uri>
        </include>
        <actor name="actor">
          <skin>
            <filename>walk.dae</filename>
          </skin>
        </actor>
      </world>
    </sdf>

Take a look at it in Gazebo, and you'll see a person walking in place.

    gazebo walk.world

## Skin

The actor in the example above is really simple, all it loads is a COLLADA file
described within the `<skin>` tag.

> **Note**: If you've made
[custom](http://gazebosim.org/tutorials?tut=import_mesh&cat=build_robot)
Gazebo models before, you might have used COLLADA files as visuals and
collisions for your models. When used within *links*, COLLADA animations are
ignored, but when used within *skins*, they are loaded!

The file specified in `<filename>` can be an absolute path, for example:

    /home/<user>/my_gazebo_models/skeleton_model/skeleton.dae

You can also tell Gazebo to look for the mesh in all the directories contained in
the environment variable
[`GAZEBO_MODEL_PATH`](http://gazebosim.org/tutorials?tut=components),
like this:

    model://skeketon_model/skeleton.dae

Finally, you can use a few example meshes which are installed with Gazebo by
referencing directly to their filenames. Below is the list of the ones
available. Take a look at some of them substituting into the world above!

* `moonwalk.dae`
* `run.dae`
* `sit_down.dae`
* `sitting.dae`
* `stand_up.dae`
* `stand.dae`
* `talk_a.dae`
* `talk_b.dae`
* `walk.dae`

## Animation

### Combining different skins ans animations

Sometimes, it is useful to combine different skins with different animations.
Gazebo allows us to take the skin from one file, and the animation from
another file, as long as they have compatible skeletons.

For example, the files `walk.dae` and `moonwalk.dae` are compatible so they can
be mixed with each other. The person walking has a green shirt, and the person
moonwalking has a red shirt.

* If you want a person *moonwalking* with a *green* shirt, use `walk` for the skin
and `moonwalk` for the animation.

* If you want a person *walking* with a *red* shirt, use `moonwalk` for the skin
and `walk` for the animation.

The animation tag goes alongside the skin tag, and it takes a `name` parameter.
Like this:

    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="default">
        <include>
          <uri>model://sun</uri>
        </include>
        <actor name="actor">
          <skin>
            <filename>walk.dae</filename>
          </skin>
          <animation name="animation">
            <filename>moonwalk.dae</filename>
          </animation>
        </actor>
      </world>
    </sdf>

Try out different combinations now!

### Sync animation and trajectory

By this point, you already know everything about creating trajectories and
loading static animations. It's time to learn how to combine them.

You might be thinking "I'll just add `<skin>`, `<animation>` and `<trajectory>`
tags to my actor and they will work together". I'm not going to stop you, go ahead
and try that, I'll even give you an example:

    <sdf version="1.6">
      <world name="default">
        <include>
          <uri>model://sun</uri>
        </include>
        <actor name="actor">
          <skin>
            <filename>walk.dae</filename>
          </skin>
          <animation name="animation">
            <filename>walk.dae</filename>
          </animation>
          <script>
            <trajectory id="0" type="walking">
              <waypoint>
                <time>0</time>
                <pose>0 2 0 0 0 -1.57</pose>
              </waypoint>
              <waypoint>
                <time>2</time>
                <pose>0 -2 0 0 0 -1.57</pose>
              </waypoint>
              <waypoint>
                <time>2.5</time>
                <pose>0 -2 0 0 0 1.57</pose>
              </waypoint>
              <waypoint>
                <time>7</time>
                <pose>0 2 0 0 0 1.57</pose>
              </waypoint>
              <waypoint>
                <time>7.5</time>
                <pose>0 2 0 0 0 -1.57</pose>
              </waypoint>
            </trajectory>
          </script>
        </actor>
      </world>
    </sdf>

Go ahead and load it and see what happens. That's not what yopu expected,
right? The actor's legs are not moving at all. That's because Gazebo doesn't
know which animation to match with which trajectory. So let's change the
animation name to match the trajectory type like this:

          <animation name="walking">
            <filename>walk.dae</filename>
          </animation>

Ok, so the actor was both moving back and forth in the world, and moving his
legs. But that didn't look very natural, right? His feet were sliding on the
ground.

Let's tell Gazebo to synchronize the distance travelled by the trajectory, with
the animation, by setting `<interpolate_x>` to true inside `<animation>`:

          <animation name="walking">
            <filename>walk.dae</filename>
            <interpolate_x>true</interpolate_x>
          </animation>

Now you finally have the two animations playing in perfect sync. You should be
seeing the person walking from one side to the other, faster in one direction,
and slower the other way.

[[file:files/full_animation.gif|300px]]

Try playing with the times and the distances in the waypoint, and you'll see
that the actor's legs will move faster or slower depending on the trajectory.

# Closed-loop trajectories

You just learned how to create actors and set their trajectories through SDF.
The limitation to that is that the trajectory is running in an open-loop,
that is, it is not taking any feedback from the environment. Now we're going to
take a look at an example of how to change the trajectory dinamically using
plugins.

> **Tip**: If you're not familiar with Gazebo plugins, take a look at some
[plugin tutorials](http://gazebosim.org/tutorials?cat=write_plugin) first.

Gazebo has an example world with actors moving around while avoiding obstacles.
Take a look at it running:

    gazebo worlds/cafe.world

Here's what it looks like:

<iframe width="560" height="315" src="https://www.youtube.com/embed/nZN07_5r568" frameborder="0" allowfullscreen></iframe>

## Plugin in SDF

Just like for models, it's possible to write custom plugins for any actor and
assign the plugin in the SDF description. Let's take a look at the part of
[cafe.world](https://bitbucket.org/osrf/gazebo/raw/default/worlds/cafe.world)
which refers to one of the actors in the video:

    <actor name="actor1">
      <pose>0 1 1.25 0 0 0</pose>
      <skin>
        <filename>moonwalk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.000000</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <plugin name="actor1_plugin" filename="libActorPlugin.so">
        <target>0 -5 1.2138</target>
        <target_weight>1.15</target_weight>
        <obstacle_weight>1.8</obstacle_weight>
        <animation_factor>5.1</animation_factor>
        <ignore_obstacles>
          <model>cafe</model>
          <model>ground_plane</model>
        </ignore_obstacles>
      </plugin>
    </actor>

We can see that instead of giving a list of specific waypoints to follow, a
plugin was given. Inside the plugin tag, there are several parameters which
can be tuned specifically for this plugin. We won't get into the details on how
the plugin works, the intent here is to show that a few parameters can be
exposed, and the logic to determine the trajectory will be inside the plugin.

## Plugin C++ code

The source code for the `ActorPlugin` can be found
[here](https://bitbucket.org/osrf/gazebo/raw/default/plugins/ActorPlugin.cc).
And
[here](https://bitbucket.org/osrf/gazebo/raw/default/plugins/ActorPlugin.hh)
is the header.

The first trick is to listen to world update begin events like this:

<include from='/  this->connections/' to='/_1)));/' src='https://bitbucket.org/osrf/gazebo/raw/default/plugins/ActorPlugin.cc' />


    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
        std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

This way, we specify a callback `ActorPlugin::OnUpdate`, which will be called
at every world iteration. This is the function where we will update our actor's
trajectory. Let's see what the plugin is doing in that function:



    void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
    {
      // Time delta
      double dt = (_info.simTime - this->lastUpdate).Double();

      ignition::math::Pose3d pose = this->actor->GetWorldPose().Ign();
      ignition::math::Vector3d pos = this->target - pose.Pos();
      ignition::math::Vector3d rpy = pose.Rot().Euler();

      double distance = pos.Length();

      // Choose a new target position if the actor has reached its current
      // target.
      if (distance < 0.3)
      {
        this->ChooseNewTarget();
        pos = this->target - pose.Pos();
      }

It starts by checking the current information, like the time and actor pose.
If it's already reached the target destination, we pick a new one.

      // Normalize the direction vector, and apply the target weight
      pos = pos.Normalize() * this->targetWeight;

      // Adjust the direction vector by avoiding obstacles
      this->HandleObstacles(pos);

      // Compute the yaw orientation
      ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
      yaw.Normalize();

      // Rotate in place, instead of jumping.
      if (std::abs(yaw.Radian()) > GZ_DTOR(10))
      {
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
            yaw.Radian()*0.001);
      }
      else
      {
        pose.Pos() += pos * this->velocity * dt;
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
      }

      // Make sure the actor stays within bounds
      pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
      pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
      pose.Pos().Z(1.2138);

Then it goes on calculating the target pose taking into account obstacles and
making sure we have a smooth motion. The following steps are the most
important, because they involve actor-specific API.

      // Distance traveled is used to coordinate motion with the walking
      // animation
      double distanceTraveled = (pose.Pos() -
          this->actor->GetWorldPose().Ign().Pos()).Length();

      this->actor->SetWorldPose(pose, false, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() +
        (distanceTraveled * this->animationFactor));
      this->lastUpdate = _info.simTime;
    }

We first set the actor's world pose as if it was a static model, with `SetWorldPose`.
This will not trigger the animation however. That's done by telling the actor which
point of its skeleton animation it should be in, with `SetScriptTime`.

