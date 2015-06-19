# Overview
This tutorial demonstrates building a custom world using SDF.
And provide a simple example on how to sense object interaction
and manipulate object color using the world simulation API.

We assume that you have already completed the
[installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix)
and the
[Sim API](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix)
tutorials.

# Documentation
The full sim API documentation can be found
[here](https://s3.amazonaws.com/osrf-distributions/haptix/api/0.7.1/haptix__sim_8h.html).

Using the [SDF format](http://www.sdformat.org/),
the documentation for building a gazebo world can be found
[here](http://gazebosim.org/tutorials?cat=build_world).

# Example

First, build a world using SDF:

<include lang='xml' src='https://bitbucket.org/osrf/gazebo_tutorials/raw/c1710212bfcc1a11594a9280d4a7db01c3e5de15/haptix_world_sim_api/files/custom_haptix.world'/> 

Note that this world file is very similar to the default [arat.world](https://bitbucket.org/osrf/handsim/src/62b1deba4ab2f82b7910beb959042212c3c9bfae/worlds/arat.world?at=default), with the main difference being the addition of the `sphere_visual_model`:

~~~
    <model name="sphere_visual_model">
      <pose>0.4 0.0 1.2 0 0 3.1416</pose>
      <link name="sphere_link">
        <visual name="sphere_visual">
          <geometry>
            <sphere>
              <radius>0.03</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Green</name>
            </script>
          </material>
        </visual>
        <collision name="sphere_visual">
          <geometry>
            <sphere>
              <radius>0.03</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
      <static>1</static>
    </model>
~~~

Next, write a simple Octave/Matlab to sense contact state on the visual
sphere model, and change the color of the sphere from green to red when
it comes into contact with other objects:

<include lang='matlab' src='https://bitbucket.org/osrf/gazebo_tutorials/raw/2d9132fe45f8b938ad94c3db871f1109db4bd40f/haptix_world_sim_api/files/custom_world.m'/> 

## Start Gazebo handsim simulation

## Run the code: MATLAB, Octave on Linux

First see [Sim API tutorial Example section](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix#Example) on how to run Matlab or Octave scripts.

Download tutorial files:

 - [custom_haptix.world](https://bitbucket.org/osrf/gazebo_tutorials/src/haptix_world_sim_api/haptix_world_sim_api/files/custom_haptix.world)

 - [custom_world.m](https://bitbucket.org/osrf/gazebo_tutorials/src/haptix_world_sim_api/haptix_world_sim_api/files/custom_world.m)

To run the example, start gazebo in terminal:

~~~
gazebo custom_haptix.world
~~~

Next, invoke the `custom_world.m` script in Matlab or Octave command prompt
and the sphere should change color from green to red as the hand passes through it:

<iframe width="600" height="450" src="https://www.youtube.com/embed/0R_xmgG_jBI" frameborder="0" allowfullscreen></iframe>

