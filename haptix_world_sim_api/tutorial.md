# Overview
This tutorial demonstrates building a custom world using SDF.
It also provides a simple example on how to sense object interactions
and manipulate object color using the simulation world API.

We assume that you have already completed the
[installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix)
and the
[world API](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix)
tutorials.

# Documentation
The full world API documentation can be found
[here](https://s3.amazonaws.com/osrf-distributions/haptix/api/0.7.1/haptix__sim_8h.html).

The documentation for building a Gazebo world using [SDF format](http://www.sdformat.org/) can be found [here](http://gazebosim.org/tutorials?cat=build_world).

# Matlab Example

## Intoduction

First, build a world using SDF:

<include lang='xml' src='https://github.com/osrf/gazebo_tutorials/raw/master/haptix_world_sim_api/files/custom_haptix.world'/>

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

<include lang='matlab' src='https://github.com/osrf/gazebo_tutorials/raw/master/haptix_world_sim_api/files/custom_world.m'/>

## Get the code:

Download tutorial files:

 - [custom_haptix.world](https://github.com/osrf/gazebo_tutorials/blob/haptix_world_sim_api/haptix_world_sim_api/files/custom_haptix.world)

 - [custom_world.m](https://github.com/osrf/gazebo_tutorials/blob/haptix_world_sim_api/haptix_world_sim_api/files/custom_world.m)

# Haptix C-API Example

## Intoduction

Using the same world as the Matlab example above,
and write a simple C code to sense contact state on the visual
sphere model, and change the color of the sphere from green to red when
it comes into contact with other objects:

<include lang='c' src='https://github.com/osrf/gazebo_tutorials/raw/master/haptix_world_sim_api/files/custom_world.c'/>

Build `custom_world.c` as you would with any Haptix C API code as shown in [Compile section under the Haptix C API tutorial](http://gazebosim.org/tutorials?tut=haptix_comm&cat=haptix#Compileyourcontroller).

## Get the code:

Download tutorial files:

 - [custom_haptix.world](https://github.com/osrf/gazebo_tutorials/blob/haptix_world_sim_api/haptix_world_sim_api/files/custom_haptix.world)

 - [custom_world.c](https://github.com/osrf/gazebo_tutorials/blob/haptix_world_sim_api/haptix_world_sim_api/files/custom_world.c)

# Start Gazebo handsim simulation

To run the example, start gazebo in terminal:

~~~
gazebo custom_haptix.world
~~~

## Run the code: MATLAB, Octave on Linux

First see [world API tutorial Example section](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix#Example) on how to run Matlab or Octave scripts.

Next, invoke the `custom_world.m` script in Matlab or Octave command prompt
and the sphere should change color from green to red as the hand passes through it:

Hint for linux users, at octave or matlab prompt, add path to haptix scripts:

~~~
path('[replace with your path to install]/lib/x86_64-linux-gnu/haptix-comm/octave', path)
path('[replace with path to your custom_world.m]', path)
~~~

before running `custom_world.m`.

## Run the code: Using C-API on Linux

Run compiled binary from `custom_world.c` as you would with any Haptix C API code as shown in [Running the simulation section under the Haptix C API tutorial](http://gazebosim.org/tutorials?tut=haptix_comm&cat=haptix#Runningthesimulationwithyourcontroller).

## Example Video
<iframe width="600" height="450" src="https://www.youtube.com/embed/bWaWNZu-0n4" frameborder="0" allowfullscreen></iframe>
