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


Next, write a simple Octave/Matlab to sense contact state on the visual
sphere model, and change the color of the sphere from green to red when
it comes into contact with other objects:

## Start Gazebo handsim simulation

## Run the code: MATLAB, Octave on Linux

First see [Sim API tutorial Example section](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix#Example) on how to run Matlab or Octave scripts.

Download tutorial files:

 - [custom_haptix.world](https://bitbucket.org/osrf/gazebo_tutorials/raw/world_sim_api/world_sim_api/files/custom_haptix.world)

    <include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/world_sim_api/world_sim_api/files/custom_haptix.world'/> 

 - [custom_world.m](https://bitbucket.org/osrf/gazebo_tutorials/raw/world_sim_api/world_sim_api/files/custom_world.m)

    <include lang='matlab' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/world_sim_api/world_sim_api/files/custom_world.m'/> 

To run the example, start gazebo in terminal:

~~~
gazebo custom_haptix.world
~~~

Next, invoke the `custom_world.m` script in Matlab or Octave command prompt
and the sphere should change color from green to red as the hand passes through it:

<iframe width="600" height="450" src="https://www.youtube.com/embed/0R_xmgG_jBI" frameborder="0" allowfullscreen></iframe>

