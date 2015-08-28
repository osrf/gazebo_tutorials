# Overview

This tutorial demonstrates how you can create a population of models by
using the SDF `<population>` tag. A population consists of a collection
of identical models.

Adding a population of models is a matter of specifying the following
parameters:

1. Model (e.g.: `table`, `coke_can`).

1. Number of objects to be part of the population.

1. Shape and dimensions of the container within which the objects will be
arranged (e.g.: `box`, `cylinder`).

1. The position and orientation of the population's container.

1. Distribution of the objects within the container (e.g.: `random`, `grid`).

For reference, check the
[SDF specification](http://sdformat.org/spec?ver=1.5&elem=world#world_population)
 for a complete specification of the `<population>` tag and its parameters.

# Creating an object population

## Quick start

1. Let's start by creating a directory for this tutorial:

    ~~~
    mkdir ~/tutorial_model_population
    cd ~/tutorial_model_population
    ~~~

1. Download this file:
[`can_population.world`](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world)
into the current directory. You can use this command:

    ~~~
    wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world
    ~~~

    You should get this world file:

    <include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

1. Start gazebo:

    ~~~
    gazebo can_population.world
    ~~~

    You should see a population of soda cans randomly located around the
    world's origin. The cans are arranged within a box container of size
    2 x 2 x 0.01 meters.

## The world explained

Let's go further and understand the different elements of the `can_population.world`.

<include from='/    <population name/' to='/</model>/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

In this snippet we can see how to specify a population element by using the
`<population>` tag. Every population should have a unique name, and this is
specified by the `name` attribute. Within the `population` tag, you can see
how to select a model by using the `<model>` tag. Each element of the population
will be inserted into the simulation with a unique name that will be created by
appending to the model name the suffix `_clone_i`, where `i` is the ith element
of the population. You can see the list of models spawned in the Gazebo scene
here:

[[file:files/model_list_6-0.png|640px]]

The most common type of population consists of inanimate objects
such as trees, rocks, and buildings. We recommend you use the `<population>` tag
for static models, and exclude mobile entities, such as robots, which often
require more precise placement and are fewer in number.

<include from='/      <pose>/' to='/<\/box>/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

The above block of code specifies the region in which the objects will be
placed. In this case, all the objects are spawned within a 3D bounding box with
sides `2 x 2 x 0.01 m`, centered at (0, 0, 0) with orientation (0, 0, 0). As an
alternative to `<box>`, a `<cylinder>` region is also allowed by specifying its
radius and length. (Check out the
[SDF specification](http://sdformat.org/spec?ver=1.5&elem=world#population_cylinder)
for a full description of the `<cylinder>` parameters.) The `<pose>` element
sets the reference frame of the population's region.

<include from='/      <model_count>/' to='/</model_count>/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

Above you can see how the number of models in the population is determined. Any
positive number is allowed, but take into consideration that the higher the
number, the more impacted the performance may be.

<include from='/      <distribution>/' to='/</distribution>/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

The `<distribution>` element sets how the objects are placed within the region.

## Distribution types

1. `random`: Models placed at random. Note that the objects might collide with
  one another.

1. `uniform`: Models placed in a pseudo-2D grid pattern. We use K-Means to
  approximate the solution and locate the number of specified objects inside the
  region.

1. `grid`: Models evenly placed in a 2D grid pattern. This distribution also
  requires that you specify the number of rows, columns, and distance between
  each element. Note that the element `<model_count>` is ignored in this
  distribution. The number of objects inserted into the simulation will be equal
  to the number of rows multiplied by the number of columns.

1. `linear-x`: Models evenly placed in a row along the global x-axis.

1. `linear-y`: Models evenly placed in a row along the global y-axis.

1. `linear-z`: Models evenly placed in a row along the global z-axis.

For a more advanced example you can check the
[population.world](http://bitbucket.org/osrf/gazebo/raw/gazebo5/worlds/population.world)
world file deployed with Gazebo.

And of course, you can test it by typing:

~~~
gazebo worlds/population.world
~~~

[[file:files/gazebo_population.png|640px]]
