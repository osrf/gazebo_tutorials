# Overview

**Prerequisites:** [Quick Start](http://gazebosim.org/tutorials/?tut=quick_start)

This tutorial demonstrates how the user can create a population of models by
using the `<population>` tag.

Adding a model population, is a matter of specifying the following parameters:

1. Model. E.g.: table, coke_can.

1. Number of objects to be part of the population.

1. Region in which the object will be located. E.g.: box, cylinder.

1. Distribution used to locate the objects within the region. E.g.: random, grid.

For reference, check the [SDF API](http://gazebosim.org/sdf/dev.html) for a complete specification of the `<population` tag and its parameters.

# Creating an object population.

1. Let's start by creating a directory for this tutorial:

      mkdir ~/tutorial_model_population
      cd ~/tutorial_model_population

1. Copy the following into `can_population.world`:

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

The above code is also located [here](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world).

1. Start gazebo:

      gazebo can_population.world

You should see a population of soda cans randomly located around the origin.

Let's go further and understand what are the different elements of the `can_population.world`.

<include from='/population name/' to='</model>' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

In this snippet we can see how we can specify a population element by using the `<population> tag. Every population should have a unique name, and this is specified by the `name` attribute. Within the `population` tag, you can see how to select a model by using the `<model>` tag. Each element of the population will be inserted into the simulation with a unique name that will be created by appending to the model name the suffix `_clone_i`, where `i` is the ith element of the population. You can see the list of models spawned in the Gazebo scene here:

<img src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/model_list.png" width="640px"/>

The most common type of population to create consists of inanimate objects
such as trees, rocks, and buildings. We recommend to use the `<population` tag for static models, and exclude mobile entities, such as robots, which often require more precise placement and are fewer in number.

<include from='/<region>/' to='</region>' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

In this section of the population tag you can see how the `<region>` element is used. This element specifies the region in which the objects will be placed. In this case, all the objects are spawned within a 3D bounding box where two vertices `<min>` and `<max>` are defined in world coordinates. As an alternative to `<box>`, a `<cylinder>` region is also allowed by specifying the center of the base, its radius, and height. Check out the [SDF specification]((http://gazebosim.org/sdf/dev.html)) for a full description of the `<cylinder>` parameters.

<include from='/<model_count>/' to='</model_count>' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

Above you can see how the number of models in the population is determined. Any positive number is allowed, but take into consideration that the higher the number, the more impact in performance you may have.

<include from='/<distribution>/' to='</distribution>' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/model_population/files/can_population.world' />

Finally, the `<distribution>` element sets how the objects are placed within the region. All the distribution types are described as follow:

1. random: Models placed at random. Note that the objects might collide between each other.

1. uniform: Models placed in a pseudo evenly distribution within the region.

1. grid: Models evenly placed in a 2D grid pattern. This distribution also requires to specify the number of rows, columns, and distance between each element. Note that the element `<model_count>` is ignored in this distribution. The number of objects inserted into the simulation will be equal to the value of rows multiplied by the value of columns.

1. linear-x: Models evenly placed in a row along the global x-axis.

1. linear-y: Models evenly placed in a row along the global y-axis.

1. linear-z: Models evenly placed in a row along the global z-axis.

For a more advanced example you can check the [population.world](http://bitbucket.org/osrf/gazebo/raw/default/worlds/population.world) world file deployed with Gazebo.

And of course, you can test it by typing:

        gazebo population.world
