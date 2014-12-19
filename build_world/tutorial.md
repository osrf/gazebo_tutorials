This tutorial describes the process of creating a world with both static and dynamic objects.

# Terminology

* **World:** The term used to describe a collection of robots and objects (such as buildings, tables, and lights), and global parameters including the sky, ambient light, and physics properties.
* **Static:** Entities marked as static (those having the <static>true</static element in SDF), are objects which only have collision geometry. All objects which are not meant to move should be marked as static, which is a performance enhancement.
* **Dynamic:** Entities marked as dynamic (either missing the <static> element or setting <static>false</static> in SDF), are objects which have both inertia and a collision geometry.

# Setup

1.  Make sure Gazebo is [installed](http://gazebosim.org/tutorials?cat=install).

1.  Create a working directory for this tutorial:

    ~~~
    $ mkdir ~/build_world_tutorial; cd ~/build_world_tutorial
    ~~~

1.  Start up gazebo, and you should see a world with just a ground plane.

    ~~~
    $ gazebo
    ~~~

[[file:files/empty_world.png|640px]]

# Adding Objects

Gazebo provides two mechanisms for adding objects to Gazebo.

1.  The first is a set of simple shapes, located above the render window.

[[file:files/empty_world_simple_shapes_highlighted.png|640px]]

1.  The second is via the [model database](https://bitbucket.org/osrf/gazebo_models), which is accessible by selecting the `Insert` tab in the upper left corner.

[[file:files/empty_world_insert_highlighted.png|640px]]

## Adding Simple Shapes

Boxes, spheres, and cylinders may be added to the world by clicking on the appropriate icon above the render window. Each shape is of unit size:

* Box: 1x1x1 meter
* Sphere: 1 meter diameter
* Cylinder: 1 meter diameter, 1 meter length

Select the box icon, and then move your mouse onto the render window. You will see a box that moves with your mouse. Left click when you are happy with the position of the box.

Repeat the same procedure for the sphere and cylinder. You should end with a world similar to this:

[[file:files/simple_shapes.png|640px]]

## Adding Model from the Model Database

Gazebo's model database is a repository of all types of models including robots, tables, and building.

1.  Select the `Insert` tab in the upper left hand corner to access the model database.

    > The list of models are divided into sections according to there current location. Each section is labeled with a path or URI. Selecting an object located on a remote server will cause the model to be downloaded and stored in `~/.gazebo/models`.

2.  Try adding various models to the world. Be patient when downloading models, as some may be large.

3.  You should end up with something similar to the following, depending on the models you decided to add:

[[file:files/added_models_to_empty_world.png|640px]]

# Position Models

The pose of each model may be altered through the translate and rotate tools:

[[file:files/empty_translate_rotate_highlighted.png|640px]]

## Translation
The translate tools allows you to move the object along the x,y, and z axes. Select this tool now, and then move an object by left-pressing and dragging the object.

By default the object will translate around on the x,y plane. You may control which axis the object moves along by pressing and holding the x,y, or z key while dragging the object.

Trying moving the objects around now into a different configuration.

## Rotation
The rotate tool allows you to orient a model around the x,y, and z axes. Select this tool now, and then rotate an object by left-pressing and dragging the object.

By default the object will rotate around the z axis (yaw).  You may control which axis the object rotates around by pressing and holding the x, y, or z key while dragging the object.

Try rotating the objects into a different configuration.

# Delete Models

Model may also be deleted by selecting them and the hitting the `Delete` key, or by right-clicking on a model and selecting `Delete`.

Try deleting a few models.

# Saving a World.

Once you are happy with a world it can be save through the `File` menu.

Select the `File` menu now, and choose `Save As`.

A pop-up will appear asking you to enter a new filename. Enter `my_world.sdf` and click okay.

# Loading a World

A saved world may be loaded on the command line:

~~~
gazebo my_world.sdf
~~~

The filename must be in the current working directory, or you must specify the complete path.

# Next

[Next: Modifying a world tutorial](http://gazebosim.org/tutorials/?tut=modifying_world)
