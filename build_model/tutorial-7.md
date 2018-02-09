# Overview

This tutorial describes the details of a _SDF Model Object_.

SDF Models can range from simple shapes to complex robots. It refers to the `<model>` [SDF](http://gazebosim.org/sdf.html) tag, and is essentially a collection of links, joints, collision objects, visuals, and plugins. Generating a model file can be difficult depending on the complexity of the desired model. This page will offer some tips on how to build your models.

# Components of a SDF Models

> **Links:** A link contains the physical properties of one body of the model. This can be a wheel, or a link in a joint chain. Each link may contain many collision and visual elements. Try to reduce the number of links in your models in order to improve performance and stability. For example, a table model could consist of 5 links (4 for the legs and 1 for the top) connected via joints. However, this is overly complex, especially since the joints will never move. Instead, create the table with 1 link and 5 collision elements.

>> **Collision:** A collision element encapsulates a geometry that is used to collision checking. This can be a simple shape (which is preferred), or a triangle mesh (which consumes greater resources). A link may contain many collision elements.

>> **Visual:** A visual element is used to visualize parts of a link. A link may contain 0 or more visual elements.

>> **Inertial:** The inertial element describes the dynamic properties of the link, such as mass and rotational inertia matrix.

>> **Sensor:** A sensor collects data from the world for use in plugins. A link may contain 0 or more sensors.

>> **Light:** A light element describes a light source attached to a link. A link may contain 0 or more lights.

> **Joints:** A joint connects two links. A parent and child relationship is established along with other parameters such as axis of rotation, and joint limits.

> **Plugins:** A plugin is a shared library created by a third party to control a model.


# Building a Model

### Step 1: Collect your meshes

This step involves gathering all the necessary 3D mesh files that are required to build your model. Gazebo provides a set of simple shapes: box, sphere, and cylinder. If your model needs something more complex, then continue reading.

Meshes come from a number of places. [Google's 3D warehouse](https://3dwarehouse.sketchup.com/index.html) is a good repository of 3D models. Alternatively, you may already have the necessary files. Finally, you can make your own meshes using a 3D modeler such as [Blender](http://blender.org) or [Sketchup](http://sketchup.google.com).

Gazebo requires that mesh files be formatted as STL, Collada or OBJ, with Collada and OBJ being the preferred formats.

> **Tip:** Use your 3D modeling software to move each mesh so that it is centered on the origin. This will make placement of the model in Gazebo significantly easier.

> **Tip:** Collada and OBJ file formats allow you to attach materials to the meshes. Use this mechanism to improve the visual appearance of your meshes.

> **Tip:** Keep meshes simple. This is especially true if you plan on using the mesh as a collision element. A common practice is to use a low polygon mesh for a collision element, and higher polygon mesh for the visual. An even better practice is to use one of the built-in shapes (box, sphere, cylinder) as the collision element.


### Step 2: Make your model SDF file

Start by creating an extremely simple model file, or copy an existing model file. The key here is to start with something that you know works, or can debug very easily.

Here is a very rudimentary minimum box model file with just a unit sized box shape as a collision geometry and the same unit box visual with unit inertias:

Create the `box.sdf` model file

~~~
gedit box.sdf
~~~

Copy the following contents into [box.sdf](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/build_model/files/box.sdf):
<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/build_model/files/box.sdf' />

Note that the origin of the Box-geometry is at the geometric center of the box, so in order to have the bottom of the box flush with the ground plane, an origin of `<pose>0 0 0.5 0 0 0</pose>` is added to raise the box above the ground plane.
> **Tip:** The above example sets the simple box model to be static, which makes the model immovable. This feature is useful during model creation. Once you are done creating your model, set the `<static>` tag to false if you want your model to be movable.


### Step 3: Add to the model SDF file

With a working `.sdf` file, slowly start adding in more complexity. With each new addition, load the model using the graphical client to make sure your model is correct.

Here is a good order in which to add features:

1. Add a link.
1. Set the collision element.
1. Set the visual element.
1. Set the inertial properties.
1. Go to #1 until all links have been added.
1. Add all joints (if any).
1. Add all plugins (if any).
