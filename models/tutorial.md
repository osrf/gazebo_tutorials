#What is a model?
Models are one of the most important aspects of Gazebo. Models in Gazebo define a
physical entity with dynamic, kinematic, and visual properties. A model can represent
anything from a simple shape to a complex robot; even the ground is a model. In addition,
a model may have one or more plugins, which affect the model's behavior. Any environment
or even a single element can make up a model. There exists a whole [database](https://bitbucket.org/osrf/gazebo_models/src)
of online models which the simulator can access.

Models are defined in an XML format called the Simulation Description Format ([SDF](http://sdformat.org/)).
Originally developed as part of the Gazebo robot simulator, SDF was designed with scientific
robot applications in mind. Over the years, SDF has become a stable, robust, and extensible format
capable of describing all aspects of robots, static and dynamic objects, lighting, terrain, and
even physics. It is used for simulation, visualization, motion planning, and robot control. The
format of SDF is described by [XML](https://en.wikipedia.org/wiki/XML), which facilitates updates and
allows conversion from previous versions. Learn how to programmatically build a simple model [here](http://gazebosim.org/tutorials?tut=build_model). 

#What’s in a model?
As can be seen [here](http://sdformat.org/spec?ver=1.5&elem=model), models are made up of quite a few components :

> **Links:** [Link](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Link.html) class defines a rigid body entity, containing information on inertia, visual and collision properties of a rigid body.
A link contains the physical properties of one body of the model. This can be a wheel of
a vehicle, or a link in a joint chain. Each link may contain many collision and visual
elements. Each link has three main componets: Inertial (the m in F=ma for the physics engines);
Collision (geometrical element, a shape that comes to play whenever collisions are considered);
Visual (similar to collision: a geometrical element, but it comes to play while rendering).

Tip : Try to reduce the number of links in your models to improve performance and stability.
For example, a table model could consist of 5 links (4 for the legs and 1 for the top) connected via
joints. However, this is overly complex, especially since the joints will never move. Instead, create
the table with 1 link and 5 collision elements.

>> **Collision: (Geometry)** Base class for all [collision](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Collision.html#details) entities. A collision element is a geometry that is used to
check if models will collide with each other and consecutively decides the behaviour of the two elements
if they collide. This can be a simple shape (which is preferred), or a [triangle mesh](https://en.wikipedia.org/wiki/Triangle_mesh), which might represent 3D shapes more accurately, but consumes greater computational resources.
A link may contain many collision elements. 
The collision is used by the physics engine whenever two surfaces come in contact with each other.
A visual element might be too complex to support real-time collision checking, so instead, we use a dummy
element called a collision element. For example: the Polaris Ranger (a model in the model [database](https://bitbucket.org/osrf/gazebo_models/src)) model has a car visual but has a box as collision element, making the physics more simple and efficient. 

>> **Visual: (Geometry)** A [visual](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1rendering_1_1Visual.html) is the component that 
makes the link look a certain way. You can think of it as the skin of the link.
A visual is a renderable object. Aside from the user, the visual is only seen by a subset of sensors,
like cameras. A link may contain 0 or more visual elements.

Most graphical engines use polygonal meshes to visualize elements; triangles are the most simple
polygons. With increasing curvature, triangle size decreases and the quantity increases.

>> **Inertial:** The [inertial](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Inertial.html)(for physics engines) 
element describes the dynamic properties of the link, such as mass and inertia matrix. 
These properties will affect the way the link interacts with forces in the world. 
For example, how it falls due to gravity, and which impulses it generates when it collides with other links.


>> **Sensor:** A sensor collects data from the world for use in plugins. 
For example, a camera sensor collects images, and a laser range sensor collects distances. 
A link may contain 0 or more sensors.
Sensors are crucial in replicating real-time robots and environments. 
No robot can function without taking input from the external world.
The various types of sensors in the Gazebo API can be found here:
1. Camera: Render to offscreen buffer
2. Kinect: Depth camera
3. Laser: CPU and GPU-based ray casting
4. Contact: Generated by collision engine
5. RFID: Information generated from model positions
6. Force torque: Specific to joints

[These](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/group__gazebo__sensors.html) tutorials describe how to use and modify sensors.


> **Joints:** A joint connects two links with kinematic and dynamic properties. 
A parent and child relationship is established along with other parameters such as axis of rotation, and joint limits. 
More specifications can be found [here](http://sdformat.org/spec?ver=1.5&elem=joint#joint_parent).

The various types of joints are:
1. Revolute
2. Revolute2
3. Prismatic
4. Screw
5. Fixed
6. Ball
7. Universal
8. Gearbox

> **Plugins:**  A plugin is a shared library created by a third party to control models. 
A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation. 
The plugin has direct access to all of the functionality of Gazebo through the standard C++ classes.
Plugins are useful because they:

>> Let developers control almost any aspect of Gazebo
>> Are self-contained routines that are easily shared
>> Can be inserted and removed from a running system

Previous versions of Gazebo utilized controllers. These behaved in much the same way as plugins, 
but were statically compiled into Gazebo. Plugins are more flexible, 
and allow users to pick and choose what functionality to include in their simulations.

You should use a plugin when:
>> You want to programmatically alter a simulation.
Example: move models, respond to events, insert new models given a set of preconditions.
>> You want a fast interface to Gazebo, without the overhead of the transport layer
Example: No serialization and deserialization of messages.
>> You have some code that could benefit others and want to share it.

A Model plugin is attached to, and controls, a specific model in Gazebo. 
Similarly, a World plugin is attached to a world, and a Sensor plugin to a specific sensor. 
The System plugin is specified on the command line, and loads first during a Gazebo startup. 
This plugin gives the user control over the startup process.
A plugin type should be chosen based on the desired functionality. 
Use a World plugin to control world properties, such as the physics engine, ambient lighting, etc. 
Use a Model plugin to control joints and the state of a model. 
Use a Sensor plugin to acquire sensor information and control sensor properties.
Model Plugins are further discussed [here](http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin).

#Nested Models
Nested Models connect models in a parent-child relationship.
Nested models come into play in cases when you need a model which itself is an assembly of other models.
Nesting enables users to specify /<model> elements inside another <model> element.

# Why do you need nesting?
Suppose I have a model consisting of a coffee shop and a different model consisting of a table. 
I want the coffee shop model to contain lots of tables, so I need to nest the table 
models within the coffee shop model, creating one model out of many.

# How do we do nesting?

~~~
<model name=”xyz”>
<pose=... >
<link= …. >
< collision= …>

<model name=”xyz_inner”>
<pose = …..>
<link =  ….>
…
</model>
.
.
.

</model>
~~~

Example can be seen [here](https://bitbucket.org/osrf/gazebo/raw/e4b49fd4734aac84389c47ee76bd8a0bb4c6d081/worlds/nested_model.world).





