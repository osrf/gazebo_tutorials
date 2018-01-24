# Introduction

As of Gazebo6, it is possible to control which geometries collide. A bitmask
may be applied to an SDF collision element, where the bitwise-and of two
collision's bitmasks determines if they may collide.

# How collide bitmasks work

A bitmask is a number, usually specified in hexadecimal, that is associated
with some object. In our case the objects are collision geometries.  When
two bitmasks are compared, through a bitwise-and operation,
the outcome is used to control what logic is applied. The
logic in our case is whether a collision is possible or
not.

Let's consider an example that consists of three boxes, **boxA**, **boxB**,
and **boxC**. The boxes have bitmasks 0x01, 0x02, and 0x03 for **boxA**,
**boxB**, and **boxC** respectively. In binary these bitmasks would be:

  - **boxA**: 01
  - **boxB**: 10
  - **boxC**: 11

Let's assume that all three boxes are in as simulated world, where **boxA**
is directly below **boxB** and **boxC** is directly above boxB. This creates
a stack of boxes with **boxA** on bottom, and **boxC** on top. We will also
assume that all the boxes are capable of colliding with the ground.

Once simulation starts, **boxA** and **boxB** will be in collision. However,
the bitwise-and of their bitmasks will produce a value of zero. This
indicates to the simulation engine that collisions should be ignored,
and the result is **boxB** passes through boxA.

At this point **boxA** and **boxB** are both co-located and resting on the
ground.  Meanwhile, **boxC** is falling from its start position. BoxC will
eventually hit boxA and boxB. The bitwise-and of boxC's bitmask and the
bitmask belonging to both **boxA** and **boxB** will result in a value
greater than zero. This non-zero result, also considered ''true'' by
programming languages, indicates to the simulation engine that collisions
should be generated.  The result is that **boxC** will come to rest on top
of **boxA** and boxB. 

This example is available as a run-able Gazebo demo.

  1. Run the demo world, and start in a paused state

      ~~~
      gazebo -u worlds/shapes_bitmask.world
      ~~~

  1. Press the play button to see the boxes drop.

      [[file:files/collide_bitmask.png|640px]]

# How to set a collide bitmask

##1)Using GUI in Gazebo Software:##
In Model Editor, Right click on Link -> Link Inspector -> Collision and change **Collide bitmask** as stated above by box examples.
here bitmask value can be specified in range 0-100000000 using base-10.

##2)Using SDF :##
A collide bitmask may be set using [SDF](http://sdformat.org), an XML file
format for describing simulation properties and entities. The bitmask XML
element is a child of the surface XML element. The bitmask value may be specified using either base-10 or base-16. The following is a simple example, note that some necessary SDF elements have been removed for clarity.

~~~
<model name="box">
  <link name="link">
    <collision name="collision">
      <geometry> ... </geometry>

      <surface>
        <contact>
          <collide_bitmask>0x01</collide_bitmask>
        </contact>
      </surface>

    </collision>
    <visual name="visual"> ... </visual>
  </link>
</model>
~~~

### Default value

Each geometry, including the ground plane, has a default value of 0xffff for
its collide bitmask. You may use the upper 16-bits for objects that you do
not want to collide with the ground plane, and other default objects.

### Self Collision
Self collide option is used for control of collisions between the links in the model itself.
##how to set self collide :##
Set Self collide to **true** value (Right click on Link -> Link Inspector -> Link).

or if you are working on SDF file then follow the example below :

~~~
<model name="model">
  <link name="link1">
    <collision name="collision">...</collision>
    <self_collide>1</self_collide>
    <visual name="visual"> ... </visual>
  </link>
  
  <link name="link2">
    <collision name="collision">...</collision>
    <self_collide>1</self_collide>
    <visual name="visual"> ... </visual>
  </link>
</model>
~~~

this way link 1 and link2 of same model will collide for the proper bitmask values.