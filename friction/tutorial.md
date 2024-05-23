# Introduction

Each physics engine in Gazebo (for example ODE, Bullet, Simbody, DART) has
different friction models. Please refer to the [SDFormat
specification](http://sdformat.org/spec?ver=1.5&elem=collision#surface_friction)
and the [Physics Parameters tutorial](/tutorials?tut=physics_params#Frictionparameters)
for a complete
listing of available friction parameters.

The rest of this tutorial will assume you are using ODE, the default physics
engine.

# How pyramid friction works in ODE

When two object collide in simulation, such as a ball rolling on a plane,
the collision checker generates one or more contact points each with a position
and a unit vector specifying the direction of the contact normal.
At each contact point, constraints are applied to prevent penetration in
the normal direction and to enforce friction constraints in the tangential
directions. There are several friction models available for use in ODE: the
default `pyramid_model` as well as the `box_model` and `cone_model`.
The remainder of this tutorial focuses on the `pyramid_model`; please see
the description of the `friction_model` parameter in the
[Physics Parameters tutorial](/tutorials?tut=physics_params#Frictionparameters)
for more information about the other models.

In addition to the normal constraint acting in the direction of the normal unit
vector, the `pyramid_model` defines two friction directions along which
Coulomb friction constraints are enforced to limit the relative surface motion
of the objects in contact.
The two friction directions (denoted as first friction direction / "F-dir 1" /
`fdir1` and second friction direction / "F-dir 2") are unit vectors orthogonal
to the normal vector and to each other (see the illustration from the ODE user
manual below).
The user can specify a desired value for the first friction direction, which
may be adjusted to ensure that it is orthogonal to the normal vector, and the
second friction direction will be computed automatically.

![Illustration of variable related to ODE contact constraint.](http://ode.org/wiki/images/b/b9/Contact.jpg)

The `pyramid_model` friction constraints accept different parameters for each
direction, such as `mu` for the Coulomb friction coefficient in the
first direction and `mu2` for the second direction,
with the tangential force in each direction (`F1` and `F2`) constrained not
to exceed the product of Coulomb friction coefficient and
normal force magnitude (`N >= 0`) as `|F1| <= mu * N` and `|F2| <= mu2 * N`.

Specifying different friction parameter values in different directions
(also known as anisotropic friction) can be useful,
for example in modeling wheel contact with different behavior in the
longitudinal and lateral wheel directions.
Note however that the `pyramid_model` friction constraints are solved
independently, which simplifies the math but can introduce non-physical behavior
in some situations.
See the comparison of friction models in the
[Physics Parameters tutorial](/tutorials?tut=physics_params#Frictionparameters)
and consider using the `cone_model` if this is a concern.

When two objects in collision specify '''mu''' and '''mu2''', Gazebo will
choose the smallest '''mu''' and '''mu2''' from the two colliding objects.

The valid range of values for '''mu''' and '''mu2''' is any non-negative number,
where 0 equates to a friction-less contact
and a large value approximates a surface with infinite friction.
Tables of friction coefficient values for a variety of
materials can be found in engineering handbooks
or [online references](http://www.engineeringtoolbox.com/friction-coefficients-d_778.html).

## How to specify friction

It's always best to refer to the [SDF documentation](http://sdformat.org/spec?ver=1.5&elem=collision#surface_friction).

The following example will specify a box with low friction:

~~~
<model name="box">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.01</mu>
            <mu2>0.01</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
~~~

This is a more exhaustive [example](https://github.com/osrf/gazebo/blob/gazebo_1.9/sdf/worlds/test_friction.world).

## Specifying body-fixed friction directions with `fdir1`

By default, the first friction direction for all contacts is the X-axis
(`1 0 0`) in the world frame.
A model can specify body-fixed friction directions by specifying a vector
with non-zero length in the
[`fdir1` parameter](http://sdformat.org/spec?ver=1.11&elem=collision#ode_fdir1)
in the surface parameters of a collision.
This unit vector is interpreted in the coordinate frame fixed to its parent
collision, so care should be taken when the collision pose orientation differs
from its parent link and model.

### Simple example with boxes

For example, the following model contains two links with equivalent shape and
surface properties. The first link has no pose rotation, while the second
link has a 90-degree "roll" rotation about the body-fixed X axis, which
requires the box dimensions and `fdir1` specification to differ in order
to account for the rotation and maintain the same net behavior.

~~~
<sdf version='1.6'>
  <model name='fdir1_example'>
    <link name='unrotated_link_fdir1_in_y'>
      <collision name='collision'>
        <geometry>
          <box>
            <!-- smallest: X, largest Z -->
            <size>0.1 0.4 0.9</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <!-- first friction dir: Y -->
              <fdir1>0 1 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <!-- smallest: X, largest Z -->
            <size>0.1 0.4 0.9</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
    <link name='equivalent_link_with_pose_rotation'>
      <!-- Rotated 90 degrees in "roll" along X -->
      <pose>1 0 0 1.5707963267948966 0 0</pose>
      <collision name='collision'>
        <geometry>
          <box>
            <!-- smallest: X, largest Y -->
            <size>0.1 0.9 0.4</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <!-- first friction dir: Z -->
              <fdir1>0 0 1</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <!-- smallest: X, largest Y -->
            <size>0.1 0.9 0.4</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
~~~

### More general example with many boxes

A more general example is
[friction_dir_test.world](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/test/worlds/friction_dir_test.world)
which includes concentric semi-circles of boxes with varying rotations of
model, link, and collision pose as well as variations of `fdir1`.
The models are constructed such that boxes with equivalent surface properties
are initially positioned in a radial line from the center of the semicircles
and are named with a common integer suffix (`_0`, `_1`, `_2`, etc).
The boxes have a high friction coefficient in one direction
and zero friction in the other direction (`mu == 100` and `mu2 == 0`), and
the world's gravity vector is inclined, causing boxes to slide at an angle
defined by their friction directions, as shown in the following animation:

![animation of concentric rings of boxes sliding along their defined friction directions](https://osrf-migration.github.io/gazebo-gh-pages/data/bitbucket.org/repo/jgXqbo/images/2692916182-friction_dir_test_small.gif)

### Example with a spinning wheel

Special care is needed when specifying a body-fixed friction direction for
a spinning wheel, since the body-fixed frame rotates as the wheel spins.
The best approach is to align the first friction direction with the axis of
rotation of the wheel, so that the first friction direction unit vector is
invariant to the rotation.

For example, the `trisphere_cycle` model has three wheels attached by
revolute `*_spin` joints with the `//axis/xyz` unit vector specified along
the joint Y axis (`0 1 0`):
[wheel_front_spin](https://github.com/osrf/gazebo_models/blob/master/trisphere_cycle/model.sdf#L279-L285),
[wheel_rear_left_spin](https://github.com/osrf/gazebo_models/blob/master/trisphere_cycle/model.sdf#L329-L335),
and
[wheel_rear_right_spin](https://github.com/osrf/gazebo_models/blob/master/trisphere_cycle/model.sdf#L379-L385).
To specify the `fdir1` parameter parallel to the axis of the `*_spin` joints,
the parameter should be set to `0 1 0` to match the joint axis specification,
since there are no pose rotations for the wheel links or collisions.
The `fdir1` parameter is not set by default for the `trisphere_cycle`, but
a variant used for testing a plowing model
(see [plowing_effect_trisphere_cycle](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/test/models/plowing_effect_trisphere_cycle/model.sdf#L339))
does set it to `0 1 0` as suggested here.

![trisphere_cycle model](https://osrf-migration.github.io/gazebo_models-gh-pages/data/bitbucket.org/repo/EgGzpd/images/3164252219-trike.png)

This approach to setting the first friction direction for wheels establishes
a convention that the lateral wheel direction should be the first friction
direction. The second friction direction will then be the longitudinal
direction.

Note that extra care should be taken when the wheel link and collision poses
have nonzero rotations specified, which is common when modeling wheel
collisions with Z-aligned cylinders.
For example, in the test world named
[tire_drum_test.world](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/test/worlds/tire_drum_test.world)
the [wheel link has a 90-degree pose rotation about the X axis](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/test/worlds/tire_drum_test.world#L23).
Though the wheel spin joint axis is parallel to the Y-axis of the world frame,
the joint axis XYZ value is specified along the body-fixed Z-axis
(see [model](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/test/worlds/tire_drum_test.world#L175))
and the wheel's `fdir1` parameter uses the [same value](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/test/worlds/tire_drum_test.world#L60).
