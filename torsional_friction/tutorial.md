This tutorial is about torsional friction. For translational friction, see [this ](http://gazebosim.org/tutorials?tut=friction) tutorial.

# Overview

** Note: Torsional friction currently works only with the ODE physics engine.**

This tutorial will describe how torsional friction works on Gazebo and how the
SDF parameters should be set.

# Why torsional friction

## Behaviour without torsional friction

Imagine a box rotating on a plane. There's a large surface of contact between
the box and the plane, and at each point of contact, translational friction
acts to decelerate the box. You can see it for yourself:

1. Open Gazebo and insert a box

1. Choose `View -> Transparent` and then `View -> Contacts`

1. Right-click the box and choose `Apply Force and Torque`

1. On the dialog, write 1000.0 Nm on the torque's Z axis, then press
`Apply Torque`. The box will rotate a bit and then stop.

Now try the same with a sphere. You'll see that the sphere will start spinning
and never stop. Why is that?

    [[file:files/box_and_sphere.png|600px]]

The different behaviours happen because Gazebo has translational friction on by
default with default parameters, but torsional friction is off by default.

Therefore, the box stops because the rotation is about the center of the box,
but the contact points are not in the center. Thus, each point has some linear
velocity with respect to the ground plane and triggered translational friction.

On the other hand, the sphere's single point of contact doesn't have a linear
velocity, so translational friction doesn't act on it. We need to give some
parameters to the sphere to enable torsional friction.

## Behaviour with torsional friction

Let's add another sphere to the world and add some torsional friction to it and
see how the behaviour is affected.

1. Insert another sphere in the world

1. Right-click the sphere and choose `Edit model`, you'll enter the model editor

1. Double click the sphere to open its inspector. Go to the collision tab, then
find `Surface -> Friction`. There are a few parameters there which are important
to torsional friction, they will be explained below. For now, we want to set
`Use curvature` to true and input the sphere's radius into `Curvature radius`, which is 0.5 m.

1. Finally, we want to make the sphere very heavy so it presses against the
ground and we see the effects of friction more quickly. So on the `Link` tab,
set the mass to 10000 Kg.

1. Now go to `File -> Exit Model Editor` and make sure you save the model.

1. Back in the main window, repeat the same process above for the new sphere.
Unlike the first sphere (which is still rotating), the new sphere quickly stops
spinning.

# How it works

## Torsional friction equation

Torsional friction torque is computed based on contact depth and the surface
radius as follows:

    T = 3 * PI * a * mu3 * N / 16

Where:

* **T**: Torsional friction torque.

* **N**: Normal force at contact.

* **mu3**: Coefficient of torsional friction. This is usually the same as the
translational friction coefficients **mu** and **mu2**.

* **a**: Contact patch radius (`patch_radius` on SDF). This is the radius of
the contact area between surfaces. A sphere on top of a plane generates a
circular patch area which depends on the sphere radius and the contact depth as
seen below.

    [[file:files/radius_depth.png|400px]]

The patch is calculated as:

    a = sqrt(R * d)

Where:

* **R**: Surface radius at contact point (`curvature_radius` on SDF).

* **d**: Contact depth.

## SDF parameters

As seen in the equations above, unlike translational friction, torsional
friction doesn't depend only on the normal force and the friction coefficient.
It also depends on the area of contact between surfaces.

SDF offers two ways of parametrizing the contact surface. The user can either
define a `patch_radius` (`a` above), which will be always the same independently
of the contact depth or a `curvature_radius` (`R` above), which will take depth
into account. Note that both these values will be valid for a whole surface, so
picking values for non-spherical surfaces might require fine tuning.

To choose between methods, you can set the `use_curvature_radius` tag to true
for `curvature_radius` and false to use `patch_radius`.

### Default values

* **mu3**: Like **mu** and **mu2**, **mu3** has a default value of 1.0.

* **use_curvature_radius**: False by default, so the `patch_radius` is used.

* **patch_radius**: Zero by default, so even if though `mu3` is set, there will
be no torsional friction.

* **curvature_radius**: Zero by default.

### Example

Back to the example above, since we knew the sphere radius, we chose the
`curvature_radius` method by setting `use_curvature_radius` to true. Then we
set the correct radius as well.

Since torsional friction by definition is very little (think of how long a top
keeps spinning), we increased the sphere mass to a huge value to make a lot of
pressure and increase contact depth, thus increasing torsional friction.

On SDF, the torsional friction for the sphere example would look as follows:

    <model ...>
     ...
      <link ...>
        <collision ...>
         ...
          <surface>
            <friction>
              <ode>
                <mu3>1.0</mu3>
                <curvature_radius>0.5</curvature_radius>
                <use_curvature>true</use_curvature>
              </ode>
            </friction>
           ...
          </surface>
        </collision>
      </link>
    </model>

# Example world

Gazebo comes with a demo world which you can run as follows:

    gazebo -u worlds/torsional_friction_demo.world

<iframe width="560" height="315" src="https://www.youtube.com/embed/LuteVPy92-0" frameborder="0" allowfullscreen></iframe>


