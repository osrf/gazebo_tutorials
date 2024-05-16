# Overview

This tutorial describes how to use the [BuoyancyPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1BuoyancyPlugin.html) in concert with the
[LiftDragPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1LiftDragPlugin.html) to simulate the behavior of underwater objects.

# Prerequisites
See [this
tutorial](/tutorials?tut=aerodynamics&cat=physics)
for an overview of the [LiftDragPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1LiftDragPlugin.html).

# Background

## Buoyancy

[Buoyancy](http://en.wikipedia.org/wiki/Buoyancy) is the force opposing
gravity exerted on an object immersed in a fluid. When an object is
submerged in a volume of fluid pressure increases. There is greater pressure
at the bottom of the column of fluid (in the direction of gravity) than
there is at the top. Buoyancy is due to this pressure difference: it is the
tendency for objects to move towards lower pressure. Buoyancy is the reason
some objects float in water.

By [Archimedes' principle](http://en.wikipedia.org/wiki/Archimedes%27_principle), the
buoyancy force on an object is equal to the weight of the fluid displaced by
submerging that object in the fluid. The weight of the fluid is the force
due to gravity on the fluid. To find this force, we must know the mass of
the displaced fluid, which we can derive from its volume and density. The
volume of the displaced fluid is equal to the volume of the object that
displaced the fluid. The density is a known property of the fluid (for
    example, water has a density of approximately 999 kg/m^3 at 1 atm
    pressure).

Thus:

~~~
buoyancy force on object = volume of object*density of fluid*gravity
~~~

## Using the BuoyancyPlugin

The `BuoyancyPlugin` is a [Model plugin](http://gazebosim.org/api/code/dev/classgazebo_1_1ModelPlugin.html) that calculates the buoyancy force for each link in the model and applies the force to the center of volume of the link.  The following optional plugin parameters can be specified using SDF.

* `<fluid_density>`: The density of the fluid surrounding the object in kilograms/cubic meters.  Defaults to 999.1026 kg/m^3, the density of water.
* `<link>`:
  * `name`: an attribute of the `<link>` element. Must match the name of an existing link in the model.
  * `<center_of_volume>`: The center of volume of the link in the link frame. A point in 3-space. Automatically calculated if unspecified.
  * `<volume>`: The volume of the link in cubic meters. Automatically calculated if unspecified.

If a link's `<center_of_volume>` and `<volume>` are not specified, they will
be calculated from the dimensions of the collision shapes that compose the
object. This calculation will be accurate if the object is composed of
simple shapes (boxes, cylinders, and spheres). A bounding box approximation
is used for meshes, and polyline shapes.

Here is an example `BuoyancyPlugin` SDF:

~~~
<model name="boat">
  <link name="body">
    <!-- ... link info here ... -->
  </link>

  <plugin name="BuoyancyPlugin" filename="libBuoyancyPlugin.so">
    <!-- a viscous liquid -->
    <fluid_density>2000</fluid_density>
    <link name="body">
      <center_of_volume>1 2 3</center_of_volume>
      <volume>50</volume>
    </link>
  </plugin>
</model>
~~~

# Demo

Open the `underwater.world` environment with Gazebo-classic paused:

~~~
gazebo --verbose worlds/underwater.world -u
~~~

[[file:files/submarines.png | 600px]]

This world contains three submarines. When Gazebo-classic is unpaused, the white
submarine will float to the top of the world, the black submarine will sink
to the ground plane, and the yellow submarine will maintain a constant
height if left undisturbed.

Each submarine model has a propeller link. The yellow submarine has been designed to move forward when the propeller is spun.

Drag out the right hand menu and select the yellow submarine model. Give the
`spinning_joint` a small positive torque.

[[file:files/leftmenu.png | 600px]]

The submarine will move forward as the propeller spins. Increasing the
torque will cause the submarine to propel forward faster.

<iframe width="420" height="315" src="https://www.youtube.com/embed/Jmz-N7zqK8g" frameborder="0" allowfullscreen></iframe>

## Explanation

### Buoyancy

The white submarine has a density of 500 kg/m^3. It immediately floats to
the top of the world when simulation is unpaused because the buoyancy force
is greater than gravity. On the other hand, the black submarine has
a density is 1500 kg/m^3, so it sinks to the ground plane. The yellow
submarine has a density of exactly 999.1026 kg/m^3, so it floats at
equilibrium unless disturbed by other forces.

### Lift and Drag

The submarine propellers are constructed so that spinning the propeller
joint generates drag forces in forward direction. The blades are tilted such
that the angle of attack for the blades is always less than 45 degrees. The
propeller blades spin in the plane orthogonal to the direction of forward
motion. Because lift and drag are orthogonal, a lift force gets generated in
the forward direction. The lift force on the propeller causes the entire
body to move forward.

For each propeller blade, the lift and drag forces are applied at the center
of pressure of the blade, approximately 70% of the radius from the center of
the propeller along the axis of the blade. The center of pressure is also
the point at which the velocity is measured for calculating the lift and
drag forces. This is important for a rotating body because if the center of
the propeller were used to calculate lift and drag, the forces would always
be zero.
