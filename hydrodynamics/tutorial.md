# Overview

This tutorial describes how to use the `BuoyancyPlugin` in concert with the
`LiftDragPlugin` to simulate the behavior of underwater objects.

# Prerequisites
See [this tutorial](http://gazebosim.org/tutorials?tut=lift_drag&branch=lift_drag)
for an overview of the `LiftDragPlugin`.

# Background

## Buoyancy

[Buoyancy](http://en.wikipedia.org/wiki/Buoyancy) is the force opposing gravity
exerted on an object immersed in a fluid.
It is due to the pressure increase in the volume of fluid when the object is
submerged. There is greater pressure at the bottom of the column of fluid (in the
direction of gravity) than there is at the top. Buoyancy is due to this pressure
difference: it is the tendency for objects to move towards lower pressure. Buoyancy
is the reason some objects float in water.

By [Archimedes' principle](http://en.wikipedia.org/wiki/Archimedes%27_principle), 
the buoyancy force on an object is equal to the weight of the fluid displaced by
submerging that object in the fluid. The weight of the fluid is the force due to
gravity on the fluid. To find this force, we must know the mass of the displaced
fluid, which we can derive from its volume and density. The volume of the displaced
fluid is equal to the volume of the object that displaced the fluid. The density
is a property of the fluid (for example, water has a density of 1000 kg/m^3).
Thus:

~~~
buoyancy force on object = volume of object*density of fluid*gravity
~~~

## Using the BuoyancyPlugin
The `BuoyancyPlugin` is a model plugin that calculates the buoyancy force for each link
in the model and applies the force to the center of volume of the link.

The following parameters can be specified to the plugin SDF, but all parameters are optional.

* `fluid_density`: The density of the fluid surrounding the object in kilograms/cubic meters.
Defaults to 1000 kg/m^3, the density of water.
* `link` element:
..* `name`: an attribute of the `link` element. Must match the name of an existing link
in the model.
..* `center_of_volume`: The center of volume of the link in the link frame. A point in
3-space. Automatically calculated if unspecified.
..* `volume`: The volume of the link in cubic meters. Automatically calculated if unspecified.

If the link's center of volume and volume are not specified, they will be calculated
from the dimensions of the collision shapes that compose the object. This calculation
will be accurate if the object is composed of simple shapes (boxes, cylinders, and spheres),
but a bounding box approximation is used for meshes, polyline shapes, etc.

Here is an example `BuoyancyPlugin` block:

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
Open the `underwater.world` environment with Gazebo paused:

```
gazebo --verbose worlds/underwater.world -u
```

[[file:files/submarines.png]]

This world contains three submarines. When you unpause Gazebo, the white submarine
will float to the top of the world, the black submarine will sink to the ground
plane, and the yellow submarine will maintain a constant height if left undisturbed.

Each submarine model has a propeller link. The yellow submarine has been designed to
move forward when the propeller is spun.

Drag out the left hand menu and select the yellow submarine model. Under the
"Velocity" tab, give the `spinning_joint` joint a value of 300.

[[file:files/leftmenu.png]]

Wait for a moment for the model to accelerate. It will turn around, then start
propelling itself at a constant velocity. When it hits the ground plane, it will
bounce off and propel itself along the angle of deflection.

<iframe width="500" height="313" src="https://youtu.be/Y_y4iXy5YGk" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

## Explanation

### Buoyancy
The white submarine has a density of 500 kg/m^3. It immediately floats to the
top of the world when simulation is unpaused because the buoyancy force is greater
than gravity. On the other hand, the black submarine's density is 1500 kg/m^3, so
it sinks to the ground plane. The yellow submarine has a density of exactly 1000
kg/m^3, so it floats at equilibrium unless disturbed by other forces.

### Lift and Drag

# Extra credit

