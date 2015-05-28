# Overview

This tutorial gives an overview of the physical phenomena of lift and drag
and how they are implemented in Gazebo in the `LiftDragPlugin`. After this
tutorial, you will be able to simulate underwater and aerodynamic robots.

# Physics background

## Fluid mechanics

Fluid mechanics is the study of the forces on or due to liquids and gases.
Solving fluid mechanics problems is complex, and a truly
faithful simulation of fluid mechanics would be computationally prohibitive.
Thus, Gazebo simulates the forces on an object immersed in a fluid and applies
the forces to the object's links directly. In particular, the phenomena of lift
and drag are instrumental to underwater and aerodynamic vehicles.

## Lift

[Lift](https://en.wikipedia.org/wiki/Lift_%28force%29) is the force on a body
due to fluid flowing past the body in the component perpendicular to the flow
direction.

## Drag
[Drag](https://en.wikipedia.org/wiki/Drag_%28physics%29) forces are the forces
on a body due to fluid flowing past the body that acts opposite to the object's
motion.

## Angle of attack and alpha slope
[Angle of attack](http://en.wikipedia.org/wiki/Angle_of_attack) is the angle
between the direction of motion of the body and the reference plane. The
reference plane is usually horizontal (perpendicular to gravity).

When modeling aerodynamics, we must consider the relationship between angle of
attack and coefficient of lift for a body:

[[file:files/lift_curve.png]]

The alpha-lift curve for an object is often determined experimentally.

Drag coefficient has a similar relationship with alpha, though note that the
alpha/lift coefficient and alpha/drag coefficient curves are not necessarily the
same for a given body.

## Stall
The critical angle of attack is the angle at which the alpha-lift curve reaches
its maximum. Stall is defined as the period after the critical angle of attack,
when lift coefficient decreases as a function of angle of attack.

# Using the `LiftDragPlugin`

The `LiftDragPlugin` makes an important assumption about the
relationship between angle of attack (or alpha) and lift coefficient.
Instead of a smooth curve, the alpha/lift coefficient curve is simplified
as two lines.

[[file:files/lift_curve_simplified.png]]

The same assumption is made about the relationship between angle of attack
and drag coefficient.

## Input parameters

* `link_name`: Name of the link affected by the group of lift/drag properties.
* `air_density`: Density of the fluid this model is suspended in.
* `area`: Surface area of the link.
* `a0`: The initial "alpha" or initial angle of attack. `a0` is also the
y-intercept of the alpha-lift coefficient curve.
* `cla`: The ratio of the coefficient of lift and alpha slope before stall.
Slope of the first portion of the alpha-lift coefficient curve.
* `cda`: The ratio of the coefficient of drag and alpha slope before stall.
* `cp`: Center of pressure. The forces due to lift and drag will be applied here.
* `forward`: 3-vector representing the forward direction of motion in the link frame.
* `upward`: 3-vector representing the direction of lift or drag.
* `alpha_stall`: Angle of attack at stall point; the peak angle of attack.
* `cla_stall`: The ratio of coefficient of lift and alpha slope after stall.
Slope of the second portion of the alpha-lift coefficient curve.
* `cda_stall`: The ratio of coefficient of drag and alpha slope after stall.

## Fixed wing model
