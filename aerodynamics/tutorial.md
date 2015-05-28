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

[[file:files/lift_curve.png | 600px]]

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

[[file:files/lift_curve_simplified.png | 600px]]

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

Open the `cessna_demo.world` environment with Gazebo:

~~~
gazebo --verbose worlds/cessna_demo.world
~~~

[[file:files/cessna_demo.png | 600px]]

This world contains a model of the Cessna C-172 with three different plugin
types:

* [CessnaPlugin](https://bitbucket.org/osrf/gazebo/raw/default/plugins/CessnaPlugin.hh): This model plugin exposes the topic `~/cessna_c172/control`
for controlling the thrust and control surfaces via Gazebo messages. It also
publishes the state of the model into the topic `~/cessna_c172/state`.
Please, read the documentation included in the header file of this plugin for a
detailed explanation of its required and optional plugins. Here is the plugin
block included in our `cessna_demo.world`:

  ~~~
  <!-- A plugin for controlling the thrust and control surfaces -->
  <plugin name="cessna_control" filename="libCessnaPlugin.so">
    <propeller>cessna_c172::propeller_joint</propeller>
    <propeller_max_rpm>2500</propeller_max_rpm>
    <left_aileron>cessna_c172::left_aileron_joint</left_aileron>
    <left_flap>cessna_c172::left_flap_joint</left_flap>
    <right_aileron>cessna_c172::right_aileron_joint</right_aileron>
    <right_flap>cessna_c172::right_flap_joint</right_flap>
    <elevators>cessna_c172::elevators_joint</elevators>
    <rudder>cessna_c172::rudder_joint</rudder>
  </plugin>
  ~~~

* [CessnaGUIPlugin](https://bitbucket.org/osrf/gazebo/raw/default/plugins/CessnaGUIPlugin.hh): This GUI plugin publishes Cessna messages to modify the
angle of the control surfaces and the thrust power. Next you can find the Cessna
control keys:

~~~
  a Increase thrust (+1 %)
  z Decrease thrust (-1 %)
  s Increase ailerons + flaps angle (+1 degree)
  x Decrease ailerons + flaps angle (-1 degree)
  d Increase elevators angle (+1 degree)
  c Decrease elevators angle (-1 degree)
  f Increase rudder angle (+1 degree)
  v Decrease rudder angle (-1 degree)
~~~

* [LiftDragPlugin](https://bitbucket.org/osrf/gazebo/raw/default/plugins/LiftDragPlugin):
The propeller will generate thrust based on its angular speed. The control
surfaces will generate different forces according to their specific angles.

Open a new terminal and execute the following command to visualize the state of
the Cessna:

~~~
gz topic -e /gazebo/default/cessna_c172/state
~~~

Go ahead and press 'a' to increase the thrust up to 50% (0.5). The propeller
should start spinning and the model should gain speed along the landing strip.

Press 'x' to change ailerons and flaps up to 30 degrees (0.52 rads). Your model
should take off.

Restore ailerons and flaps to 0 degrees by pressing 's' to make the Cessna
cruise.

You can press 'f' and 'v' to change de rudder angle and make the plane to turn
and face the landing strip.

Shutdown the engine by pressing 'z' until the thrust is 0 and press 's' to set
flaps and ailerons to 30 degrees again. If you're a good pilot you should be
able to land on the ground.
