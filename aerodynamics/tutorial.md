# Overview

This tutorial gives an overview of the physical phenomena of lift and drag
and how they are implemented in Gazebo in the [LiftDragPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1LiftDragPlugin.html). After this
tutorial, you will be able to simulate aerodynamic robots.

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

[Drag](https://en.wikipedia.org/wiki/Drag_%28physics%29) forces are the
forces on a body due to fluid flowing past the body that acts opposite to
the object's motion.

## Angle of attack and alpha slope

[Angle of attack](http://en.wikipedia.org/wiki/Angle_of_attack), AOA, or alpha,
is the angle between the direction of motion of the body and the reference
plane. The reference plane is usually horizontal (perpendicular to gravity).

[[file:files/angleOfAttack.jpg | 600px]]

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

The [LiftDragPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1LiftDragPlugin.html) makes an important assumption about the
relationship between angle of attack (or alpha) and lift coefficient.
Instead of a smooth curve, the alpha/lift coefficient curve is simplified
as two lines.

[[file:files/lift_curve_simplified.png | 600px]]

The same assumption is made about the relationship between angle of attack
and drag coefficient.

Note in this example, the airfoil has non-zero camber, and has a net
positive lift at zero angle of attack.  To obtain equivalent representation
using the current Gazebo `LiftDragPlugin` plugin parameters, shift the entire
curve to the right such that the zero lift point corresponds to zero
angle of attack. And we can label the original zero angle of attack
location as `a0` in the shifted curve, i.e. `a0` is 5 degrees.
Also shift the stall angle accordingly, i.e. `alpha_stall` is now
19.2 degrees.

Here is an example plugin that implements the lift coefficient
values and stall angles from figure above,

~~~
      <plugin name="lifting_surface" filename="libLiftDragPlugin.so">

        <!-- taken from the lift curve figure -->
        <!-- alpha_0 is 5 degrees -->
        <a0>0.08727</a0>
        <!-- alpha_stall is 19.3 degrees -->
        <alpha_stall>0.3368</alpha_stall>
        <!-- slope of the lift curve to the left of the stall angle -->
        <cla>5.418</cla>
        <!-- slope of the lift curve to the right of the stall angle -->
        <cla_stall>-2.1419</cla_stall>

        <!-- below are just random values in this example -->
        <cda>0.0</cda>
        <cda_stall>0.0</cda_stall>
        <cma>0.0</cma>
        <cma_stall>0.0</cma_stall>
        <area>3</area>
        <fluid_density>1.2041</fluid_density>
        <forward>-1 0 0</forward>
        <upward>0 -1 0</upward>
        <cp>0 0 1</cp>
        <link_name>lifting_surface_link</link_name>
        <radial_symmetry>false</radial_symmetry>
      </plugin>
~~~

Further, the airfoil coordinate system is defined by graph below:

[[file:files/airfoil_coordinates.png | 600px]]

Note without simulating induced drag, the airfoil `forward` direction indicates the intended forward flight direction of the airfoil parallel to the chord line, and the `upward` direction is the direction perpendicular to the `forward` direction towards the lifting direction corresponding to positive angle of attack. By convention, Drag is opposite of the inertial velocity of the body, and lift is perpendicular to drag direction towards positive angle of attack.

Lastly, the center of pressure is defined as an offset in the parent link frame. This is where the free stream velocity is measured, as well as where the computed lift, drag forces and moments are applied. See plot below:

[[file:files/cp_location.png | 600px]]

## Input parameters

The following parameters are used by the [LiftDragPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1LiftDragPlugin.html).

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

* [CessnaPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1CessnaPlugin.html):
This model plugin exposes the topic `~/cessna_c172/control` for controlling the
thrust and control surfaces via Gazebo messages. It also publishes the state of
the model into the topic `~/cessna_c172/state`. Please, read the documentation
included in the header file of this plugin for a detailed explanation of its
required and optional parameters. Here is the plugin block included in our
`cessna_demo.world`:

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

* [CessnaGUIPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1CessnaGUIPlugin.html):
This GUI plugin publishes Cessna messages to modify the angle of the control
surfaces and the thrust power. Next you can find the available Cessna control
keys:

~~~
  w         Increase thrust (+10 %)
  s         Decrease thrust (-10 %)
  d         Increase rudder angle (+1 degree)
  a         Decrease rudder angle (-1 degree)
  Left-Key  Left roll (+1 degree)
  Right-Key Right roll (+1 degree)
  Up-Key    Pitch down (+1 degree)
  Down-Key  Pitch up (+1 degree)
  1         Preset for take-off
  2         Preset for cruise
  3         Preset for landing
~~~

* [LiftDragPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1LiftDragPlugin.html):
We are using this plugin in some of the plane elements to generate lift and
drag. The propeller will generate thrust according to its angular speed. The
control surfaces will generate different forces according to their specific
angles and speed.

Open a new terminal and execute the following command to visualize the state of
the Cessna:

~~~
gz topic -e /gazebo/default/cessna_c172/state
~~~

In the Gazebo window, right click on the model and press `Follow`. The user
camera will follow the plane during the flight and you will not loose it.

Press '1' to start the preset for take-off. The propeller
should start spinning and the model should gain speed along the landing strip.

Use the `Down-arrow key` to pitch up and take-off. Try to balance the plane
on the air with the arrow keys.

<iframe width="420" height="315" src="https://www.youtube.com/embed/iMHGnEhOIhs" frameborder="0" allowfullscreen></iframe>

You can explore all the different control combinations detailed before while
you are flying.
