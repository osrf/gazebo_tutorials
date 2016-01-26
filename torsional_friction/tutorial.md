This tutorial is about torsional friction. For translational friction, see
[this](http://gazebosim.org/tutorials?tut=friction) tutorial.

# Overview

**Note: Torsional friction currently works only with the ODE physics engine.**

This tutorial will explain how torsional friction works and describe how to
set it up from the GUI or SDF.

# Why torsional friction

## Behaviour without torsional friction

When a box is rotating on top of a plane, there's a large surface of contact
between them. At each point of contact, translational friction acts to
decelerate the box. You can try it by yourself:

1. Open Gazebo with the default physics engine and insert a box

1. Choose `View -> Transparent` and then `View -> Contacts`, so we can see the
contact points.

1. Right-click the box and choose `Apply Force and Torque`

1. On the dialog, write 1000.0 Nm on the torque's Z axis, then press
`Apply Torque`. The box will rotate a bit and then stop.

Now try the same with a sphere. You'll see that the sphere starts spinning and
never stops. Why is that?

[[file:files/box_and_sphere.png|600px]]

The different behaviours happen because Gazebo has translational friction
enabled by default with default parameters, but torsional friction is disabled
by default.

The box has a large surface with several points of contact with the ground, you
can see them masked as blue spheres on the image above. Each contact point has
both angular and linear velocity perpendicular to the contact normal. The
presence of linear velocity triggers translational friction.

On the other hand, the sphere's single point of contact doesn't have a linear
velocity, so translational friction doesn't act on it. If we want the sphere to
stop due to friction, we will need to set up torsional friction for it.

## Behaviour with torsional friction

Let's add another sphere to the world and set a few parameters to it which will
enable torsional friction.

1. Insert another sphere in the world

1. Right-click the sphere and choose `Edit model`, you'll enter the model
editor.

1. Double click the sphere to open its inspector. Go to the Collision tab, then
find `Surface -> Friction`. There are a few parameters there which are important
for torsional friction, they will be explained below. For now, we want to set
`Use patch radius` to false and input the sphere's radius into `
Surface radius`, which is 0.5 m. Note that the torsional friction coefficient
is 1.0 by default.

1. Torsional friction is typically very low because of the small contact area.
For our experiment, we want a lot of friction so the sphere stops fast. One way
to achieve this is to make the sphere very heavy so it presses against the
ground. So let's go on the `Link` tab and set the mass to 10000 Kg. The heavier
it is, the more it presses against the ground and the higher the friction.

1. Apply your changes, close the inspector and  go to `File -> Exit Model Editor`.
Make sure you save the new model.

    > **Tip:** The simulation stays paused after returning from the Model Editor.
    Make sure that you click the play button before continuing.

1. Back in the main window, apply torque to the new sphere as you did for the
old one. Unlike the first sphere (which is still rotating, it will rotate
forever), the new sphere quickly stops spinning when the torque is applied.

# How it works

## Torsional friction equation

Torsional friction torque is computed based on contact depth and surface
radius as follows: (you can find more detailed calculations
[here](http://nbviewer.ipython.org/github/osrf/collaboration/blob/master/Torsional%20Friction.ipynb))

    T = 3*PI/16 * a * coefficient * N

Where:

* **3 PI / 16**: constant approximately equal to 0.589

* **T**: Torque due to torsional friction.

* **N**: Normal force at contact.

* **coefficient**: Coefficient of torsional friction. This is usually the same
as the translational friction coefficients **mu** and **mu2**.

* **a**: Contact patch radius (`patch_radius` in SDF). This is the radius of
the contact area between surfaces. A sphere on top of a plane generates a
circular patch area which depends on the sphere radius and the contact depth as
seen below.

    [[file:files/radius_depth.png|400px]]

The patch is calculated as:

    a = sqrt(R * d)

Where:

* **R**: Surface radius at contact point (`surface_radius` in SDF).

* **d**: Contact depth.

## SDF parameters

As seen in the equations above, unlike translational friction, torsional
friction doesn't depend only on the normal force and the friction coefficient.
It also depends on the area of contact between surfaces.

SDF offers two ways of parametrizing the contact surface. The user can either
define a `patch_radius` (**a** above), which will be always the same
independently of the contact depth or a `surface_radius` (**R** above), which
is used together with contact depth. Note that in both cases, the user is
specifying a single value for the whole surface, so picking values for
non-spherical surfaces might require fine tuning.

To choose between methods, you can set the `use_patch_radius` tag to true
for `patch_radius` and false to use `surface_radius`.

### Default values

* **coefficient**: Like **mu** and **mu2**, **coefficient** has a default value
of 1.0.

* **use\_patch\_radius**: True by default, so the `patch_radius` is used.

* **patch\_radius**: Zero by default, so even if `coefficient` is set, there will
be no torsional friction.

* **surface_radius**: Zero by default, so even if `coefficient` is set, there
will be no torsional friction.

### Example

Back to the example above, since we knew the sphere radius, we chose the
`surface_radius` method by setting `use_patch_radius` to false. Then we
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
              <torsional>
                <coefficient>1.0</coefficient>
                <surface_radius>0.5</surface_radius>
                <use_patch_radius>false</use_patch_radius>
              </torsional>
            </friction>
           ...
          </surface>
        </collision>
      </link>
    </model>

# Example world

Gazebo comes with a demo world which you can run as follows:

    gazebo -u worlds/torsional_friction_demo.world

On the demo there are various models rotating. Models with higher friction stop
first.

<iframe width="560" height="315" src="https://www.youtube.com/embed/LveVKwiXlx0" frameborder="0" allowfullscreen></iframe>


