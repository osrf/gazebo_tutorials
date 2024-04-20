# Overview
This tutorial gives an overview of the haptix-comm simulation-specific API.

We assume that you have already completed the
[installation step](/tutorials?tut=haptix_install&cat=haptix).

# Documentation
The full world API documentation can be found
[here](https://s3.amazonaws.com/osrf-distributions/haptix/api/0.7.1/haptix__sim_8h.html).

## Conventions and terminology

A "transform" is a 3D position and rotation, also known as a pose. In the world API,
we represent rotations as [quaternions](http://en.wikipedia.org/wiki/Quaternion).

Unless otherwise stated, all vectors and transforms are relative to a fixed world frame.

Through the documentation, we refer to "links", "joints", and "models".

A "link" is a rigid body that moves as a whole.

A "joint" holds together two links, constraining their motion. A joint may have limits.

A "model" is a named object that may consist of zero or more links and joints. Most objects
in this example will only have one link and zero joints, such as the wood cube and the
cricket ball. However, the robotic prosthetic arm is an example of a model with many
links and joints.

For example, a typical doorway has three links: the frame, which is fixed, the door
itself, and the handle.
The hinges that connect the frame and the door
are modeled as a joint that allows the door to swing.
Likewise, the door and the handle are connected by a
joint that allows the handle to turn.
The doorway as a whole would be considered a model.

# Example
In our example, we will demonstrate how to use the world API to manipulate
objects in the ARAT world.

## Run the code: MATLAB
The HAPTIX MATLAB SDK should already contain the `hxs_example.m` script used in
this tutorial. If it was moved or deleted, you can download it
[here](https://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m)
If you need to redownload it, make sure to place the script into the folder
containing the HAPTIX MATLAB SDK, or on your MATLAB path.

To run the example, first make sure Gazebo-classic is running (double-click on the desktop icon
or use the terminal).

Open MATLAB and navigate to the HAPTIX SDK folder. Then type `hxs_example` into the
command prompt.

Watch the Gazebo-classic window as the script runs through each example API call.

## Run the code: Octave on Linux
Installing the `haptix-comm` package on Linux will install an `octave` folder to
`/usr/lib/x86_64-linux-gnu/haptix-comm`. This folder contains `hxs_example` script
used in this tutorial. If it was moved or deleted, you can download it
[here](https://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m)

To run the example, first make sure Gazebo-classic is running (double-click on the desktop icon
or use the terminal).

In a terminal, navigate to the aforementioned `haptix-comm/octave` folder and
type `octave hxs_example.m`.

Watch the Gazebo-classic window as the script runs through each example API call.

## The code explained

### `hxs_sim_info`
Retrieves simulation information, which includes the current camera transform
and a list of all models in the simulation. This API call is a good starting
point for iterating through every model in the simulation.

~~~
info = hxs_sim_info();
~~~

In this example, we use the result `info` struct to iterate through every model
in the world, displaying the names of all models, links, and joints.
In this loop, we could also query the links and joints for their current dynamic
state, such as position, velocity, and internal/external forces/torques.

### `hxs_camera_transform` and `hxs_set_camera_transform`
Get and set the position and orientation of the simulation camera angle.

<include lang='matlab' from="/% Get the user camera pose/" to="/hxs_set_camera_transform\(new_tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>

Here, we store the current camera transform in a struct `tx`, translate and
rotate it, and set the new camera transform.

### `hxs_model_color` and `hxs_set_model_color`
Get and set the color of an object as a (red, green, blue, alpha) 4-tuple,
where alpha represents the transparency of the object. An object with an alpha
value of 0 is fully transparent (invisible), while an alpha value of 1 means
the object is fully opaque (solid).

<include lang='matlab' from="/% Change the table color./" to="/hxs_model_color\('table'\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>

Here, we set the color of the table from red to green to blue.

### `hxs_contacts`
Get points at which one model is contacting other models.

<include lang='matlab' from="/% Get contact information/" to="/hxs_contacts\('wood_cube_5cm'\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>

This example retrieves the contacts struct and then traverses it to print out each
contact point for the wooden cube. Expect to see four contact points, one on each
corner of the face of the cube that contacts the "lid" link of the wooden case.

### `hxs_linear_velocity` and `hxs_set_linear_velocity`
Get and set the linear velocity of the model. The linear velocity is a 3-vector
representing the (x, y, z) components of the model's velocity in meters per second.

~~~
vel = hxs_linear_velocity('wood_cube_5cm');
~~~

When the velocity is retrieved in this way, it represents the velocity of the
model's canonical link. For a single-link object like the wood cube, this is
the velocity of the whole model. But for the robotic arm, this is the velocity
of the forearm link, which is established as the canonical link in SDF.

<include lang='matlab' from="/% Move by setting linear velocity/" to="/\[-0.5; 0; 0\]\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>

When we set the velocity of the model in this way, the API will set the velocity
of each link in the model.

### `hxs_angular_velocity` and `hxs_set_angular_velocity`
Get and set the angular velocity of a model. The angular velocity is a 3-vector
representing the velocity about the (x, y, z) axes in radians per second.

~~~
vel = hxs_angular_velocity('wood_cube_5cm');
~~~
This function will return the angular velocity of the model's canonical link.

<include lang='matlab' from="/% Move by setting angular velocity/" to="/, \[0; 0; 100\]\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>
For single-link models, `hxs_set_angular_velocity` will set the angular velocity of the model
relative to the center of mass of the link.

### `hxs_apply_force`, `hxs_apply_torque`, and `hxs_apply_wrench`
The `hxs_apply_X` functions take a model, a link, a vector or a struct representing
the force, torque, or wrench to apply, and a duration to apply the force in seconds.

If duration is "0", then the force, torque, or wrench will have a persistent duration.

~~~
hxs_apply_force('wood_cube_5cm', 'link', [-1.0; 0; 0], 0.2);
~~~
This function applies a force of 1 Newton in the negative X direction on the wood
cube model for 0.2 seconds.

~~~
hxs_apply_torque('wood_cube_5cm', 'link', [0; 0; 0.1], 0.1)
~~~
This function applies a torque of 0.1 Newton-meters about the Z axis on the wood
cube model for 0.1 seconds.

<include lang='matlab' from="/% Apply force and torque at the same time./" to="/wrench, 0.1\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>
This function applies a wrench, which is a simultaneous force and torque. The wrench
has a force component of 1 Newton in the positive Z direction, and a torque
component  of 0.1 Newton-meters about the Z axis.

### `hxs_model_gravity_mode` and `hxs_set_model_gravity_mode`
Get and set the gravity mode of the model. If the gravity mode is 1, then the
model will experience gravity (by default, 9.81 meters/second^2 in the negative Z
direction). If the gravity mode is 0, the model will not experience gravity and will
float in the air unless disturbed by external forces.

<include lang='matlab' from="/\% Check gravity mode on wooden cube/" to="/hxs_set_model_gravity_mode\('wood_cube_5cm', gravity_mode\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>
This example retrieves the current gravity mode, which is 1 by default. It then
turns gravity off and nudges it upward. If gravity acted on the cube, it would fall back
towards the table, but since gravity was turned off it will float.

### `hxs_model_transform` and `hxs_set_model_transform`
Get and set the transform of the model. This is defined as the current transform of the
canonical link (usually the center of mass for single-link models).

<include lang='matlab' from="/\% Get the pose of the cube/" to="/hxs_set_model_transform\('wood_cube_5cm', tx\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>
This example gets the transform of the cube and then modifies it to a different location
and orientation.

#### Setting the transform of the arm
You can also set the position and orientation of the robotic arm:

<include lang='matlab' from="/\% Set the position of the arm/" to="/arm_tx\)/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>

These commands will be overridden by motion tracking technology such as the
Optitrack. If you want them to take effect, make sure motion tracking is paused or
disabled.

### `hxs_model_collide_mode` and `hxs_set_model_collide_mode`
Get and set the collide mode of the model. There are three possible collide modes:

* `hxsCOLLIDE`: The default collide mode. The model will collide with other models
set to this collide mode.
* `hxsNOCOLLIDE`: The model will not collide with anything. It will pass through
other models, even if the other models are set to `hxsCOLLIDE`. `hxs_contacts`
will not detect contacts when this model pass through other models.
* `hxsDETECTIONONLY`: The model will pass through other models and contact forces
will be ignored, like `hxsNOCOLLIDE`. However, collisions will still be detected
by `hxs_contacts`. This means that if the `hxsDETECTIONONLY` model passes through
another model, `hxs_contacts` will have contact information for the models at
that timestep (unless the model has `hxsNOCOLLIDE` set).

For most applications, we recommend either `hxsCOLLIDE` or `hxsDETECTIONONLY`.

<include lang='matlab' from="/\% Check collide mode on the cube/" to="/hxs_set_model_collide_mode\('wood_cube_5cm', collide_mode\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>

In this example, we check the cube's collide mode, which is set to the default
`hxsCOLLIDE`. We then set it to 0, which corresponds to `hxsNOCOLLIDE`, causing it
to drop through the table. We then set it back to the original collide mode.

### `hxs_add_model` and `hxs_remove_model`
`hxs_add_model` adds a new model to the scene based on a complete SDF description.
You can also specify a new name for the model and an initial position and orientation.

To learn how to create a model in SDF, refer to the
[Gazebo-classic model building tutorials](/tutorials?cat=build_robot)
or learn how to use the [model editor](/tutorials?cat=model_editor_top).

`hxs_remove_model` removes the model with the matching name from the world. Note that
`hxs_remove_model` will return an error if it tries to remove a model that does
not exist.

<include lang='matlab' from="/\% Define a new model./" to="/hxs_remove_model\('green_cricket_ball'\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>
This example takes a hardcoded SDF string and creates a new `green_cricket_ball` model.
The model appears in the global coordinate frame at position (0, 0, 5), which means that
it drops onto the wooden case. We apply a torque to show that the model reacts to
external disturbances, then remove it from the world.

### `hxs_set_model_joint_state`
For models with one or more joints, this function takes a model and the name of one of its joints
and two scalar values. It then sets the position and velocity of the specified joint in the model
based on the scalar inputs.

<include lang='matlab' from="/\% Set the state of a wrist joint/" to="/'wristy', 0.5, 0.0\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>

In this example, we try to set the state of the robotic arm's wrist joint.
The joint will snap back to its original position because the motors in the arm
are actively controlling the joints. To properly set the wrist joint, use `hx_update`
(described in [another tutorial](/tutorials?tut=haptix_matlab&cat=haptix)).

### `hxs_reset`
Reset the world. If argument "0" is passed to reset, the robotic arm and the viewpoint
will not reset. If a non-zero argument is passed to reset, everything in the world will
reset to its original position.

<include lang='matlab' from="/\% Move the camera/" to="/hxs_reset\(1\);/" src='http://bitbucket.org/osrf/haptix-comm/raw/default/matlab/hxs_example.m'/>
