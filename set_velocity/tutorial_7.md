# Setting Velocity on Links And Joints
This tutorial will describe how to programatically set velocities on Joints and Links.
This is a common task done in a custom [plugin](tutorials?cat=plugins).

## Supported Joint Types
Not all joints can be commanded to move at target velocity.
Revolute, revolute2, prismatic, screw, and universal joints can be set.
Ball, gearbox, and fixed joints cannot be set using any method described below.
However, while a gearbox joint velocity cannot be set, the parent or child joint can be set if it is one of the supported joint types.

## Set Velocity Instantaneously
All physics engines used by gazebo support setting an instananeous velocity.
This causes objects to move at the given speed without appling any forces or torques to do so.
Calls to [`Joint::GetForceTorque()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Joint.html#a85f6b25f1d0d6451a84875c18c57535d), [`Link::GetWorldForce()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#ab6d63e2c37c0273d1f8fd820d208f894) and [`Link::GetWorldTorque()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#ab4f3ec4a752b81b69198055b525cc026) will not show any additional forces or torques when using these methods.

### Joints
Velocity on joints can be set using [`Joint::SetVelocity()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Joint.html#ae32987acf99308e4aca7f2c399f3e731).
It takes two parameters: axis, and velocity.
The axis parameter is an index, and it may be 0 or 1.
Zero means the first axis on the joint, and one means the second if applicable.

#### Number of axis by joint type

| Type | Number |
|------------|:--------------:|
| revolute2  |        2       |
| prismatic  |        1       |
| screw      |        2       |
| universal  |        2       |
| revolute   |        1       |


The second parameter is the velocity.
It is meters per second for prismatic joints, and radians per second for all others.

### Links
Linear velocity on links can be set with [`Link::SetLinearVel()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#a110267b99cacd79cd377ca8619956645).
It accepts a [three dimensional vector](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1math_1_1Vector3.html) with the target linear velocity.
The velocity must be expressed in the world frame.

Angular velocity on links can be set with [`Link::SetAngularVel()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#a996d99f2897ebca28979b24b7f23faa1).
It also accepts a three dimensional vector with the target angular velocity.
The velocity must be expressed in the world frame.

## Set Velocity With Joint Motors (ODE Only)
Joint velocity can be set by applying the exact required force to a joint.
Gazebo only supports this method when using the ODE physics engine (the default engine).
It relies on the [ODE Joint Motor feature](https://www.ode-wiki.org/wiki/index.php?title=Manual:_Joint_Types_and_Functions#Stops_and_motor_parameters).

Configuring a joint motor is done using [`Joint::SetParam()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Joint.html#a48402b4fa13b0209246396c0d726d914).
It accepts three parameters: key, axis, and value.
The key parameter is a string that names the parameter to be changed.
The axis parameter is an index that may be 0 or 1.
The value parameter must have exactly the right type; the compiler won't be able to add an implicit conversion.
Joint motors require setting `double` parameters.
This call will work `joint->SetParam('fmax', 0, 0.0)`, while this won't compile `joint->SetParam('fmax', 0, 0)`.

### Joints
Setting up a joint motor requires requires two calls to [`Joint::SetParam()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Joint.html#a48402b4fa13b0209246396c0d726d914).
The first call sets the key `vel`.
This is the velocity the joint should travel at.
The other call sets the key `fmax`.
This is the maximum force or torque a joint motor may apply to get the joint to the target velocity.
If it is larger than the force required, the link will be at the target velocity at the next time step.
Otherwise the motor will keep applying a force over many time steps until the velocity is reached.
The motor will continue to keep the joint at the target velocity until `fmax` is set back to zero.

### Links
Joint motors can be used to move links at a target velocity by creating a joint connecte the link and to the world.
It is critical that the joints are created when the velocity is to be applied, and deleted afterwards.

Linear velocity can be set by creating a prismatic joint between the world and the link to be moved.
The prismatic joint constrains rotation and two degrees of translation, so the link will not rotate or move off the prismatic joint axis while the velocity is applied to it.
The following example shows a class that can set the linear velocity of a link using this method.

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/ode_perfect_linear.hh' />

Angular velocity can be set by creating a revolute joint between the world and the link to be moved.
The revolute joint contrains translation and two degrees of rotation, so the link will only be able to rotate about the revolute joint's axis while the velocity is aplied.
The following example shows a class that can set the angular velocity of a link using this method.

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/ode_perfect_angular.hh' />

It is possible to control angular and linear velocity at the same time using a combination of these joints and an additional programatically created link.
Implementing this is left to the reader.

## Setting Velocity by Applying Force/Torqe with a PID controller
Another method of setting velocity is to use a [PID controller](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1common_1_1PID.html) to apply forces and torques.
It works on all physics engines, but requires tuning the PID gains for the object being moved.
In general the velocity cannot be achieved instantaneously.
A PID controller must exist for multiple updates to achive the target velocity.
Unlike the joint motor method, the other degrees of freedom are not locked while the forces are being applied.

### Joints
A PID controller be used to apply forces on the joint axis.
The following example can apply a force or torque to one axis on a joint.

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/pid_joint.hh' />

Extending it to support controlling two axis at the same time is left to the reader.

### Links
All 6 degrees of freedom can be set to target velocities simultaneously by using a PID controller for each degree of freedom..
The following example can apply a forces and torques to a link to set a target linear and angular velocity.

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/pid_link.hh' />

Extending it to have separate PID gains for each link, or apply forces and torques relative to the link are exercises left to the reader.
