# Setting Velocity on Links And Joints
This tutorial will describe how to programatically set velocities on Joints and Links in Gazebo 7.
This is a common task done in a custom [plugin](tutorials?cat=plugins).

# Examples
[Downloaded an example plugin here](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/).
Follow these steps to build and run the plugin.

```
cd Downloads/set_vel_plugin/
mkdir build
cd build
cmake ..
make
GAZEBO_PLUGIN_PATH=. gazebo --pause --verbose ../set_velocity.world
```

Finally unpause the world to see everything move.
All of the methods used to set velocity are explained below.

# Methods
There are three methods to set velocity:

1. Set Instantaneous Velocity
2. Configure a joint motor (ODE only)
3. Create a PID controller

### Set Velocity Instantaneously

**Advantages**

* Supported on all physics engines
* Simple, only one function call
* Object moves at target velocity right away

**Disadvantages**

* Object doesn't "feel" a force accellerating it

All physics engines used by gazebo support setting an instananeous velocity.
Objects move at the target speed without any forces or torques being applied.
This means calls to [`Joint::GetForceTorque()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Joint.html#a85f6b25f1d0d6451a84875c18c57535d), [`Link::GetWorldForce()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#ab6d63e2c37c0273d1f8fd820d208f894) and [`Link::GetWorldTorque()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#ab4f3ec4a752b81b69198055b525cc026) will not show any additional forces or torques when using these methods.

The object is not constrained to the velocity permanently.
Forces or torques may change the speed of the object after the velocity is set.
It must be set every time step to keep the object at a constant velocity forever.

### Set Velocity With Joint Motors

**Advantages**

* Can reach target velocity in a single update
* Object feels the force that accellerates it
* The exact required force is used, no under or overshooting the target velocity

**Disadvantages**

* ODE only (default physics engine used by gazebo)
* When using to set a link's velocity all other degrees of freedom are locked

Joints motors can be used to reach a velocity by applying the exact required force to a joint.
Gazebo only supports this method when using the ODE physics engine (the default engine).
It relies on the [ODE Joint Motor feature](https://www.ode-wiki.org/wiki/index.php?title=Manual:_Joint_Types_and_Functions#Stops_and_motor_parameters).

### Setting Velocity Using PID Controllers

**Advantages**

* Supported on all physics engines
* Object feels the force that accellerates it
* Does not need to lock other degrees of freedom

**Disadvantages**

* PID gains need to be tuned per object
* Can under shoot or over shoot the target velocity
* It takes multiple updates to reach the target velocity

Another method of setting velocity is to use a [PID controller](https://en.wikipedia.org/wiki/PID_controller) to apply forces and torques.
It works on all physics engines, but requires tuning constants specifically for the object being moved.
Unlike the joint motor method, the other degrees of freedom are not locked while the forces are being applied.

In general the velocity cannot be achieved instantaneously with a PID controller.
It must exist and be active for multiple updates to achive the target velocity.

# Setting Velocity on Joints
This section will show how to use the three methods to set velocity on a joint.

[[file:pictures/set_joint_velocity.gif|400px]]

> **Note**
> Not all joints can be commanded to move at target velocity.
> Revolute, revolute2, prismatic, screw, and universal joints can be set.
> Ball, gearbox, and fixed joints cannot be set using any method described below.
> However, while a gearbox joint velocity cannot be set, the parent or child joint can be set if it is one of the supported joint types.

### Set Joint Velocity Instantaneously
Velocity on joints can be set instantaneously using [`Joint::SetVelocity()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Joint.html#ae32987acf99308e4aca7f2c399f3e731).
The velocity is achieved by moving the child link.
Notice at first the top link (child) on the gray (leftmost) joint moves while the bottom link is stationary.
The momentum of the top link causes the whole gray object to move when the joint limit is hit.

```
          this->model->GetJoint("gray_joint")->SetVelocity(0, 1.0);
```

It takes two parameters: axis, and velocity.
The axis parameter is an index, and it may be 0 or 1.
Zero means the first axis on the joint, and one means the second if applicable.

<style>
table, th,td {padding: 5px;}
</style>
<table border="1">
  <tr>
    <th>Type</th>
    <th>Number of Axes</th>
  </tr>
  <tr>
    <td>prismatic</td>
    <td>1</td>
  </tr>
  <tr>
    <td>revolute</td>
    <td>1</td>
  </tr>
  <tr>
    <td>revolute2</td>
    <td>2</td>
  </tr>
  <tr>
    <td>screw</td>
    <td>1</td>
  </tr>
  <tr>
    <td>universal</td>
    <td>2</td>
  </tr>
</table>

The second parameter is the velocity.
It is meters per second for prismatic joints, and radians per second for all others.

### Set Joint Velocity Using Joint Motors
Configuring a joint motor is done using [`Joint::SetParam()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Joint.html#a48402b4fa13b0209246396c0d726d914).

```
          this->model->GetJoint("orange_joint")->SetParam("fmax", 0, 100.0);
          this->model->GetJoint("orange_joint")->SetParam("vel", 0, 1.0);
```

It accepts three parameters: key, axis, and value.
The key parameter is a string that names the parameter to be changed.
The axis parameter is an index that may be 0 or 1.

> **Note**
> The value parameter must have exactly the right type.
> Joint motors require setting `double` parameters.
> This call will work `joint->SetParam('fmax', 0, 0.0)` while this will have a runtime error `joint->SetParam('fmax', 0, 0)`.

Setting up a joint motor requires requires two calls.
The first call sets the key `vel` to the velocity the joint should travel at.
It is meters per second for prismatic joints and radians per second for all others.
The other call sets the key `fmax`.
It is the maximum force or torque a joint motor may apply during a time step.
Set it larger than the force required to be at the target velocity at the next time step.
Set it smaller to apply a force over many time steps until the velocity is reached.
Stop applying force by setting `fmax` back to zero.

### Set Joint Velocity Using PID controllers
A [PID controller](https://en.wikipedia.org/wiki/PID_controller) can be used to apply forces on the joint axes.
The class [`physics::JointController`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1JointController.html) can manager the PID controllers for you.

<include from="/          this->jointController\.reset/" to="/          this->jointController->SetVelocityTarget\(name, 1.0\);/" src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/src/SetJointVelocityPlugin.cpp"/>

The controller gains must be configured for each object being moved.
The velocity target is meters per second for prismatic joints, and radians per second for all others.
[`JointController::Update()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1JointController.html#aec0783b5a136e042adcc47bae4fe5291) must be called every time step to apply forces

```
          this->jointController->Update();
```

# Setting Velocity on Links
This section will show how to use the three methods to set velocity on a link.

[[file:pictures/set_link_velocity.gif|400px]]

### Set Link Velocity Instantaneously

<include from="/          \/\/ Link velocity instantaneously without applying forces/" to='/          model->GetLink\(\"white_link_2\"\)->SetAngularVel\({1, 0, 0}\);/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/src/SetLinkVelocityPlugin.cpp"/>

Linear velocity on links can be set with [`Link::SetLinearVel()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#a110267b99cacd79cd377ca8619956645).
Angular velocity on links can be set with [`Link::SetAngularVel()`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Link.html#a996d99f2897ebca28979b24b7f23faa1).
Both accept a [three dimensional vector](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1math_1_1Vector3.html) with the target linear velocity.
The velocity must be expressed in the world frame in meters per second or radians per second.

###  Set Link Velocity Using Joint Motors
Joint motors can be used to move links by creating a joint connecting the link to the world.
It is critical that the joints are created when the velocity is to be applied, and deleted afterwards.

Linear velocity can be set by creating a prismatic joint between the world and the link to be moved.
Then a joint moter can be configured as described above.
The link will not rotate or move off the prismatic joint axis until the joint is detached.

<include from="/        \/\/ create prismatic joint with the world as a parent/" to='/        this->joint->SetParam\(\"vel\", 0, magnitude\);/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/include/ode_perfect_linear.hh"/>

Angular velocity can be set by creating a revolute joint between the world and the link to be moved.
The link will only be able to rotate about the revolute joint axis until the joint is detached.

<include from="/        \/\/ create revolute joint with the world as a parent/" to='/        this->joint->SetParam\(\"vel\", 0, magnitude\);/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/include/ode_perfect_angular.hh"/>

Controlling both linear and angular velocity at the same time requires an additional link.
The link should be programmatically created with the same origin as the link whose velocity is to be set.
Create a prismatic joint with the world as the parent and the phantom link as the child.
Then create a revolute joint with the phantom link as the parent and the link to be moved as the child.
Finally create joint motors on both joints to reach the target velocity.

<include from="/        \/\/ Create a phantom link/" to='/        this->revoluteJoint->SetParam\(\"vel\", 0, angularMagnitude\);/' src="http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/include/ode_perfect_velocity.hh"/>

### Set Link Velocity Using PID controllers
A [PID controller](http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1common_1_1PID.html) can be used to set a velocity by appling forces or torques.
This requires tuning constants for each object whose velocity is to be set.

<include from="/      \/\/ Hard coded gains\. Tune these for your own application!/" to='/      double angular_imax = 123456789\.0;/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/include/pid_link.hh' />

Each degree of freedom (x, y, z, roll, pitch, yaw) must have it's own PID controller.
Fewer controllers can be used if it is permissable for the link to move freely on some degrees of freedom.
For example, setting a translational velocity while allowing the object to rotate requries only 3 PID controllers: x, y, z.

<include from="/      \/\/ Add a PID controller for each DoF/" to='/      }/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/include/pid_link.hh' />

Every physics update the error between the actual velocity and the target velocity needs to be given to the controller.

<include from="/      \/\/ Calculate the error between actual and target velocity/" to='/      worldTorque\.z = this->controllers\[5\]\.Update\(angularError\.z, dt\);/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/include/pid_link.hh' />

The controllers will output forces and torques that should be applied to the link to correct for the current error.

<include from="/      \/\/ Add those forces to the body/" to='/      this->link->AddTorque\(worldTorque\);/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/set_velocity/examples/set_vel_plugin/include/pid_link.hh' />

The object will move at the desired velocity.
The amount of velocity error depends on the PID gains chosen.
Tune these until you get acceptable performance.
