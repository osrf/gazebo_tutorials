# Introduction
This tutorial describes how to use a force/torque sensor on a joint.
This sensor publishes force and torque readings to a topic.


# Quick Start
This section gives a quick view of what a force/torque sensor does.

## Steps to see the sensor in action

### Create a world with a force/torque sensor
Save this world as `force_torque_tutorial.world`

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/force_torque_sensor/files/force_torque_tutorial.world' />

### Launch the  world
Launch it with:

```
gazebo --verbose force_torque_tutorial.world
```

### Open up the topic view to the force/torque sensor's output topic
In a new terminal open the topic viewer with the following command:

```
gz topic --view /gazebo/default/model_1/joint_01/force_torque/wrench
```


### Apply forces and torques in gazebo
[Apply a force](tutorials?tut=apply_force_torque) to `link_1` of 500 N in the +Y direction.
Observe the output in the topic viewer window.


## Understanding the example world

### At the start of the world
[[file:files/force_torque_demo.png|480px]]

This world has one link and one joint.
The link is a sphere with a mass of 10 kg offset 1.5 m from the joint.
The joint connects the sphere to the world, allowing rotation on its X axis.
At the start the sphere balanced at the top of the joint, floating above the ground plane.
There is no torque on the joint.
The force on the joint comes from gravity.

```
forceJointZ = mass * g
            = 10 kg * -9.8 m/s^2
            = -98 N
```

### After applying a force to the link
[[file:files/force_torque_toppled.png|480px]]

Applying a force to `link_2` causes it to topple over and rest at a 90 degree angle, the limit of the joint.
The joint's +Y axis points towards the ground plane.
The gravity acting on the block is applying a torque about the X axis.

[[file:files/force_torque_toppled_diagram.png|480px]]

The mass supported by the joint remains the same, so the magnitude of the force is the same.
THe direction of the force changes, it becomes +98 N on the Y axis.
The mass of the sphere is positioned 1.5 m away on the joint's Z axis.
The torque about the X axis is:

```
torqueJoint01_x = r X R
                = ||r|| * ||F|| * sin(theta)
                = distanceZ * (massLink2 * g) * sin(theta)
                = 1.5 m * (10 kg * 9.8 m/s^2) * sin(-90)
                = -147 Nm
```

**Note: Readings near joint limits may jump depending on physics engine parameters. See [#2209](https://bitbucket.org/osrf/gazebo/issues/2209)**

# Understanding the Force/Torque Sensor

## SDF parameters

### Generic parameters
All sensors have a common set of parameters in the [SDF sensor schema](http://sdformat.org/spec?ver=1.6&elem=sensor).

#### `<always_on>`
If `true` the sensor will always measure force and torque.
If `false` the sensor will only update if there is a subscriber connected to the sensor's topic.
This setting is important when accessing the sensor through code.
Calls to `ForceTorqueSensor::Torque()` or `ForceTorqueSensor::Force()` will return stale data if there are no subscribers.
This can be detected by checking if `IsActive()` returns false.
Code can make a sensor update with no subscribers by calling `SetActive(true)`.

#### `<update_rate>`
This is the rate in Hz that the sensor should update itself.
It is the number of messages that will be published by this sensor per simulated second.

#### `<visualize>`
There is no visualization for this sensor.
This setting has no effect.

#### `<topic>`
The force/torque sensor does not currently support this parameter.

#### `<frame>`
The force/torque sensor does not currently support this parameter as it is described in the SDF specification.

#### `<pose>`
A string given as `x y z r p y` that describes the location of the origin of the sensor frame.

### Force/Torque specific parameters
A force/torque sensor is created by adding `<sensor>` tag with the attribute `type` set to `force_torque`.
There are two additional parameters that can be set.

```
<sensor name="my_cool_sensor" type="force_torque">
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
</sensor>
```

#### `<frame>`
The value of this element may be one of: `child`, `parent`, or `sensor`.
It is the frame in which the forces and torques should be expressed.
The values `parent` and `child` refer to the parent or child links of the joint.
The value `sensor` means the measurement is rotated by the rotation component of the `<pose>` of this sensor.
The translation component of the pose has no effect.

Regardless, the torque is always a force applied at a distance from the origin of the joint.

#### `<measure_direction>`
This is the direction of the measurement.
Try changing the example above to `parent_to_child`.
When toppled it reports a force of -98 N on the Y axis and a torque of +147 Nm about the X axis.
This is the same magnitude as before but opposite direction.

## Adding a force/torque sensor to a link
While the SDF schema allows a `<sensor>` tag to be placed on either a link or a joint, the force/torque sensor only works on joints.
If the sensor is added to a link, running gazebo with `--verbose` shows the following error:

```
[Err] [Link.cc:114] A link cannot load a [force_torque] sensor.
```
