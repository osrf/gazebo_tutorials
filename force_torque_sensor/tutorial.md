# Introduction
This tutorial describes how to use a force/torque sensor.
TODO This is a sensor that can be added to a link or a joint.
It publishes force and torque readings to a topic.


# Quick Start
This section gives a quick view of what a force/torque sensor does.

## Steps to see the sensor in action

**Create a world with a force/torque sensor**

Save this world as `force_torque_tutorial.world`

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/force_torque_sensor/files/force_torque_tutorial.world' />

**Launch the  world**

Launch it with:

```
gazebo --verbose force_torque_tutorial.world
```

**Open up the topic view to the force/torque sensor's output topic**

In a new terminal open the topic viewer with the following command:

```
gz topic --view /gazebo/default/model_1/joint_01/force_torque/wrench
```


**Apply forces and torques in gazebo**

[Apply a force](tutorials?tut=apply_force_torque) to `link_1` of 500 N in the +Y direction.
Observe the output in the topic viewer window.


## Understanding the example world

**At the start of the world**

[[file:files/force_torque_demo.png|480px]]

The demo world has one link and one joints.
The link is a sphere with a mass of 10 kg offset 1.5 m from the joint.
`joint_01` connects a sphere to the word, allowing rotation on its X axis.
At the start the sphere balanced at the top of the joint.
The sensor on `joint_01` reports the following:

[[file:files/force_torque_topic_window.png|320px]]

This joint is keeping the link floating above the ground plane.
The force on the joint comes from gravity.

```
forceJointZ = mass * g
            = 10 kg * -9.8 m/s^2
            = -98 N
```

**After applying a force to the link**

[[file:files/force_torque_toppled.png|480px]]

Applying a force to `link_2` causes it to topple over and rest at a 90 degree angle.
`joint_01` has rotated so that the +Y axis points towards the ground plane.
The block now applies a torque in the -X direction.

[[file:files/force_torque_toppled_diagram.png|480px]]

The mass supported by the joint remains the same, so the force should be +98 N on the Y axis.
The mass of the sphere is positioned 1.5 m away from `joint_01` on the joint's Z axis.
The torque about the X axis is:

```
torqueJoint01_x = r X R
                = ||r|| * ||F|| * sin(theta)
                = distance * (massLink2 * g) * sin(theta)
                = 1.5 m * (10 kg * 9.8 m/s^2) * sin(-90)
                = -147 Nm
```


# Understanding the Force/Torque Sensor



