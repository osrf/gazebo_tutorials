# Introduction
This tutorial describes how to use a force/torque sensor.
TODO This is a sensor that can be added to a link or a joint
It publishes force and torque readings to a topic.


# Quick start
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

A window should open that looks like this:

[[file:files/force_torque_topic_window.png|320px]]

**Apply forces and torques in gazebo**

[Apply a force](tutorials?tut=apply_force_torque) to `link_2` of 500 N in the + Y direction.
Observe the output in the topic viewer window.


## Understanding the demo world

**At the start of the world**

[[file:files/force_torque_demo.png|480px]]

The demo world has two links and two joints.
The two links are a sphere and a block, each with a mass of 10 kg.
`joint_01` connects a sphere to the word, allowing rotation on its X axis.
`joint_12` connects a block to the sphere with all degrees of freedom fixed.
At the start the block is balanced perfectly above the sphere.
The sensor on `joint_01` reports the following:

```
wrench {
  force {
    x: 0
    y: 0
    z: -195.99999999999994
  }
  torque {
    x: 0
    y: 0
    z: 0
  }
}
```

This joint is keeping both links floating above the ground plane.
The force of -196 N on the Z axis comes from gravity.

```
forceJoint01_z = massLink1 * g + massLink2 * g
               = 10 kg * -9.8 m/s^2 + 10 kg * -9.8 m/s^2
               = -196 N
```

**After applying 500 N to link_2**

[[file:files/force_torque_toppled.png|480px]]

Applying a force to `link_2` causes the it to topple over and rest at a 90 degree angle.
`joint_01` has rotated so that the +Y axis points towards the ground plane.
The block now applies a torque in the -X direction.

[[file:files/force_torque_toppled_diagram.png|480px]]

The mass supported by the joint remains the same, so the force should be +196 N on the Y axis.
The mass of the block is positioned 1.5 m away from `joint_01` with an angle of -90 degrees.
The torque about the X axis should be:

```
torqueJoint01_x = r X R
                = ||r|| * ||F|| * sin(theta)
                = distance * (massLink2 * g) * sin(theta)
                = 1.5 m * (10 kg * 9.8 m/s^2) * sin(-90)
                = -147 Nm
```

However watching the sensor readings it is clear the force and torque vary wildly.
What's going on?


# Understanding the force_torque sensor



