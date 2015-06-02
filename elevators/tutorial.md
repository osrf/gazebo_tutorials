# Overview

This tutorial describes how to use elevators in Gazebo. An elevator in
Gazebo consists of a car, that moves up and down, with a door. Primatic joints control the car and door.

# Functionality

The elevator functionality is implemented through an
[ElevatorPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1ElevatorPlugin.html).
This plugin controls two prismatic joints:

1. lift joint: used to raise and lower the elevator,
1. door joint: used to open and close the elevator door.

During the plugin's update cycle, the lift joint will move the elevator to
the requested floor, and the door joint will open or close the door
depending on the elevator's current state. Movement to a particular floor
can be requested by sending a [string message](http://gazebosim.org/api/msgs/dev/gz__string_8proto.html) with the floor number to the
elevator's topic. Specification of these parameters is described in the
following section.

The elevator's control logic is:

1. Wait for a floor request.
1. Close door.
1. Move to requested floor.
1. Open door.
1. Wait for five seconds.
1. Close the door.
1. Go to #1.


# Parameters

The lift joint, door joint, floor height, and control topic are all
specified in SDF. The following is an example:

~~~
<plugin filename="libElevatorPlugin.so" name="elevator_plugin">
  <lift_joint>elevator::lift</lift_joint>
  <door_joint>elevator::door</door_joint>
  <floor_height>3.075</floor_height>
  <topic>~/elevator</topic>
</plugin>
~~~

In this example, an `elevator` model exists elsewhere in the SDF file with
a joint for lifting called `lift`, and a joint for the door called `door`.
The height of each floor is 3.075 meters, and the elevator plugin listens
for floor requests on the `~/elevator` topic.

# Example

A complete [example world](https://bitbucket.org/osrf/gazebo/src/default/worlds/elevator.world) is distributed with Gazebo. You can run this
world using the following command:

~~~
gazebo worlds/elevator.world
~~~

This world has two floors and a single elevator. Place a simple shape on the
landing in front of the elevator to trigger a floor request. This floor
request takes place thanks to the Occupied Region Event, described next.

# Occupied Region Event

The elevator plugin works well the [OccupiedEvent](http://gazebosim.org/api/code/dev/classgazebo_1_1OccupiedEventSource.html) component of the [SimEventPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1SimEventsPlugin.html). The OccupiedEvent sends a message whenever a 3D region becomes occupied with a model. Refer to the [OccupiedEvent tutorial](http://gazebosim.org/tutorials?tut=occupiedevent&cat=plugins) for more information.
