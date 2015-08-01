# Overview

The [SimEventsPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1SimEventsPlugin.html) contains multiple components, one of which is the
[JointEvent](http://gazebosim.org/api/code/dev/classgazebo_1_1JointEventSource.html). The JointEvent will send a message on the sim_eventstopic when a joint's position (or velocity, or applied force) enters or leaves a specified range.

# Usage and Example

The JointEvent component is instantiated through the
`libSimEventsPlugin.so` and relies on at least one `<region>` and `<event>`,
  where the `<event>` has a `<type>` of `joint`.

The following world features a model with a revolute joint.

~~~
mkdir joint_event
cd joint_event
~~~

Copy the world code and save it as joint_event.world

<include lang='sdf' src='./joint_event_position.world'/>

~~~


~~~

Multiple regions and events may be specified. See the
[worlds/elevator.world](https://bitbucket.org/osrf/gazebo/src/default/worlds/elevator.world) for a longer example.
