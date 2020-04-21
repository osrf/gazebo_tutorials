# Overview

The [SimEventsPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1SimEventsPlugin.html) contains multiple components, one of which is the
[OccupiedEvent](http://gazebosim.org/api/code/dev/classgazebo_1_1OccupiedEventSource.html). The OccupiedEvent will send a message on a topic whenever a model
occupies a specified three dimensional region.

# Usage and Example

The OccupiedEvent component is instantiated through the
`libSimEventsPlugin.so` and relies on at least one `<region>` and `<event>`,
  where the `<event>` has a `<type>` of `occupied`.

In the following example, a region is specified as a box volume with
a minimum corner at `0, 0, 0` and a maximum corner at `1, 1, 1` in the world
coordinate frame. The name of this region is `region1`. An `<event>`, of
`<type>` occupied, with the name `region1_event` is also specified that uses
`region1`. When a model occupies `region1` a [string
message](http://gazebosim.org/api/msgs/dev/gz__string_8proto.html) with
a data value of "0"  is sent on the `~/elevator` topic.

~~~
<plugin filename="libSimEventsPlugin.so" name="event_plugin">
  
  <region>
    <name>region1</name>
    <volume>
      <min>0 0 0</min>
      <max>0 1 1</max>
    </volume>
  </region>
  
  <event>
    <name>region1_event</name>
    <type>occupied</type>
    <region>region1</region>
    <topic>~/elevator</topic>
    <msg_data>0</msg_data>
  </event>
  
</plugin>
~~~

Multiple regions and events may be specified. See the
[worlds/elevator.world](https://github.com/osrf/gazebo/src/default/worlds/elevator.world) for a longer example.
