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
on="1.5">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <plugin name="SimEvents" filename="libSimEventsPlugin.so">

    <!-- Angle: must be in range -PI to PI  -->
    <event>
      <name>joint_angle</name>
      <type>joint</type>
      <model>revoluter</model>
      <joint>joint</joint>
      <range>
        <type>normalized_angle</type>
        <min>3</min>
        <max>3.3</max>
      </range>
    </event>

    <!-- Velocity -->
    <event>
      <name>joint_velocity</name>
      <type>joint</type>
      <model>revoluter</model>
      <joint>joint</joint>
      <range>
        <type>velocity</type>
        <min>3</min>
        <max>3.3</max>
      </range>
    </event>


    <!-- Applied Force -->
    <event>
      <name>joint_force</name>
      <type>joint</type>
      <model>revoluter</model>
      <joint>joint</joint>
        <type>applied_force</type>
        <range>
          <type>applied_force</type>
          <min>3</min>
          <max>3.3</max>
        </range>
    </event>

    <!-- Close the plugin element  -->
    </plugin>

    <model name='revoluter'>
        <pose>0.5 0.5 0.5 0 0 0</pose>
        <link name='base'>
        <pose>0 0 0 0 -0 0</pose>
          <visual name='visual'>
            <geometry>
              <box>
                <size>0.5 1 1</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Orange</name>
              </script>
            </material>
          </visual>
          <collision name='collision'>
            <geometry>
              <box>
                <size>0.5 1 1</size>
              </box>
            </geometry>
          </collision>
        </link>
        <link name='needle'>
          <pose>0.25 0 -0.2 0 -0 0</pose>
            <visual name='visual'>
              <geometry>
                <cylinder>
                  <radius>0.05</radius>
                  <length>0.5</length>
                </cylinder>
              </geometry>
            </visual>

          <collision name='collision'>
            <geometry>
              <cylinder>
                <radius>0.05</radius>
                <length>0.5</length>
              </cylinder>
            </geometry>
           </collision>
        </link>

        <joint name='joint' type='revolute'>
          <parent>needle</parent>
          <child>base</child>

          <pose>0 0 0 0 0 0</pose>
          <axis>
            <xyz>1 0 0</xyz>
          </axis>
        </joint>

        <!-- this joint keeps the model from moving around -->
        <joint name ='fix' type='fixed'>
          <parent>world</parent>
          <child>base</child>
        </joint>

    </model>

  </world>
</sdf>

~~~


