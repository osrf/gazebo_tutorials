# Overview

The [SimEventsPlugin](http://gazebosim.org/api/code/dev/classgazebo_1_1SimEventsPlugin.html) contains multiple components, one of which is the
[JointEvent](http://gazebosim.org/api/code/dev/classgazebo_1_1JointEventSource.html).
The JointEvent will send a message on the `sim_events` topic when a joint's position (or velocity, or applied force) enters or leaves a specified range.

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

<include lang='sdf' src='./joint_event.world'/>

~~~
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 0</gravity>
    </physics>
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
      <joint>my_joint</joint>
      <range>
        <type>normalized_angle</type>
        <min>1.55</min>
        <max>1.70</max>
      </range>
    </event>

    <!-- Velocity -->
    <event>
      <name>joint_velocity</name>
      <type>joint</type>
      <model>revoluter</model>
      <joint>my_joint</joint>
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
      <joint>my_joint</joint>
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
          <pose>0.25 0 0 0 -0 0</pose>
          <visual name='visual'>
            <pose>0 0 -0.2 0 -0 0</pose>
            <geometry>
              <cylinder>
                <radius>0.05</radius>
                <length>0.5</length>
              </cylinder>
            </geometry>
          </visual>

          <collision name='collision'>
            <pose>0 0 -0.2 0 -0 0</pose>
            <geometry>
              <cylinder>
                <radius>0.05</radius>
                <length>0.5</length>
              </cylinder>
            </geometry>
           </collision>
        </link>

        <joint name='my_joint' type='revolute'>
          <parent>base</parent>
          <child>needle</child>

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

## Position

As the joint named `my_joint` enters or exits the position range between 1.55 and 1.70 radians, a [`gazebo::msgs::SimEvents` message](https://bitbucket.org/osrf/gazebo/src/572e57088a6fe24e316ce8be15e3fac54057649b/plugins/events/SimEventsPlugin.cc?at=default&fileviewer=file-view-default#SimEventsPlugin.cc-74) is published over the topic `/gazebo/sim_events`.

~~~
    <!-- Position -->
    <event>
      <name>joint_angle</name>
      <type>joint</type>
      <model>revoluter</model>
      <joint>joint</joint>
      <range>
        <type>normalized_angle</type>
        <min>1.55</min>
        <max>1.70</max>
      </range>
    </event>
~~~

## Velocity

As the joint named `my_joint` enters or exits the velocity range between 3 and 3.3 rad/s, a [`gazebo::msgs::SimEvents` message](https://bitbucket.org/osrf/gazebo/src/572e57088a6fe24e316ce8be15e3fac54057649b/plugins/events/SimEventsPlugin.cc?at=default&fileviewer=file-view-default#SimEventsPlugin.cc-74) is published over the topic `/gazebo/sim_events`.

~~~
    <!-- Velocity -->
    <event>
      <name>joint_velocity</name>
      <type>joint</type>
      <model>revoluter</model>
      <joint>my_joint</joint>
      <range>
        <type>velocity</type>
        <min>3</min>
        <max>3.3</max>
      </range>
    </event>
~~~

## Applied force

As the force applied by user on the joint named `my_joint` enters or exits the effort range between 3 and 3.3 Nm, a [`gazebo::msgs::SimEvents` message](https://bitbucket.org/osrf/gazebo/src/572e57088a6fe24e316ce8be15e3fac54057649b/plugins/events/SimEventsPlugin.cc?at=default&fileviewer=file-view-default#SimEventsPlugin.cc-74) is published over the topic `/gazebo/sim_events`.

~~~
    <!-- Applied Force -->
    <event>
      <name>joint_force</name>
      <type>joint</type>
      <model>revoluter</model>
      <joint>my_joint</joint>
      <type>applied_force</type>
      <range>
        <type>applied_force</type>
        <min>3</min>
        <max>3.3</max>
      </range>
    </event>
~~~

# Running simulation with joint events plugin

Launch Gazebo in a terminal with the following  command

~~~
gazebo --verbose joint_event.world
~~~

The world should have a model that looks like this:

[[file:./joint_test_world.png|640px]]

Now, we need to see messages that are generated by Gazebo. More specifically, the following events will trigger a `/gazebo/sim_events` message whenever joint velocity is between 3 and 3.3 rad/s, or the force applied to the joint is between 3 and 3.3 Nm.

1. Open the Topic Visualization in the Window menu.
1. Select the `/gazebo/sim_events` topic in the topic selector. This should open a topic window.

[[file:./topic_visualizer.png|640x]]

1. Tip: Use the context menu to keep the topic window in view `Always On Top`
1. Open the link context menu by right clicking on the `needle` link in the object tree in the left hand side panel:

[[file:./apply_force.png|640x]]

1. Access the `Apply Force/Torque` menu by right clicking the `needle` link:

[[file:./apply_force_torque.png|640px]]

1. Using the `Apply Force/Torque` dialog to apply a large torque (10,000 Nm) about the x-axis of the `needle` link, hit  `Apply Torque` button once only. The white link should start rotating and the topic visualizer should start scrolling with new events:

[[file:./sim_event_topic_response.png|640px]]

The end result should look like below:

<iframe width="640" height="480" src="https://www.youtube.com/embed/Dh_YF8JAbBE" frameborder="0" allowfullscreen></iframe>

As the joint rotates into user specified angular region, a sim events message is published. And as the joint rotates out of the user specified region, another message is published. Similarly joint velocity and joint force are monitored by the `joint_velocity` and `joint_velocity` SimEvents blocks.
