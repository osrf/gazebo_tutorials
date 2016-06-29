# Overview

This tutorial demonstrates a new custom handsim world created for simulation based
manipulation dexterity testing inspired by a paper titled
["The strength-dexterity test as a measure of dynamic pinch performance"]
(http://www.asbweb.org/conferences/2001/pdf/012.pdf)
written by Valero-Cuevas et. al. in 2003.

For this tutorial, we assume that you have already completed the
[HAPTIX handsim installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix)
and we strongly recommend that you have completed the
[simulation world API tutorials](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix).

# Running the Simulation Example

To start Gazebo handsim scoring plugin example, run gazebo in terminal:

~~~
gazebo --verbose worlds/luke_hand.world
~~~

By default it brings up the desktop world with the [Luke Hand model](http://gazebosim.org/tutorials?tut=haptix_luke_hand&cat=haptix).

Integrated in this world is a spring compression test that uses
the SimEventsPlugin to keep track of the status of aforementioned task completion.

## Example Video

Below is an example of the custom world as tele-operated by keyboard and spacenav options:

<iframe width="600" height="450" src="https://www.youtube.com/watch?v=pj6mMQOjTPM" frameborder="0" allowfullscreen></iframe>

In the video, there are three little task completion indicator dots at the lower right hand side of the hand visualization GUI.
The left most circular dot indicates correct compression without buckling, green if compressed without buckling, red if uncompressed or buckled.
The middle circular dot indicates compression hold time, it fades from white to green as the spring is compressed correctly and held for 3 seconds.
The right most circular dot indicates task success as it turns from white to green.

Initially, the three circles are red-white-white, indicating the spring is undisturbed.
When the spring is compressed sufficiently (compression length between 1 to 10cm), and
the springs are relatively straight (torsional spring joint at the middle of the spring exhibits < 0.1 radians in flex), the first circle turns green.
A timer is started for this successful unbuckled compression, and the second circle fades from white to green.
When the timer reaches 3 seconds, the second and third circular indicators turn green, and
  the program considers this a successful test trial.

# Relevant Documentations

In the [luke_hand.world](https://bitbucket.org/osrf/handsim/src/8fe03d4d113659c1cc04ea788792b1b7e995c267/worlds/luke_hand.world?at=default&fileviewer=file-view-default#luke_hand.world-4),
  a new `libSimEventsPlugin.so` plugin block has been added:

~~~
    <plugin name="SimEvents" filename="libSimEventsPlugin.so">
      <!-- spring 3 -->
      <event>
        <name>compressed_bottom</name>
        <type>joint</type>
        <model>spring_buckle_test_3</model>
        <joint>joint_bottom_1</joint>
        <range>
          <type>position</type>
          <min>-0.10</min>
          <max>-0.01</max>
        </range>
      </event>
      <event>
        <name>buckled_x</name>
        <type>joint</type>
        <model>spring_buckle_test_3</model>
        <joint>joint_1_2</joint>
        <range>
          <type>normalized_angle</type>
          <min>-0.1</min>
          <max> 0.1</max>
        </range>
      </event>
      <event>
        <name>buckled_y</name>
        <type>joint</type>
        <model>spring_buckle_test_3</model>
        <joint>joint_2_3</joint>
        <range>
          <type>normalized_angle</type>
          <min>-0.1</min>
          <max> 0.1</max>
        </range>
      </event>
      <!-- spring 2 -->
      <event>
        <name>compressed_bottom</name>
        <type>joint</type>
        <model>spring_buckle_test_2</model>
        <joint>joint_bottom_1</joint>
        <range>
          <type>position</type>
          <min>-0.10</min>
          <max>-0.01</max>
        </range>
      </event>
      <event>
        <name>buckled_x</name>
        <type>joint</type>
        <model>spring_buckle_test_2</model>
        <joint>joint_1_2</joint>
        <range>
          <type>normalized_angle</type>
          <min>-0.1</min>
          <max> 0.1</max>
        </range>
      </event>
      <event>
        <name>buckled_y</name>
        <type>joint</type>
        <model>spring_buckle_test_2</model>
        <joint>joint_2_3</joint>
        <range>
          <type>normalized_angle</type>
          <min>-0.1</min>
          <max> 0.1</max>
        </range>
      </event>
      <!-- spring 1 -->
      <event>
        <name>compressed_bottom</name>
        <type>joint</type>
        <model>spring_buckle_test_1</model>
        <joint>joint_bottom_1</joint>
        <range>
          <type>position</type>
          <min>-0.10</min>
          <max>-0.01</max>
        </range>
      </event>
      <event>
        <name>buckled_x</name>
        <type>joint</type>
        <model>spring_buckle_test_1</model>
        <joint>joint_1_2</joint>
        <range>
          <type>normalized_angle</type>
          <min>-0.1</min>
          <max> 0.1</max>
        </range>
      </event>
      <event>
        <name>buckled_y</name>
        <type>joint</type>
        <model>spring_buckle_test_1</model>
        <joint>joint_2_3</joint>
        <range>
          <type>normalized_angle</type>
          <min>-0.1</min>
          <max> 0.1</max>
        </range>
      </event>
    </plugin>
~~~

(For reference, [here are the documentation for SDF format](http://www.sdformat.org/) and here are [some basic tutorials on using SDF to build simulation worlds and models](http://gazebosim.org/tutorials?cat=build_world)).

And the accompanying source code blocks that interprets the results of the `SimEventsPlugin` data can be found in
[HaptixGUIPlugin.cc](https://bitbucket.org/osrf/handsim/src/default/src/HaptixGUIPlugin.cc).

This block:

<include lang='c' src='https://bitbucket.org/osrf/handsim/raw/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc' from='/void HaptixGUIPlugin::ScoringUpdate\(\)/' to='/void HaptixGUIPlugin::PollTracking/' /> 

updates the GUI visual to keep track of the task completion state by changing colors of three circles to the lower right hand
side of the hand diagram used for contact sensor visualization.
Note the `HaptixGUIPlugin::ScoringUpdate` function is spawned in its own thread [here](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-718).
And the `HaptixGUIPlugin::OnSimEvents` function [referenced here](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-1560) [subscribes to state updates](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-463) published by the `SimEventsPlugin`.

For reference, the Gazebo SimEvents API documentation can be found
[here](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1SimEventsPlugin.html).
This plugin subscribes to gazebo topic `/gazebo/sim_events` to monitor for changes in the simulated spring joints.

