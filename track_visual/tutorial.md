# Overview

This tutorial demonstrates how you can track a visual with the user camera by
using the SDF `<track_visual>` tag or the GUI.

## Quick start

1. Let's start by creating a directory for this tutorial:

    ~~~
    mkdir ~/tutorial_track_visual
    cd ~/tutorial_track_visual
    ~~~

2. Download this file:
[`track_visual.world`](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/track_visual/files/track_visual.world)
into the current directory. You can use this command:

    ~~~
    wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/track_visual/files/track_visual.world
    ~~~

    You should get this world file:

    <include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/track_visual/files/track_visual.world' />

3. Start gazebo:

    ~~~
    gazebo track_visual.world
    ~~~

    You should see a cube moving randomly around the world's origin.

## The world explained

Let's go further and understand the different elements of the `track_visual.world`.

<include from='/    <gui/' to='/</name>/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/track_visual/files/track_visual.world' />

Within the `track_visual` tag, you can see how to select a visual by specifying a `<name>` tag. The visual will be tracked from startup. The camera will follow the model by changing its orientation.

<include from='/    <static/' to='/</max_dist>/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/track_visual/files/track_visual.world' />

Because `<static>` is set to `false`, the position of the camera may vary but the distance between the camera and the model will depend on the value of the `min_dist` and `max_dist` elements.

Now change `<static>` to `true`:

    ~~~
    <gui>
      <camera name="user_camera">
        <track_visual>
          <name>box</name>
          <static>true</static>
        </track_visual>
      </camera>
    </gui>
    ~~~

Relaunch Gazebo. The position of the camera is now fixed relatively to the model being tracked.

By default, the camera is located 5 meters behind and 1 meter above the model. Try changing its location by specifying a `<xyz>` tag:

    ~~~
    <gui>
      <camera name="user_camera">
        <track_visual>
          <name>box</name>
          <static>true</static>
          <xyz>3 2 5</xyz>
        </track_visual>
      </camera>
    </gui>
    ~~~

Relaunch Gazebo. The camera is now located 3 meters in front, 2 meters to the left, and 5 meters above the tracked model.

Finally, we introduce the `<relative>` tag whose default value is `true`. When the `<static>` tag is `true`, this element specifies whether the coordinates provided by `<xyz>` are relative to the model reference frame or the world reference frame. Try changing the value of the `<relative>` tag:

    ~~~
    <gui>
      <camera name="user_camera">
        <track_visual>
          <name>box</name>
          <static>true</static>
          <relative>false</relative>
          <xyz>0 0 10</xyz>
        </track_visual>
      </camera>
    </gui>
    ~~~

Relaunch Gazebo. The camera now stays at 10 meters above the origin.

To illustrate the last element, we will track a sphere instead of a cube. Change `<name>` to `sphere` and make sure that `<static>` and `<relative>` are set to `true`:

    ~~~
    <gui>
      <camera name="user_camera">
        <track_visual>
          <name>box</name>
          <static>true</static>
          <relative>true</relative>
          <xyz>2 0 1</xyz>
        </track_visual>
      </camera>
    </gui>
    ~~~

Relaunch Gazebo. You should now see the camera following the sphere. But there is one issue: the camera jumps back and forth whenever the sphere makes a half turn. That is because, by default, the camera is configured to inherit the yaw rotation of the tracked visual.

To prevent this behaviour, simply set the `<inherit_yaw>` tag to `false`:

    ~~~
    <gui>
      <camera name="user_camera">
        <track_visual>
          <name>box</name>
          <static>true</static>
          <relative>true</relative>
          <inherit_yaw>false</inherit_yaw>
          <xyz>2 0 1</xyz>
        </track_visual>
      </camera>
    </gui>
    ~~~

Relaunch Gazebo. The camera should now follow the sphere flawlessly.

## Dynamic Reconfigure

The camera properties can be adjusted dynamically within Gazebo.

In the `World` tab, select the `GUI` item. A list of GUI properties will be displayed in the list box below. Click the triangle to expand the properties if needed.

[[file:files/tutorialGUIExpanded.png|257px]]

These properties allow you to dynamically change the parameters defined in `<track_visual>`. They will be applied whenever a model is being followed by the camera.
