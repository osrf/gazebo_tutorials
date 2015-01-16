# Overview
This tutorial will teach you how to how to create your own world with the Atlas robot.

## Setup
We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim).

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

## Modify an empty world
Open gazebo with no command line arguments:

~~~
gazebo
~~~

You will see a blank world.

Now, add some simple shapes to the environment using the icons in the toolbar. You can add boxes, spheres or cylinders.

[[file:files/shapes_toolbar.png|640px]]

You can even add pre-existing models to the environment using the "Insert Models" tab:

[[picture]]

When you're done, save the world by clicking "File" then "Save World As". You can also use the keyboard shortcut, CTRL-SHIFT-S. Choose a filename ending in .world. When you are finished, exit Gazebo.

## Add Atlas to the .world
Now, open the newly saved .world file in your favorite text editor.

Copy/paste the following block into the file. Make sure the block is inside of the <world></world> element.

~~~
    <plugin filename="libVRCPlugin.so" name="vrc_plugin">
      <atlas>
        <model_name>atlas</model_name>
        <pin_link>utorso</pin_link>
      </atlas>
      <drc_vehicle>
        <model_name>golf_cart</model_name>
        <seat_link>chassis</seat_link>
      </drc_vehicle>
      <drc_fire_hose>
        <fire_hose_model>fire_hose</fire_hose_model>
        <coupling_link>coupling</coupling_link>
        <standpipe_model>standpipe</standpipe_model>
        <spout_link>standpipe</spout_link>
        <thread_pitch>-1000</thread_pitch>
        <coupling_relative_pose>1.17038e-05 -0.125623 0.35 -0.0412152 -1.57078 1.61199</coupling_relative_pose>
      </drc_fire_hose>
    </plugin>
    <plugin filename="libVRCScoringPlugin.so" name="vrc_scoring" />
~~~

## 
