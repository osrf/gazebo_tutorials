# Overview
This tutorial will teach you how to how to spawn the Atlas robot into your own world.

## Setup
We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim).

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

##
Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on `drcsim_gazebo`:

~~~
cd ~/ros
roscreate-pkg world_create_tutorial drcsim_gazebo
~~~

~~~
roscd world_create_tutorial
~~~
The above command is equivalent to: cd `~/ros/world_create_tutorial`

~~~
mkdir worlds launch
~~~

~~~
roscd drcsim_gazebo
~~~
The above command takes you to the `drcsim_gazebo` directory, which may be different according to your installation. It will be something like `/opt/ros/indigo/share/drcsim_gazebo`. We will copy a file from there to help spawn Atlas:

~~~
cp launch/atlas_no_controllers.launch ~/ros/world_create_tutorial/launch/
~~~

~~~
roscd world_create_tutorial/
~~~

Next, copy and paste this launchfile into `~/ros/world_create_tutorial/launch/atlas.world`, or download the file [here](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_create_atlas_world/files/atlas.launch):

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_create_atlas_world/files/atlas.launch' />

This launchfile is a direct copy of the `atlas.launch` file in `drcsim_gazebo/launch`, except that the `gzworld` argument has been changed to a different file. This file doesn't exist yet, but we will create it in the next step.

If you want to spawn Atlas in a different position, change `robot_initial_pose/x` and `robot_initial_pose/y` to whatever you desire.

## Modify an empty world
Open gazebo with no command line arguments:

~~~
gazebo
~~~

You will see a blank world.

Now, add some simple shapes to the environment using the icons in the toolbar. You can add boxes, spheres or cylinders.

[[file:files/shapes_toolbar.png|640px]]

You can even add pre-existing models to the environment using the "Insert Models" tab:

[[file:files/insert_models.png|640px]]

When you're done, save the world by clicking "File" then "Save World As". You can also use the keyboard shortcut, CTRL-SHIFT-S. Call it `myworld.world` and save it to `~/ros/world_create_tutorial/worlds`. When you are finished, exit Gazebo.

## Add Atlas to the .world
Now, open the new `.world` in your favorite text editor.

Copy/paste the following block into the file. Make sure the block is a direct child of the `<world></world>` element.

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
~~~
This plugin element spawns Atlas using the VRCPlugin, which also manages the robot's standing controllers, the control interface to the robot, etc.

When you are finished, launch Atlas in the new world:

~~~
roslaunch world_create_tutorial atlas.launch
~~~

You should see Atlas spawned next to the objects you placed in the previous step:

[[file:files/atlas_spawned.png]]
