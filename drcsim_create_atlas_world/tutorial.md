# Overview
This tutorial will teach you how to create a Gazebo world and spawn Atlas into it.

## Setup
We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim).

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

## Create package
Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on `drcsim_gazebo`:

~~~
cd ~/ros
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
roscreate-pkg world_create_tutorial drcsim_gazebo
roscd world_create_tutorial
mkdir worlds launch
~~~

Next, copy and paste this launchfile into `~/ros/world_create_tutorial/launch/atlas.launch`, or download the file [here](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_create_atlas_world/files/atlas.launch):

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_create_atlas_world/files/atlas.launch' />

This launchfile is nearly identical of the `atlas.launch` file in `drcsim_gazebo`, except that the `gzworld` argument has been changed to a different file. This file doesn't exist yet, but we will create it in the next step.

If you want to spawn Atlas in a different position, change `robot_initial_pose/x` and `robot_initial_pose/y`.

## Modify an empty world
Open gazebo with no command line arguments:

~~~
gazebo
~~~

You will see a blank world.

Now, add some simple shapes to the environment using the icons in the toolbar. You can add boxes, spheres or cylinders.

[[file:files/shapes_toolbar.png|320px]]

You can even add pre-existing models to the environment using the "Insert Models" tab:

[[file:files/insert_models.png|240px]]

When you're done, save the world by clicking "File" then "Save World As". You can also use the keyboard shortcut, CTRL-SHIFT-S. Call it `myworld.world` and save it to `~/ros/world_create_tutorial/worlds`. When you are finished, exit Gazebo.

## Add Atlas to the .world
Now, open the new `.world` in your favorite text editor.

Copy/paste the following block into the file. Make sure the block is a direct child of the `<world></world>` element.

~~~language-xml
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

[[file:files/atlas_spawned.png|320px]]

From here, you're ready to modify the robot's environment any way you like.  Have a look at [SDF documentation](http://gazebosim.org/sdf) and edit `~/ros/world_modification_tutorial/worlds/atlas.world` however you like.
