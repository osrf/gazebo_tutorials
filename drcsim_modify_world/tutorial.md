# Overview

This tutorial will explain how to modify the simulated Atlas's environment, such as adding objects to the world.

1. Create a new ROS package based on the DRC simulation package.
2. Copy launch files and world from the DRC simulation package, and edit them to reference each other.
3. Create a custom world  file with extra objects.
4. Launch the new simulation with a ROS command.

## Setup ##

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim).

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

If you haven't already, create a ros directory in your home directory and add it to your `$ROS_PACKAGE_PATH`. From the command line

~~~
mkdir -p ~/ros
echo "export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}" >> ~/.bashrc
source ~/.bashrc
~~~

Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on `drcsim_gazebo`:

~~~
cd ~/ros
roscreate-pkg world_modification_tutorial drcsim_gazebo
~~~

## The code ##

The DRC Simulator includes a simulation environment that puts the Atlas Robot in an empty world.  You can launch this simulation configuration like so:

~~~
roslaunch drcsim_gazebo atlas.launch
~~~


> **For drcsim < 3.1.0**: The package and launch file had a different name:

>~~~
roslaunch atlas_utils atlas.launch
>~~~

You'll see the robot on its own, with just the ground:

[[file:files/Gazebo_with_drc_robot.png|640px]]

Press `Control-C` in the terminal where you executed the `roslaunch` command (not in the GUI) to kill the simulator.

We want to add some objects to that world.  But we don't want to modify the contents of `drcsim_gazebo`, because it is a system-installed package.  Instead, we'll use our ROS package created above in the Setup step and copy some files into it:

~~~
roscd world_modification_tutorial
~~~
The above command is equivalent to: cd `~/ros/world_modification_tutorial`

~~~
mkdir worlds launch
~~~

~~~
roscd drcsim_gazebo
~~~
The above command takes you to the `drcsim_gazebo` directory, which may be different according to your installation. It will be something like `/opt/ros/indigo/share/drcsim_gazebo`. We will copy some files from there:

~~~
cp launch/atlas.launch ~/ros/world_modification_tutorial/launch/
cp launch/atlas_no_controllers.launch ~/ros/world_modification_tutorial/launch/
cp ../drcsim_model_resources/worlds/atlas.world ~/ros/world_modification_tutorial/worlds/
~~~

~~~
roscd world_modification_tutorial/
~~~

Now that we have our own copies of the `.launch` and `.world` files, we can make changes.  We'll start with the `.launch` files: open `~/ros/world_modification_tutorial/launch/atlas.launch` in a text editor.

    gedit ~/ros/world_modification_tutorial/launch/atlas.launch

1. Look for this line:

        <include file="$(find drcsim_gazebo)/launch/atlas_no_controllers.launch">

1. In this `atlas.launch` file, which is our local copy, we want to use our local copy of the `atlas_no_controllers.launch` file.  So replace the above line with:

        <include file="$(find world_modification_tutorial)/launch/atlas_no_controllers.launch">

Now we need to change `atlas_no_controllers.launch` to refer to the correct world file.

    gedit ~/ros/world_modification_tutorial/launch/atlas_no_controllers.launch

1. Look for this line:

        <node name="gazebo" pkg="drcsim_gazebo" type="run_$(arg gzname)" args="$(arg gzworld) $(arg extra_gazebo_args)" output="screen" />

1. Again, to use local copies, we replace the above line with:

        <node name="gazebo" pkg="drcsim_gazebo" type="run_$(arg gzname)" args="$(find world_modification_tutorial)/worlds/$(arg gzworld) $(arg extra_gazebo_args)" output="screen" />

Now you're ready to make modifications to the world.  There are a variety of ways to do this; see the [building a world](http://gazebosim.org/tutorials/?tut=drcsim_build_world) tutorial to get started. For now, we'll make some simple edits via the simulator GUI. Launch the Simulator using your just-edited `atlas.launch` file:

    roslaunch world_modification_tutorial atlas.launch

You should see the same environment: Atlas in an empty world.  Make some changes:

1. Add a box.  In the toolbar, click on the box/cube icon to insert a box in the world.  Move the mouse to where you want it and click to drop it.
1. Add a sphere.  Same as for the box, except click on the sphere icon to start.
1. Add a cylinder.  Same as for the others, except click on the cylinder icon to start.

You should see something like this:

[[file:files/Gazebo_with_drc_robot_and_primitives.png|640px]]

Now you can save your changes: in the Gazebo GUI, click "File->Save World As".  When prompted for where to save it, browse to `world_modification_tutorial/worlds/atlas.world` to overwrite our copy of Atlas's simulation world.

Now, modify the world file to remove the Atlas model. This is because the robot is spawned automatically.

    gedit ~/ros/world_modification_tutorial/worlds/atlas.world

Look for the `<model name='atlas'>` tag and remove it entirely (a few hundred lines of text).  

***Note: Currently, loading a world that contains `<state>` tags is not working properly. Thus, you must also delete the `<state>` tag and all its child elements (everything from `<state>` until `</state>`). To edit the `atlas.world`:***


Now, if you haven't done so already, stop the simulation (via `Ctrl+C` in the terminal where you ran `roslaunch`), and start it again:

    roslaunch world_modification_tutorial atlas.launch

You should see the modified world, as it was when you saved it.

From here, you're ready to modify the robot's environment any way you like.  Have a look at [SDF documentation](http://gazebosim.org/sdf) and edit `~/ros/world_modification_tutorial/worlds/atlas.world` however you like.
