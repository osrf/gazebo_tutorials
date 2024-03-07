# Overview

This tutorial explains the many options for launching DRCSim.

If you have no prior experience with ROS launch files, you can learn more on the [ROS Wiki](http://wiki.ros.org/roslaunch).

# Setup

We assume that you've already installed DRCSim 4.2 or higher via the [installation instructions](/tutorials/?tut=drcsim_install). If you have not installed a version of DRCSim greater than or equal to 4.2, you must upgrade before you can complete this tutorial.

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

# Background
DRCSim supports 4 possible Atlas models, 3 supported hand models, and over 23 different worlds. The many possible combinations are all accessible via launch file arguments.

## Versions
Boston Dynamics has released four versions of the Atlas robot: version 1, version 3, version 4, and version 5.

**Version 1:** This is the first version of Atlas associated with the DRC program. This version was used by teams competing in the Virtual Robotics Challenge in June of 2013.

**Version 3:** Atlas v3 represents the hardware teams received following completion of the Virtual Robotics Challenge. This is also the version that teams used during the DRC Trials competition in December of 2013.

**Version 4:** Atlas v4 is completely untethered. This entailed a backpack redesign. DRC teams received this version of Atlas in late 2014. Take note of the two different versions of this model listed below.

 * **Version 4.0**: The URDF file (`atlas_v4.urdf`) for this robot includes two joints (`l_arm_wry2` and `r_arm_wry2`) that do not exist on the real robot. These two joints are included to make the model compatible with BDI's libAtlasSimInterface.

    Example usage: `roslaunch drcsim_gazebo atlas.launch model_args:="_v4"`
 
 * **Version 4.1**: The URDF file (`atlas_v4_no_wry2.urdf`) excludes the wry2 joints. It is not possible to use BDI's libAtlasSimInterface library with this version.

    Example usage: `roslaunch drcsim_gazebo atlas.launch model_args:="_v4_no_wry2"`

**Version 5:** The final version of Atlas was made available to teams in early 2015. This version is the same as version 4, except that the forearms are electric.

[[file:files/versions.png|750px]]

## Hands
There are three hand models supported in DRCSim: the Sandia hand, the iRobot hand, and the RobotiQ hand. There are related tutorials for controlling the [Sandia](/tutorials?tut=drcsim_grasp_sandia&cat=drcsim) hand and the [RobotiQ](/tutorials?tut=drcsim_robotiq_hand&cat=drcsim) hand.

Sandia hands:

[[file:files/sandia.png|232px]]

iRobot hands:

[[file:files/irobot.png|250px]]

RobotiQ hands:

[[file:files/robotiq.png|223px]]

## Worlds
The VRC Final Task worlds were used in the VRC (Virtual Robotics Challenge) to help determine which teams would receive an Atlas robot to compete in the DRC Finals. These worlds are accessible in DRCSim under the following world names:

~~~
vrc_final_task<1-15>.world
~~~

The DRC Practice Task worlds reflect the tasks planned for the DRC Finals as outlined in [this document](http://archive.darpa.mil/roboticschallengetrialsarchive/sites/default/files/DRC%20Trials%20Task%20Description%20Release%2011%20DISTAR%2022197.pdf) provided by DARPA. Their world names in Gazebo/DRCSim are:

~~~
drc_practice_task_<1-8>.world
~~~

# Launch file arguments
For most drcsim launch files, you can specify the model version number using the `model_args` argument and the hand type using the `hand_suffix` argument.

`atlas.launch` launches DRCSim with Atlas in an empty world:

~~~
roslaunch drcsim_gazebo atlas.launch model_args:="_v<model number>" hand_suffix:="_<sandia,irobot,robotiq>_hands"
~~~

To launch a VRC final world, the launch file name is the same as the world file name speified above:

~~~
roslaunch drcsim_gazebo vrc_final_task<task number>.launch model_args:="_v<model number>" hand_suffix:="_<sandia,irobot,robotiq>_hands"
~~~

Similarly for the DRC practice worlds:

~~~
roslaunch drcsim_gazebo drc_practice_task_<task number>.launch model_args:="_v<model number>" hand_suffix:="_<sandia,irobot,robotiq>_hands"
~~~

If you want to launch Atlas with no hands, simply leave out `hand_suffix` or pass an empty string. In most cases, if you leave out `model_args`, the default Atlas model will be v3 or v1.

## Other options

`extra_gazebo_args`: Use this argument to add extra options to Gazebo, such as `--verbose`. For a full list of Gazebo options, type `gazebo --help` into a terminal.

`gzworld`: Load a different world file. You can use this with `atlas.launch` to place Atlas into any Gazebo world.

`gzname`: Specify the executable used to invoke Gazebo. Common options include `gazebo` or `gzserver`. If the Gazebo graphical window does not start, but no errors are printed to the terminal, try setting `gzname:="gazebo"`. `gzname` is set to `gzserver` by default for the `vrc_final` worlds, which means the graphical client window will not start unless `gzclient` is launched separately.

# Example
Try launching Atlas v4 with RobotiQ hands in the world for DRC task 4:

~~~
roslaunch drcsim_gazebo drc_practice_task_4.launch model_args:="_v4" hand_suffix:="_robotiq_hands"
~~~

[[file:files/drcpractice4_atlas_v4_robotiq.png|640px]]

