# Overview

This tutorial explains the many options for launching DRCSim.

## Setup

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install) to install DRCSim 4.2. If you did not install DRCSim 4.2, you must upgrade before you can complete this tutorial.

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

## Background
There are many different combinations that can be configured when launching DRCSim. There are 4 possible Atlas models, 3 supported hand models, and over 23 different environments.

# Models
Boston Dynamics has released four versions of the Atlas model: version 1, version 3, version 4, and version 5.

[[file:files/versions.png|750px]]

# Hands
There are three hand models supported in DRCSim: the Sandia hand, the iRobot hand, and the Robotiq hand. There are related tutorials for controlling the [Sandia](http://gazebosim.org/tutorials?tut=drcsim_grasp_sandia&cat=drcsim) hand and the [Robotiq](http://gazebosim.org/tutorials?tut=drcsim_robotiq_hand&cat=drcsim) hand.

![Sandia hands][[file:files/sandia.png|232px]]

![iRobot hands][[file:files/irobot.png|250px]]

~[Robotiq hands][[file:files/robotiq.png|223px]]

# Worlds
The VRC Final Task worlds were 

vrc_final_task<1-15>.world

The DRC Practice Task worlds reflect the tasks outlined in [this document](http://archive.darpa.mil/roboticschallengetrialsarchive/sites/default/files/DRC%20Trials%20Task%20Description%20Release%2011%20DISTAR%2022197.pdf) provided by DARPA. Their world names in Gazebo/DRCSim are denoted as:

~~~
drc_practice_task_<1-8>.world
~~~

# Launchfiles
For most drcsim launchfiles, you can specify the model number using the `model_args` argument and the hand type using the `hand_suffix` argument.

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

# Other options

`extra_gazebo_args`: 

`gzworld`:

`gzname`:

## Example
Try launching Atlas v4 with Robotiq hands in the world for DRC task 4:

~~~
roslaunch drcsim_gazebo drc_practice_task_4.launch model_args:="_v4" hand_suffix:="_robotiq_hands"
~~~

[[file:files/drcpractice4_atlas_v4_robotiq.png|640px]]

