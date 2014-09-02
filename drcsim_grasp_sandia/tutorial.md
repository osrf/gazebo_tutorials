# Overview

In this tutorial, we'll send desired grasp pose to Sandia hands mounted on an Atlas robot in simulation through the use of a simple ROS topic publisher. Please note that this is a tremendously simplistic implementation of grasping, and is not in any way attempting to be a "reference" implementation of any sort. The controller is just a strawman to demonstrate motion of the hands into several canonical grasps.

# Setup

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim).

If you haven't done so, make sure to source the environment setup.sh files with every new terminal you open:

~~~
source /usr/share/drcsim/setup.sh
~~~

To save on typing, you can add this script to your `.bashrc` files, so it's automatically sourced every time you start a new terminal.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

But remember to remove them from your `.bashrc` file when they are not needed any more.

# Running the Simulation

In terminal, source the DRC simulator setup script and start the DRC robot simulation:

~~~
roslaunch drcsim_gazebo atlas_sandia_hands.launch
~~~

Look for ros nodes controlling Sandia hands by typing

~~~
rosservice list | grep sandia_hand

In particular, note the Sandia hands services:

/sandia_hands/l_hand/simple_grasp
/sandia_hands/r_hand/simple_grasp
~~~

To get more information on the type of service offered, type

~~~
rosservice info /sandia_hands/l_hand/simple_grasp
Node: /sandia_hands/l_hand/simple_grasp_left
URI: rosrpc://lcp1:37083
Type: sandia_hand_msgs/SimpleGraspSrv
Args: grasp
~~~

  The service type is [sandia_hand_msgs/SimpleGraspSrv](https://bitbucket.org/osrf/sandia-hand/src/default/ros/sandia_hand_msgs/srv/SimpleGraspSrv.srv) which contains a [SimpleGrasp message](https://bitbucket.org/osrf/sandia-hand/src/default/ros/sandia_hand_msgs/msg/SimpleGrasp.msg), to see what it is, do:

~~~
rosmsg show SimpleGrasp
[sandia_hand_msgs/SimpleGrasp]:
string name
float64 closed_amount
~~~

The `name` string may take the value of "cylindrical", "prismatic", or "spherical" and refers to the type of grasp. The `closed_amount` value should be between 0 and 1, with 0 being an open hand and 1 being a closed grasp. The source code for this controller is in [simple\_grasp.py in the sandia\_hand\_teleop package](https://bitbucket.org/osrf/sandia-hand/src/default/ros/sandia_hand_teleop/control_nodes/simple_grasp.py).

In a separate terminal, invoke the service call to close left hand half way by executing:

~~~
rosservice call /sandia_hands/l_hand/simple_grasp ' { grasp: { name: "spherical", closed_amount: 0.8 } }'
~~~

[[file:files/sandia_hand_open.png|300px]]
[[file:files/sandia_hand_closed.png|300px]]
