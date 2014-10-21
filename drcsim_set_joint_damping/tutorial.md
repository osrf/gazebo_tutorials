# Overview

This tutorial will explain how to change Atlas robot and Sandia hand joint viscous damping coefficients.

# Background

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install&cat=drcsim).

In this tutorial, we'll demonstrate how to call ROS services in command-line using [rosservice](http://www.ros.org/wiki/rosservice).

After running Gazebo with Atlas, `roslaunch drcsim_gazebo atlas_sandia_hands.launch`, you can then use `rosservice list` to look for existing services:

~~~
rosservice list | grep damping

/atlas/get_joint_damping
/atlas/set_joint_damping
/sandia_hands/l_hand/get_joint_damping
/sandia_hands/l_hand/set_joint_damping
/sandia_hands/r_hand/get_joint_damping
/sandia_hands/r_hand/set_joint_damping


~~~

Each service is described in detail below:

* To get Atlas robot model's current joint damping coefficients,
 * ROS service: `/atlas/get_joint_damping`
 * message type: [atlas_msgs/srv/GetJointDamping](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/srv/GetJointDamping.srv)
* To set Atlas robot model's joint damping coefficients,
 * ROS service: `/atlas/set_joint_damping`
 * message type: [atlas_msgs/srv/SetJointDamping](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/srv/SetJointDamping.srv)
* To get Sandia hand model's current joint damping coefficients (right hand),
 * ROS service: `/sandia_hands/r_hand/get_joint_damping`
 * message type: [atlas_msgs/srv/GetJointDamping](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/srv/GetJointDamping.srv)
* To set Sandia hand model's joint damping coefficients,
 * ROS service: `/sandia_hands/r_hand/set_joint_damping`
 * message type: [atlas_msgs/srv/SetJointDamping](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/srv/SetJointDamping.srv)

# Running the Tutorial
To run this tutorial, you'll need at least 2 separate terminals.  Don't forget to execute following setup commands in each new terminal you open.

~~~
. /usr/share/drcsim/setup.sh
~~~

Start simulation of Atlas robot with Sandia hands:

~~~
roslaunch drcsim_gazebo atlas_sandia_hands.launch
~~~

To see information the current damping coefficients for Atlas, as well as the max and min values, open a separate terminal and run the following:

~~~
rosservice call /atlas/get_joint_damping
~~~

You should see output similar to the following:

~~~
damping_coefficients: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.2, 0.1]
damping_coefficients_min: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
damping_coefficients_max: [516.7333333333333, 861.8458333333333, 395.4583333333333, 20.833333333333336, 458.3333333333333, 750.0, 1083.3333333333335, 1666.6666666666667, 916.6666666666666, 375.0, 458.3333333333333, 750.0, 1083.3333333333335, 1666.6666666666667, 916.6666666666666, 375.0, 883.3333333333334, 708.3333333333333, 475.0, 475.0, 475.0, 250.0, 883.3333333333334, 708.3333333333333, 475.0, 475.0, 475.0, 250.0]
success: True
status_message: success
~~~

Here, the array of `damping_coefficients` should have exactly 28 elements, with each element corresponds to one joint as listed in [atlas_msgs/AtlasState.msg](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/AtlasSimInterfaceState.msg).  In this example, all joints will have a damping coefficient of `0.1 N*m*sec/rad`, with the exception of `r_arm_uwy` with damping coefficient of `0.2 N*m*sec/rad`.

To see the damping values for the Sandia hands, type the following:

~~~
rosservice call /sandia_hands/r_hand/get_joint_damping
~~~

You should see the following:

~~~
damping_coefficients: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
damping_coefficients_min: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
damping_coefficients_max: [30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
success: True
status_message: success
~~~

Here, each element of the `damping_coefficients` array corresponds to a joint as listed in [SandiaHandPlugin.cc line 84 ~ 95](https://bitbucket.org/osrf/drcsim/src/default/drcsim_gazebo_ros_plugins/src/SandiaHandPlugin.cpp?at=default#cl-84).  So for example, above `rosservice` call will set all joint damping values to `30 N*m*sec/rad` with the exception of `f3_j0`, with damping coefficient of `5.0 N*m*sec/rad`.

Change the Sandia hands model joint damping coefficients by typing:

~~~
. /usr/share/drcsim/setup.sh
rosservice call /sandia_hands/r_hand/set_joint_damping "damping_coefficients: [30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 5.0, 30.0, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
~~~

Similarly for the Atlas robot model:

~~~
. /usr/share/drcsim/setup.sh
rosservice call /atlas/set_joint_damping "damping_coefficients: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.2, 0.1]" 
~~~
