# Overview

In this tutorial, we'll move a robot in simulation without dynamics through the use of a model plugin that listens to the [trajectory_msgs::JointTrajectory message](http://ros.org/wiki/trajectory_msgs) over a [ROS topic](http://www.ros.org/wiki/Topics).

## Setup

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install).

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

If you haven't already, create a ros directory in your home directory and add it to your `$ROS_PACKAGE_PATH`. From the command line

~~~
mkdir ~/ros
echo "export ROS_PACKAGE_PATH=\$HOME/ros:\$ROS_PACKAGE_PATH" >> ~/.bashrc
source ~/.bashrc
~~~

Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on `roscpp` and `trajectory_msgs`:

~~~
cd ~/ros
roscreate-pkg joint_animation_tutorial roscpp trajectory_msgs
cd joint_animation_tutorial
mkdir scripts
cd scripts
~~~

## Model Plugin Controller

A [joint trajectory model plugin](https://bitbucket.org/osrf/drcsim/src/4dd60578a573/plugins/ros/gazebo_ros_joint_trajectory.h?at=default) has been embedded in the DRC robot. The plugin subscribes to a ROS topic containing [JointTrajectory messages](http://ros.org/wiki/trajectory_msgs), and plays back the joint trajectory on the robot. This is useful when you want to move the robot without worrying about the dynamics of the robot model. This plugin also disables physics updates while playing back the joint trajectory.

### Create a ROS Publisher

Download into the current directory a python ROS node that publishes joint trajectory messages [`joint_animation.py`](http://bitbucket.org/osrf/gazebo_tutorials/src/default/drcsim_animate_joints/files/joint_animation.py'):

~~~
wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py
~~~

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Make the file executable

~~~
chmod +x joint_animation.py
~~~

### The Code explained

<include to='/tutorial.\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Standard for every rospy node. This imports roslib and then loads the manifest.xml included in the package so those packages are importable as well.

<include from='/import rospy/' to='/JointTrajectoryPoint/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Import more modules, and import the message file for JointTrajectory and JointTrajectoryPoint.

<include from='/def jointTrajectoryCommand/' to='/JointTrajectory\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

This initializes the node and creates a publisher for the /joint_trajectory topic.

<include from='/    jt =/' to='/atlas::pelvis"/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Create an instantiation of a JointTrajectory message and add the time stamp and frame_id to the header.

<include from='/    jt\.joint/' to='/append\("atlas::r_arm_uwy"\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Create the list of names of joints that will be controlled.

<include from='/    n = 1500/' to='/1\*theta\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Setup a for loop that runs for n=1500 times. It calculates joint angles at two different positions x1 and x2. There should be a position for each joint added above.

<include from='/    p.positions.append\(x1\)/' to='/    jt.points.append\(p\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Create a list of positions that the JointTrajectoryPoint will follow.
Next, add the JointTrajectoryPoint to the JointTrajectory and proceed to the next point.

<include from='/        # set duration/' to='/n,x1,x2\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

Log the point that was created.

<include from='/    pub.publish/' to='/spin\(\)/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

This will publish the single JointTrajectory message, which the robot will execute. The node will then spin, which allows the node to continue running without blocking the CPU.

<include from='/if __/' to='/: pass/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_animate_joints/files/joint_animation.py' />

The main method of the rospy node. It prevents the node from executing code if the thread has been shutdown.

## Running the Simulation

1. In terminal, start the DRC robot simulation:

    ~~~
    VRC_CHEATS_ENABLED=1 roslaunch drcsim_gazebo atlas.launch
    ~~~

1. To prevent the robot from falling over (it's not running any controllers), disable gravity by clicking on `World->Physics->gravity->z` and setting the value to `0.0`.

1. To prevent the robot from bouncing off the ground and flying into space (there's no gravity), remove the ground by clicking on `World->Models`, then right-clicking on `ground_plane` and clicking Delete.

1. With gravity off and ground plane deleted, reset the world by clicking `Edit->Reset` World.  The robot should now be in its default pose, "standing" at the origin with arms outstretched.

1. In a separate terminal:

    ~~~
    rosrun joint_animation_tutorial joint_animation.py
    ~~~

    The DRC robot should move according to the published ROS JointTrajectory message.
