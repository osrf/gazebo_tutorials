# Overview

In this tutorial, we'll move a robot in simulation without dynamics through the use of a model plugin that listens to the [trajectory_msgs::JointTrajectory message](http://ros.org/wiki/trajectory_msgs) over a [ROS topic](http://www.ros.org/wiki/Topics).

## Setup

We assume that you've already done the [installation step](http://gazebosim.org/tutorials/?tut=drcsim_install).

If you haven't done so, add the environment setup.sh files to your .bashrc.

    echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
    source ~/.bashrc

If you haven't already, create a ros directory in your home directory and add it to your `$ROS_PACKAGE_PATH`. From the command line

    mkdir ~/ros
    echo "export ROS_PACKAGE_PATH=\$HOME/ros:\$ROS_PACKAGE_PATH" >> ~/.bashrc
    source ~/.bashrc

Use [roscreate-pkg](http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on `roscpp` and `trajectory_msgs`:

    cd ~/ros
    roscreate-pkg joint_animation_tutorial roscpp trajectory_msgs
    cd joint_animation_tutorial
    mkdir scripts
    cd scripts

## Model Plugin Controller

A [joint trajectory model plugin](https://bitbucket.org/osrf/drcsim/src/4dd60578a573/plugins/ros/gazebo_ros_joint_trajectory.h?at=default) has been embedded in the DRC robot. The plugin subscribes to a ROS topic containing [JointTrajectory messages](http://ros.org/wiki/trajectory_msgs), and plays back the joint trajectory on the robot. This is useful when you want to move the robot without worrying about the dynamics of the robot model. This plugin also disables physics updates while playing back the joint trajectory.

### Create a ROS Publisher

Create a python ROS node that publishes joint trajectory messages `~/ros/joint_animation_tutorial/nodes/joint_animation.py`:


    gedit joint_animation.py

Paste the following content into it:

<pre>
#!/usr/bin/env python

import roslib; roslib.load_manifest('joint_animation_tutorial')
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control')

    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()

    pub = rospy.Publisher('/joint_trajectory', JointTrajectory)
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = "atlas::pelvis"

    jt.joint_names.append("atlas::back_lbz" )
    jt.joint_names.append("atlas::back_mby" )
    jt.joint_names.append("atlas::back_ubx" )
    jt.joint_names.append("atlas::neck_ay"  )
    jt.joint_names.append("atlas::l_leg_uhz")
    jt.joint_names.append("atlas::l_leg_mhx")
    jt.joint_names.append("atlas::l_leg_lhy")
    jt.joint_names.append("atlas::l_leg_kny")
    jt.joint_names.append("atlas::l_leg_uay")
    jt.joint_names.append("atlas::l_leg_lax")
    jt.joint_names.append("atlas::r_leg_lax")
    jt.joint_names.append("atlas::r_leg_uay")
    jt.joint_names.append("atlas::r_leg_kny")
    jt.joint_names.append("atlas::r_leg_lhy")
    jt.joint_names.append("atlas::r_leg_mhx")
    jt.joint_names.append("atlas::r_leg_uhz")
    jt.joint_names.append("atlas::l_arm_elx")
    jt.joint_names.append("atlas::l_arm_ely")
    jt.joint_names.append("atlas::l_arm_mwx")
    jt.joint_names.append("atlas::l_arm_shx")
    jt.joint_names.append("atlas::l_arm_usy")
    jt.joint_names.append("atlas::l_arm_uwy")
    jt.joint_names.append("atlas::r_arm_elx")
    jt.joint_names.append("atlas::r_arm_ely")
    jt.joint_names.append("atlas::r_arm_mwx")
    jt.joint_names.append("atlas::r_arm_shx")
    jt.joint_names.append("atlas::r_arm_usy")
    jt.joint_names.append("atlas::r_arm_uwy")

    n = 1500
    dt = 0.01
    rps = 0.05
    for i in range (n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 = -0.5*math.sin(2*theta)
        x2 =  0.5*math.sin(1*theta)

        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        jt.points.append(p)

        # set duration
        jt.points[i].time_from_start = rospy.Duration.from_sec(dt)
        rospy.loginfo("test: angles[%d][%f, %f]",n,x1,x2)

    pub.publish(jt)
    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass
</pre>


Make the file executable

    chmod +x joint_animation.py

### The Code explained ###

<pre>
#!/usr/bin/env python

import roslib; roslib.load_manifest('joint_animation_tutorial')
</pre>
Standard for every rospy node. This imports roslib and then loads the manifest.xml included in the package so those packages are importable as well.

<pre>
import rospy, math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
</pre>
Import more modules, and import the message file for JointTrajectory and JointTrajectoryPoint.

<pre>
def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control')

    pub = rospy.Publisher('/joint_trajectory', JointTrajectory)
</pre>
This initializes the node and creates a publisher for the /joint_trajectory topic.

<pre>
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = "atlas::pelvis"
</pre>
Create an instantiation of a JointTrajectory message and add the time stamp and frame_id to the header.

<pre>
    jt.joint_names.append("atlas::back_lbz" )
    jt.joint_names.append("atlas::back_mby" )
    jt.joint_names.append("atlas::back_ubx" )
    jt.joint_names.append("atlas::neck_ay"  )
    jt.joint_names.append("atlas::l_leg_uhz")
    jt.joint_names.append("atlas::l_leg_mhx")
    jt.joint_names.append("atlas::l_leg_lhy")
    jt.joint_names.append("atlas::l_leg_kny")
    jt.joint_names.append("atlas::l_leg_uay")
    jt.joint_names.append("atlas::l_leg_lax")
    jt.joint_names.append("atlas::r_leg_lax")
    jt.joint_names.append("atlas::r_leg_uay")
    jt.joint_names.append("atlas::r_leg_kny")
    jt.joint_names.append("atlas::r_leg_lhy")
    jt.joint_names.append("atlas::r_leg_mhx")
    jt.joint_names.append("atlas::r_leg_uhz")
    jt.joint_names.append("atlas::l_arm_elx")
    jt.joint_names.append("atlas::l_arm_ely")
    jt.joint_names.append("atlas::l_arm_mwx")
    jt.joint_names.append("atlas::l_arm_shx")
    jt.joint_names.append("atlas::l_arm_usy")
    jt.joint_names.append("atlas::l_arm_uwy")
    jt.joint_names.append("atlas::r_arm_elx")
    jt.joint_names.append("atlas::r_arm_ely")
    jt.joint_names.append("atlas::r_arm_mwx")
    jt.joint_names.append("atlas::r_arm_shx")
    jt.joint_names.append("atlas::r_arm_usy")
    jt.joint_names.append("atlas::r_arm_uwy")
</pre>
Create the list of names of joints that will be controlled.

<pre>
    n = 1500
    dt = 0.01
    rps = 0.05
    for i in range(n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 = -0.5*math.sin(2*theta)
        x2 =  0.5*math.sin(1*theta)
</pre>
Setup a for loop that runs for n=1500 times. It calculates joint angles at two different positions x1 and x2. There should be a position for each joint added above.

<pre>
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
</pre>
Create a list of positions that the JointTrajectoryPoint will follow.

<pre>
        jt.points.append(p)
</pre>
Add the JointTrajectoryPoint to the JointTrajectory and proceed to the next point.

<pre>
        # set duration
        jt.points[i].time_from_start = rospy.Duration.from_sec(dt)
        rospy.loginfo("test: angles[%d][%f, %f]",n,x1,x2)
</pre>
Log the point that was created.

<pre>
    pub.publish(jt)
    rospy.spin()
</pre>
This will publish the single JointTrajectory message, which the robot will execute. The node will then spin, which allows the node to continue running without blocking the CPU.


<pre>
if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass
</pre>
The main method of the rospy node. It prevents the node from executing code if the thread has been shutdown.


## Running the Simulation

1. In terminal, start the DRC robot simulation (use the launch file **without active mechanism controllers** so they do not conflict with the joint animation):

        VRC_CHEATS_ENABLED=1 roslaunch atlas_utils atlas_no_controllers.launch

1. To prevent the robot from falling over (it's not running any controllers), disable gravity by clicking on World->Physics->gravity->z and setting the value to 0.0.

1. To prevent the robot from bouncing off the ground and flying into space (there's no gravity), remove the ground by clicking on World->Models, then right-clicking on `ground_plane` and clicking Delete.

1. With gravity off and ground plane deleted, reset the world by clicking Edit->Reset World.  The robot should now be in its default pose, "standing" at the origin with arms outstretched.

1. In a separate terminal:

        rosrun joint_animation_tutorial joint_animation.py

    The DRC robot should move according to the published ROS JointTrajectory message.

## Next

[Using the VRC Plugin with the DRC Vehicle](http://gazebosim.org/tutorials/?tut=drcsim_vehicle)
