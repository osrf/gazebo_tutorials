# Overview

This tutorial explains how to use IHMC open source walking controller to perform
dynamic balancing with the Atlas or Valkyrie robot.

## Setting Up Workspace

Install prerequesites:

~~~
sudo apt-get update
sudo apt-get install drcsim-prerelease
~~~

Setup a catkin workspace

~~~
mkdir ~/catkin_ws
mkdir ~/catkin_ws/src
~~~

Checkout [ihmc_ros]() and [val_description]().

~~~
cd ~/catkin/src
git clone https://bitbucket.org/ihmcrobotics/ihmc_ros
git clone https://github.com/NASA-JSC-Robotics/val_description
~~~

Compile by running:

~~~
cd ~/catkin_ws
source /opt/ros/indigo/setup.bash
cakint_make install
~~~

## Running

Setup environment by running

~~~
cd ~/catkin_ws
source /usr/share/drcsim/setup.sh
source install/setup.bash
~~~

Launch the simulator (be sure to first `source /usr/share/drcsim/setup.sh` as usual):

~~~
roslaunch ihmc_gazebo ihmc_valkyrie_standing.launch extra_gazebo_args:="--verbose"
~~~

Download the controller

~~~
rosrun ihmc_valkyrie ihmc_dist_update.py
~~~

you should see similar outputs on the console
~~~
$ rosrun ihmc_valkyrie ihmc_dist_update.py
Update Available! Preparing to download IHMCValkyrieAPI-0.0.1.tar
Downloading IHMCValkyrieAPI-0.0.1.tar to ihmc_valkyrie package...
Untarring distribution and cleaning up...
Distribution update complete!
~~~

Next, start the controller

~~~
roslaunch ihmc_valkyrie ihmc_valkyrie_gazebo_controller.launch
~~~

You should see the console display following messages, ending with `Connected`

~~~
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://sulla:58199/

SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.13
 * /use_sim_time: True
 * /valkyrie/robot_description: <?xml version="1....

NODES
  / 
    IHMCValkyrieGazeboController (ihmc_valkyrie/IHMCValkyrieGazeboController)
    robot_state_publisher (robot_state_publisher/state_publisher)

auto-starting new master
process[master]: started with pid [29030]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 00d16682-42cb-11e5-ab34-d0509919a838
process[rosout-1]: started with pid [29043]
started core service [/rosout]
process[robot_state_publisher-2]: started with pid [29060]
process[IHMCValkyrieGazeboController-3]: started with pid [29061]
[INFO] (NetworkParameters.java:34): Loading network parameters from /home/hsu/projects/ihmc_code/ihmc_gazebo_catkin_ws/install/share/ihmc_valkyrie/configurations/IHMCNetworkParametersSim.ini
num of joints = 59
[WARN] (TaskspaceToJointspaceHandForcefeedbackControlState.java:193): Joint control Mode: FORCE_CONTROL
[WARN] (TaskspaceToJointspaceHandForcefeedbackControlState.java:193): Joint control Mode: FORCE_CONTROL
[INFO] (DRCNetworkProcessor.java:270): Connecting to controller using intra process communication
log4j:WARN No appenders could be found for logger (org.ros.node.DefaultNodeMainExecutor).
log4j:WARN Please initialize the log4j system properly.
log4j:WARN See http://logging.apache.org/log4j/1.2/faq.html#noconfig for more info.
[INFO] (RosTfPublisher.java:24): tfPrefix option set to NONE - using no prefix
IHMC ROS API node successfully started.
Announcing logging session on: name:lo (lo)
Trying port 56483
[GazeboOutputWriter] Connecting to /127.0.0.1:1235
[GazeboOutputWriter] Connected
num of joints = 59
[GazeboSensorReader] Connecting to /127.0.0.1:1234
[GazeboSensorReader] Connected
~~~

At this point, the robot should be standing

![alt text](http://bitbucket.org/osrf/gazebo_tutorials/raw/ihmc_walking_controller_john/ihmc_walking_controller/files/valkyrie_gazebo.png "Valkyrie robot performing dynamic balanced standing in Gazebo simulation.")

## ROS Interface

### Example: lower the robot center of mass:

~~~
rostopic pub -1 /ihmc_ros/valkyrie/control/com_height ihmc_msgs/ComHeightPacketMessage '{ height_offset: -0.1, trajectory_time: 1.0 }'
~~~

## Known Issues
 * Current controller library is a binary distribution. The source version will be release in October 2015.
