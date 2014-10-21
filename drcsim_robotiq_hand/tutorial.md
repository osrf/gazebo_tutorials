# Overview

In this tutorial, we'll describe how to interact with the simulated Robotiq hand,
as well as how to read its state or visualize it in rviz.

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

# Running the Simulation and controlling the gripper

In terminal, source the DRC simulator setup script and start the DRC robot simulation:

~~~
roslaunch drcsim_gazebo atlas_robotiq_hands.launch
~~~

If the initial hand configuration does not match your robot's configuration, you
can edit the file [`atlas_robotiq_hands.urdf.xacro`](https://bitbucket.org/osrf/drcsim/src/default/atlas_description/robots/atlas_robotiq_hands.urdf.xacro).
Note that there are alternative versions of this file for Atlas V3 and Atlas V4.

Once your configuration is correct you can look for ROS nodes controlling Robotiq hands by typing:

~~~
rostopic list | grep hand
~~~

In particular, note the Robotiq hand messages:

%%%
/left_hand/command
/left_hand/state
/right_hand/command
/right_hand/state
/robotiq_hands/left_hand/joint_states
/robotiq_hands/right_hand/joint_states
%%%

To get more information on the type of message offered, type

~~~
rostopic info /right_hand/command
~~~

You should see something like this:

%%%
Type: atlas_msgs/SModelRobotOutput

Publishers: None

Subscribers:
 * /gazebo (http://turtlebot:54036/)

%%%

  The message type is [atlas_msgs/SModelRobotOutput](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/SModelRobotOutput.msg) which contains all the different registers for controlling the behavior of the hand:

~~~
rosmsg show SModelRobotOutput
~~~

%%%
uint8 rACT
uint8 rMOD
uint8 rGTO
uint8 rATR
uint8 rICF
uint8 rICS
uint8 rPRA
uint8 rSPA
uint8 rFRA
uint8 rPRB
uint8 rSPB
uint8 rFRB
uint8 rPRC
uint8 rSPC
uint8 rFRC
uint8 rPRS
uint8 rSPS
uint8 rFRS
%%%

Open a separate terminal and make sure to source the environment setup.sh files as we explained at the beginning of the tutorial.
Then, invoke the next command to send a ROS message for closing the right hand half way by executing:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,0,1,0,0,0,127,255,0,155,0,0,255,0,0,0,0,0}
~~~

You can send the next ROS message for fully opening the hand:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,0,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}
~~~

[[file:files/robotiq_hand_open.png|300px]]
[[file:files/robotiq_hand_close.png|300px]]

The hand supports different grasping modes and uses multiple registers to control the position of the fingers among other parameters.
Check out the Section 4.4 of the [instruction manual](robotiq.com/media/3-FINGER-140613.pdf) for a full list of features and a description of each control register.

As an example, you can execute the following command to change the grasping mode to `pinch` and close the gripper:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,1,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0}
~~~

Or switch to wide mode and fully open the hand:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,2,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}
~~~

Or you can change to `Scissor` mode and close the fingers:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,3,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0}
~~~

And open the fingers:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,3,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}
~~~

# Showing the gripper state

The gripper state is periodically published via ROS messages and available at:

%%%
/left_hand/state
/right_hand/state
%%%

Open a separate terminal and make sure to source the environment setup.sh files as we explained at the beginning of the tutorial.
Then, execute the next command for visualizing the output of the right hand:

~~~
rostopic echo /right_hand/state
~~~

You should see something similar to:

%%%
---
gACT: 1
gMOD: 3
gGTO: 1
gIMC: 3
gSTA: 2
gDTA: 1
gDTB: 1
gDTC: 1
gDTS: 3
gFLT: 0
gPRA: 0
gPOA: 1
gCUA: 0
gPRB: 0
gPOB: 1
gCUB: 0
gPRC: 0
gPOC: 1
gCUC: 0
gPRS: 0
gPOS: 3
gCUS: 0
---
%%%

Now, come back to the terminal that you are using for sending joint commands and
close the hand by typing:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,0,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0}
~~~

You should see how the different state fields change according to the target position specified,
the current finger position, velocities, etc. Check out the Section 4.5 of the
[instruction manual](robotiq.com/media/3-FINGER-140613.pdf) for a detailed description
of the state register.

# Visualizing the gripper in rviz

The simulated robotiq gripper publishes its state using ROS, so it is possible
to graphically visualize its state using rviz. Go to the terminal where you
where visualizing the hand state and press CTRL-C.

Open rviz:

%%%
rosrun rviz rviz
%%%

Let's start by adding Atlas to the visualization by pressing the `Add` button,
and then, `RobotModel`. Change the property `Fixed frame` to `pelvis`.

Now, you should be able to visualize the hands in real time.

[[file:files/robotiq_hand_rviz.png|600px]]

# Known limitations

The following features are not yet available in drcsim:

* Speed/torque control.
* Individual Control of Scissor.
* Joint coupling between actuated and underactuated joints.

We will update this tutorial when these features become available.