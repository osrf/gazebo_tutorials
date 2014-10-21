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

# Running the Simulation

In terminal, source the DRC simulator setup script and start the DRC robot simulation:

~~~
roslaunch drcsim_gazebo atlas_robotiq_hands.launch
~~~

Look for ros nodes controlling Robotiq hands by typing

~~~
rosservice list | grep hand
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

To get more information on the type of service offered, type

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

In a separate terminal, invoke the next command to send a ROS message for closing the right hand half way by executing:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,0,1,0,0,0,127,255,0,155,0,0,255,0,0,0,0,0}
~~~

You can send the next ROS message for fully opening the hand:

~~~
rostopic pub --once right_hand/command atlas_msgs/SModelRobotOutput {1,0,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}
~~~

[[file:files/robotiq_hand_open.png|300px]]
[[file:files/robotiq_hand_close.png|300px]]
