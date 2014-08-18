# Introduction

This tutorial describes how to use the Atlas Sim Interface to command Atlas to walk dynamically or step statically.

## Setup

We assume that you've already done the [DRCSim installation step](http://gazebosim.org/tutorials?tut=drcsim_install).

If you haven't done so, make sure to source the environment setup.sh files with every new terminal you open:

~~~
source /usr/share/drcsim/setup.sh
~~~

To save on typing, you can add this script to your **.bashrc** files, so it's automatically sourced every time you start a new terminal.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
source ~/.bashrc
~~~

But remember to remove them from your **.bashrc** file when they are not needed any more.

### Create a ROS Package Workspace

If you haven't already, create a ros directory in your home directory and add it to your $ROS\_PACKAGE\_PATH. From the command line

~~~
mkdir ~/ros
export ROS_PACKAGE_PATH=${HOME}/ros:${ROS_PACKAGE_PATH}
~~~

Use [roscreate-pkg]( http://ros.org/wiki/roscreate) to create a ROS package for this tutorial, depending on **rospy** and **atlas_msgs**:

~~~
cd ~/ros
roscreate-pkg atlas_sim_interface_tutorial rospy atlas_msgs
~~~

### Create a ROS Node
Copy and paste the following code as file **~/ros/atlas_sim_interface_tutorial/scripts/walk.py** with any text editor (e.g. gedit, vi, emac):

~~~
#! /usr/bin/env python
import roslib; roslib.load_manifest('atlas_sim_interface_tutorial')

from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math
import rospy
import sys

class AtlasWalk():
    
    def walk(self):
        # Initialize atlas mode and atlas_sim_interface_command publishers        
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
          True, None)
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        
        # Assume that we are already in BDI Stand mode
        
        # Initialize some variables before starting.
        self.step_index = 0
        self.is_swaying = False
        
        # Subscribe to atlas_state and atlas_sim_interface_state topics.
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self.atlas_state = rospy.Subscriber('/atlas/atlas_state', AtlasState, self.atlas_state_cb)

        # Walk in circles until shutdown.
        while not rospy.is_shutdown():
            rospy.spin()
        print("Shutting down")
        
    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position
    def asi_state_cb(self, state):
        try:
            x = self.robot_position.x
        except AttributeError:
            self.robot_position = Point()
            self.robot_position.x = state.pos_est.position.x
            self.robot_position.y = state.pos_est.position.y
            self.robot_position.z = state.pos_est.position.z
        
        if self.is_static:
            self.static(state)
        else:
            self.dynamic(state)
    
    # /atlas/atlas_state callback. This message provides the orientation of the robot from the torso IMU
    # This will be important if you need to transform your step commands from the robot's local frame to world frame
    def atlas_state_cb(self, state):
        # If you don't reset to harnessed, then you need to get the current orientation
        roll, pitch, yaw = euler_from_quaternion([state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w])
        
    # An example of commanding a dynamic walk behavior.     
    def dynamic(self, state):                
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.WALK
        
        # k_effort is all 0s for full BDI controll of all joints.
        command.k_effort = [0] * 28
        
        # Observe next_step_index_needed to determine when to switch steps.
        self.step_index = state.walk_feedback.next_step_index_needed
        
        # A walk behavior command needs to know three additional steps beyond the current step needed to plan
        # for the best balance
        for i in range(4):
            step_index = self.step_index + i
            is_right_foot = step_index % 2
            
            command.walk_params.step_queue[i].step_index = step_index
            command.walk_params.step_queue[i].foot_index = is_right_foot
            
            # A duration of 0.63s is a good default value
            command.walk_params.step_queue[i].duration = 0.63
            
            # As far as I can tell, swing_height has yet to be implemented
            command.walk_params.step_queue[i].swing_height = 0.2

            # Determine pose of the next step based on the step_index
            command.walk_params.step_queue[i].pose = self.calculate_pose(step_index)
        
        # Publish this command every time we have a new state message
        self.asi_command.publish(command)
       
       
    # An example of commanding a static walk/step behavior.
    def static(self, state):
        
        # When the robot status_flags are 1 (SWAYING), you can publish the next step command.
        if (state.step_feedback.status_flags == 1 and not self.is_swaying):
            self.step_index += 1
            self.is_swaying = True
            print("Step " + str(self.step_index))
        elif (state.step_feedback.status_flags == 2):
            self.is_swaying = False
        
        is_right_foot = self.step_index % 2
        
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.STEP

        # k_effort is all 0s for full bdi control of all joints
        command.k_effort = [0] * 28
        
        # step_index should always be one for a step command
        command.step_params.desired_step.step_index = 1
        command.step_params.desired_step.foot_index = is_right_foot
        
        # duration has far as I can tell is not observed
        command.step_params.desired_step.duration = 0.63
        
        # swing_height is not observed
        command.step_params.desired_step.swing_height = 0.1

        if self.step_index > 30:
            print(str(self.calculate_pose(self.step_index)))
        # Determine pose of the next step based on the number of steps we have taken
        command.step_params.desired_step.pose = self.calculate_pose(self.step_index)
        
        # Publish a new step command every time a state message is received
        self.asi_command.publish(command)
        
    # This method is used to calculate a pose of step based on the step_index
    # The step poses just cause the robot to walk in a circle
    def calculate_pose(self, step_index):
        # Right foot occurs on even steps, left on odd
        is_right_foot = step_index % 2
        is_left_foot = 1 - is_right_foot
        
        # There will be 60 steps to a circle, and so our position along the circle is current_step
        current_step = step_index % 60
        
        # yaw angle of robot around circle
        theta = current_step * math.pi / 30
           
        R = 2 # Radius of turn
        W = 0.3 # Width of stride
        
        # Negative for inside foot, positive for outside foot
        offset_dir = 1 - 2 * is_left_foot

        # Radius from center of circle to foot
        R_foot = R + offset_dir * W/2
        
        # X, Y position of foot
        X = R_foot * math.sin(theta)
        Y = (R - R_foot*math.cos(theta))
        
        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, theta)
        pose = Pose()
        pose.position.x = self.robot_position.x + X
        pose.position.y = self.robot_position.y + Y
        
        # The z position is observed for static walking, but the foot
        # will be placed onto the ground if the ground is lower than z
        pose.position.z = 0
        
        pose.orientation.x = Q[0]
        pose.orientation.y = Q[1]
        pose.orientation.z = Q[2]
        pose.orientation.w = Q[3]

        return pose
       
if __name__ == '__main__':
    rospy.init_node('walking_tutorial')
    walk = AtlasWalk()
    if len(sys.argv) > 0:
        walk.is_static = (sys.argv[-1] == "static")
    else:
        walk.is_static = False
    walk.walk()
~~~

## The code explained

This node needs the following imports. 

~~~
#! /usr/bin/env python
import roslib; roslib.load_manifest('atlas_sim_interface_tutorial')

from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math
import rospy
import sys
~~~

Initializing the publishers and subscribers for this node. We publish to /atlas/atlas_sim_interface_command and /atlas/mode and listen to /atlas/atlas_sim_interface_state and /atlas/state.

~~~
class AtlasWalk():
    

    def walk(self):
        # Initialize atlas mode and atlas_sim_interface_command publishers        
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
          True, None)
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        
        # Assume that we are already in BDI Stand mode
        
        # Initialize some variables before starting.
        self.step_index = 0
        self.is_swaying = False
        
        # Subscribe to atlas_state and atlas_sim_interface_state topics.
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self.atlas_state = rospy.Subscriber('/atlas/atlas_state', AtlasState, self.atlas_state_cb)

        # Walk in circles until shutdown.
        while not rospy.is_shutdown():
            rospy.spin()
        print("Shutting down")

~~~

This is the atlas_sim_interface_state callback. It provides a lot of useful information. We can get the robot's current position (as estimated by the BDI controller). This position is what is needed to transform a local step coordinate to a global step coordinate.

~~~
    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position
    def asi_state_cb(self, state):
        try:
            x = self.robot_position.x
        except AttributeError:   
            self.robot_position = Point()
            self.robot_position.x = state.pos_est.position.x
            self.robot_position.y = state.pos_est.position.y
            self.robot_position.z = state.pos_est.position.z

~~~

There are two types of walking behavior, static and dynamic. Dynamic is much faster, but foot placement is not as precise. Also, it is much easier to give bad walking commands that cause the atlas robot to fall. Static, is stable throughout the entire step trajectory.

~~~
        if self.is_static:
            self.static(state)
        else:
            self.dynamic(state)
~~~

If the robot is rotated to the world frame, the orientation may need to be accounted for in positioning the steps. This is how you can do that. However, this node does not make use of orientation.

~~~
    # /atlas/atlas_state callback. This message provides the orientation of the robot from the torso IMU
    # This will be important if you need to transform your step commands from the robot's local frame to world frame
    def atlas_state_cb(self, state):
        # If you don't reset to harnessed, then you need to get the current orientation
        roll, pitch, yaw = euler_from_quaternion([state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w])
~~~   

This function walks the robot dynamically in a circle. It is necessary to publish 4 steps at any time, starting with the next_step_index_needed. This helps the walking controller plan for a stable walking trajectory. Some message fields aren't used or implemented in this walking behavior. Dynamic walking behavior is best for flat surfaces with no obstructions.

~~~
    # An example of commanding a dynamic walk behavior.     
    def dynamic(self, state):                
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.WALK
        
        # k_effort is all 0s for full BDI controll of all joints.
        command.k_effort = [0] * 28
        
        # Observe next_step_index_needed to determine when to switch steps.
        self.step_index = state.walk_feedback.next_step_index_needed
        
        # A walk behavior command needs to know three additional steps beyond the current step needed to plan
        # for the best balance
        for i in range(4):
            step_index = self.step_index + i
            is_right_foot = step_index % 2
            
            command.walk_params.step_queue[i].step_index = step_index
            command.walk_params.step_queue[i].foot_index = is_right_foot
            
            # A duration of 0.63s is a good default value
            command.walk_params.step_queue[i].duration = 0.63
            
            # As far as I can tell, swing_height has yet to be implemented
            command.walk_params.step_queue[i].swing_height = 0.2

            # Determine pose of the next step based on the step_index
            command.walk_params.step_queue[i].pose = self.calculate_pose(step_index)
        
        # Publish this command every time we have a new state message
        self.asi_command.publish(command)
~~~       
       
This is an example of static walking/step behavior. You only specify one step at a time, and you have to check the step_feedback field in the state message to determine when you can send the next step command. It is statically stable throughout the entire step trajectory. If you need to step over objects, or step onto steps this behavior is necessary.

~~~
    # An example of commanding a static walk/step behavior.
    def static(self, state):
        
        # When the robot status_flags are 1 (SWAYING), you can publish the next step command.
        if (state.step_feedback.status_flags == 1 and not self.is_swaying):
            self.step_index += 1
            self.is_swaying = True
            print("Step " + str(self.step_index))
        elif (state.step_feedback.status_flags == 2):
            self.is_swaying = False
        
        is_right_foot = self.step_index % 2
        
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.STEP

        # k_effort is all 0s for full bdi control of all joints
        command.k_effort = [0] * 28
        
        # step_index should always be one for a step command
        command.step_params.desired_step.step_index = 1
        command.step_params.desired_step.foot_index = is_right_foot
        
        # duration has far as I can tell is not observed
        command.step_params.desired_step.duration * [Atlas Sim Interface](http://gazebosim.org/wiki/Tutorials/drcsim/2.5/atlas_sim_interface ) How to use the Atlas Sim Interface to command Atlas to walk dynamically or step statically.= 0.63
        
        # swing_height is not observed
        command.step_params.desired_step.swing_height = 0.1

        if self.step_index > 30:
            print(str(self.calculate_pose(self.step_index)))
        # Determine pose of the next step based on the number of steps we have taken
        command.step_params.desired_step.pose = self.calculate_pose(self.step_index)
        
        # Publish a new step command every time a state message is received
        self.asi_command.publish(command)
~~~

This method calculates the pose of a step around a circle, based on the current step_index

~~~
    # This method is used to calculate a pose of step based on the step_index
    # The step poses just cause the robot to walk in a circle
    def calculate_pose(self, step_index):
        # Right foot occurs on even steps, left on odd
        is_right_foot = step_index % 2
        is_left_foot = 1 - is_right_foot
        
        # There will be 60 steps to a circle, and so our position along the circle is current_step
        current_step = step_index % 60
        
        # yaw angle of robot around circle
        theta = current_step * math.pi / 30
           
        R = 2 # Radius of turn
        W = 0.3 # Width of stride
        
        # Negative for inside foot, positive for outside foot
        offset_dir = 1 - 2 * is_left_foot

        # Radius from center of circle to foot
        R_foot = R + offset_dir * W/2
        
        # X, Y position of foot
        X = R_foot * math.sin(theta)
        Y = (R - R_foot*math.cos(theta))
        
        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, theta)
        pose = Pose()
        pose.position.x = self.robot_position.x + X
        pose.position.y = self.robot_position.y + Y
        
        # The z position is observed for static walking, but the foot
        # will be placed onto the ground if the ground is lower than z
        pose.position.z = 0
        
        pose.orientation.x = Q[0]
        pose.orientation.y = Q[1]
        pose.orientation.z = Q[2]
        pose.orientation.w = Q[3]

        return pose
~~~
 
Main method to run walk. It checks if static is specified or not.

~~~
if __name__ == '__main__':
    rospy.init_node('walking_tutorial')
    walk = AtlasWalk()
    if len(sys.argv) > 0:
        walk.is_static = (sys.argv[-1] == "static")
    else:
        walk.is_static = False
    walk.walk()
~~~

# Running

Ensure that the above python file is executable

~~~
chmod +x ~/ros/atlas_sim_interface_tutorial/scripts/walk.py
~~~

Start up simulation

~~~
roslaunch drcsim_gazebo atlas_sandia_hands.launch
~~~

Rosrun the executable, specifying static if desired

Dynamic

~~~
rosrun atlas_sim_interface_tutorial walk.py
~~~

Static

~~~
rosrun atlas_sim_interface_tutorial walk.py static
~~~

## What you should see

Atlas should begin walking in a circle. Swiftly if it is dynamic behavior, or slowly if static behavior like the image below.

[[file:files/Asi_walk.png|800px]]
