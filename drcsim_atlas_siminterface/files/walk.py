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
          True, None, queue_size=1)
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None, queue_size=1)
        
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
        # End of dynamic walk behavior
       
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
        # End of static walk behavior
        
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
