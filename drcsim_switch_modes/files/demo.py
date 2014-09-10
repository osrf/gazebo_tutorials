#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('control_mode_switch')
import rospy
import time
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath

class Demo:

    def __init__(self):
        self.NUM_JOINTS = 28
        # Latest message from /atlas/atlas_sim_interface_state
        self.asis_msg = None
        # Latest message from /atlas/imu
        self.imu_msg = None

        # Set up publishers / subscribers
        self.ac_pub = rospy.Publisher('atlas/atlas_command', AtlasCommand, queue_size=1)
        self.asic_pub = rospy.Publisher('atlas/atlas_sim_interface_command', 
          AtlasSimInterfaceCommand, queue_size=1)
        self.asis_sub = rospy.Subscriber('atlas/atlas_sim_interface_state', 
          AtlasSimInterfaceState, self.state_cb)
        self.imu_sub = rospy.Subscriber('atlas/imu', 
          Imu, self.imu_cb)
        # Wait for subscribers to hook up, lest they miss our commands
        time.sleep(2.0)
        
    def state_cb(self, msg):
        self.asis_msg = msg

    def imu_cb(self, msg):
        self.imu_msg = msg
    
    def demo(self):
        # Step 1: Go to stand-prep pose under user control
        stand_prep_msg = AtlasCommand()
        # Always insert current time
        stand_prep_msg.header.stamp = rospy.Time.now()
        # Assign some position and gain values that will get us there.
        stand_prep_msg.position = [2.438504816382192e-05, 0.0015186156379058957, 9.983908967114985e-06, -0.0010675729718059301, -0.0003740221436601132, 0.06201673671603203, -0.2333149015903473, 0.5181407332420349, -0.27610817551612854, -0.062101610004901886, 0.00035181696875952184, -0.06218484416604042, -0.2332201600074768, 0.51811283826828, -0.2762000858783722, 0.06211360543966293, 0.29983898997306824, -1.303462266921997, 2.0007927417755127, 0.49823325872421265, 0.0003098883025813848, -0.0044272784143686295, 0.29982614517211914, 1.3034454584121704, 2.000779867172241, -0.498238742351532, 0.0003156556049361825, 0.004448802210390568]
        stand_prep_msg.velocity = [0.0] * self.NUM_JOINTS
        stand_prep_msg.effort = [0.0] * self.NUM_JOINTS
        stand_prep_msg.k_effort = [255] * self.NUM_JOINTS
        # Publish and give time to take effect
        print('[USER] Going to stand prep position...')
        self.ac_pub.publish(stand_prep_msg)
        time.sleep(2.0)
    
        # Step 2: Request BDI stand mode
        stand_msg = AtlasSimInterfaceCommand()
        # Always insert current time
        stand_msg.header.stamp = rospy.Time.now()
        # Tell it to stand
        stand_msg.behavior = stand_msg.STAND_PREP
        # Set k_effort = [255] to indicate that we still want all joints under user
        # control.  The stand behavior needs a few iterations before it start
        # outputting force values.
        stand_msg.k_effort = [255] * self.NUM_JOINTS
        # Publish and give time to take effect
        print('[USER] Warming up BDI stand...')
        self.asic_pub.publish(stand_msg)
        time.sleep(1.0)
        # Now switch to stand
        stand_msg.behavior = stand_msg.STAND
        # Set k_effort = [0] to indicate that we want all joints under BDI control
        stand_msg.k_effort = [0] * self.NUM_JOINTS
        # Publish and give time to take effect
        print('[BDI] Standing...')
        self.asic_pub.publish(stand_msg)
        time.sleep(2.0)
     
        # Step 3: Move the arms and head a little (not too much; don't want to fall
        # over)
        slight_movement_msg = AtlasCommand()
        # Always insert current time
        slight_movement_msg.header.stamp = rospy.Time.now()
        # Start with 0.0 and set values for the joints that we want to control
        slight_movement_msg.position = [0.0] * self.NUM_JOINTS
        slight_movement_msg.position[AtlasState.neck_ry] = -0.1
        slight_movement_msg.position[AtlasState.l_arm_ely] = 2.1
        slight_movement_msg.position[AtlasState.l_arm_wrx] = -0.1
        slight_movement_msg.position[AtlasState.r_arm_ely] = 2.1
        slight_movement_msg.position[AtlasState.r_arm_wrx] = -0.1
        slight_movement_msg.velocity = [0.0] * self.NUM_JOINTS
        slight_movement_msg.effort = [0.0] * self.NUM_JOINTS
        slight_movement_msg.kp_position = [20.0, 4000.0, 2000.0, 20.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0]
        slight_movement_msg.ki_position = [0.0] * self.NUM_JOINTS
        slight_movement_msg.kd_position = [0.0] * self.NUM_JOINTS
        # Bump up kp_velocity to reduce the jerkiness of the transition
        stand_prep_msg.kp_velocity = [50.0] * self.NUM_JOINTS
        slight_movement_msg.i_effort_min = [0.0] * self.NUM_JOINTS
        slight_movement_msg.i_effort_max = [0.0] * self.NUM_JOINTS 
        # Set k_effort = [1] for the joints that we want to control.
        # BDI has control of the other joints
        slight_movement_msg.k_effort = [0] * self.NUM_JOINTS
        slight_movement_msg.k_effort[AtlasState.neck_ry] = 255
        slight_movement_msg.k_effort[AtlasState.l_arm_ely] = 255
        slight_movement_msg.k_effort[AtlasState.l_arm_wrx] = 255
        slight_movement_msg.k_effort[AtlasState.r_arm_ely] = 255
        slight_movement_msg.k_effort[AtlasState.r_arm_wrx] = 255
        # Publish and give time to take effect
        print('[USER/BDI] Command neck and arms...')
        self.ac_pub.publish(slight_movement_msg)
        time.sleep(2.0)
    
        # Step 4: Request BDI walk
        walk_msg = AtlasSimInterfaceCommand()
        # Always insert current time
        walk_msg.header.stamp = rospy.Time.now()
        # Tell it to walk
        walk_msg.behavior = walk_msg.WALK
        walk_msg.walk_params.use_demo_walk = False
        # Fill in some steps
        for i in range(4):
            step_data = AtlasBehaviorStepData()
            # Steps are indexed starting at 1
            step_data.step_index = i+1
            # 0 = left, 1 = right
            step_data.foot_index = i%2
            # 0.3 is a good number
            step_data.swing_height = 0.3
            # 0.63 is a good number
            step_data.duration = 0.63
            # We'll specify desired foot poses in ego-centric frame then
            # transform them into the robot's world frame.
            # Match feet so that we end with them together
            step_data.pose.position.x = (1+i/2)*0.25
            # Step 0.15m to either side of center, alternating with feet
            step_data.pose.position.y = 0.15 if (i%2==0) else -0.15
            step_data.pose.position.z = 0.0
            # Point those feet straight ahead
            step_data.pose.orientation.x = 0.0
            step_data.pose.orientation.y = 0.0
            step_data.pose.orientation.z = 0.0
            step_data.pose.orientation.w = 1.0
            # Transform this foot pose according to robot's
            # current estimated pose in the world, which is a combination of IMU
            # and internal position estimation.
            # http://www.ros.org/wiki/kdl/Tutorials/Frame%20transformations%20%28Python%29
            f1 = posemath.fromMsg(step_data.pose)
            f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.imu_msg.orientation.x,
                                                       self.imu_msg.orientation.y,
                                                       self.imu_msg.orientation.z,
                                                       self.imu_msg.orientation.w),
                             PyKDL.Vector(self.asis_msg.pos_est.position.x,
                                          self.asis_msg.pos_est.position.y,
                                          self.asis_msg.pos_est.position.z))
            f = f2 * f1
            step_data.pose = posemath.toMsg(f)
            walk_msg.walk_params.step_queue[i] = step_data
        # Use the same k_effort from the last step, to retain user control over some
        # joints. BDI has control of the other joints.
        walk_msg.k_effort = slight_movement_msg.k_effort
        # Publish and give time to take effect
        print('[USER/BDI] Walking...')
        self.asic_pub.publish(walk_msg)
        time.sleep(6.0)

        # Step 5: Go back to home pose under user control
        home_msg = AtlasCommand()
        # Always insert current time
        home_msg.header.stamp = rospy.Time.now()
        # Assign some position and gain values that will get us there.
        home_msg.position = [0.0] * self.NUM_JOINTS
        home_msg.velocity = [0.0] * self.NUM_JOINTS
        home_msg.effort = [0.0] * self.NUM_JOINTS
        home_msg.kp_position = [20.0, 4000.0, 2000.0, 20.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0]
        home_msg.ki_position = [0.0] * self.NUM_JOINTS
        home_msg.kd_position = [0.0] * self.NUM_JOINTS
        # Bump up kp_velocity to reduce the jerkiness of the transition
        home_msg.kp_velocity = [50.0] * self.NUM_JOINTS
        home_msg.i_effort_min = [0.0] * self.NUM_JOINTS
        home_msg.i_effort_max = [0.0] * self.NUM_JOINTS 
        # Set k_effort = [1] to indicate that we want all joints under user control
        home_msg.k_effort = [255] * self.NUM_JOINTS
        # Publish and give time to take effect
        print('[USER] Going to home position...')
        self.ac_pub.publish(home_msg)
        time.sleep(2.0)
    
if __name__ == '__main__':
    rospy.init_node('control_mode_switch')
    d = Demo()
    d.demo()

