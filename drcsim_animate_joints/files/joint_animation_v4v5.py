#!/usr/bin/env python

import roslib; roslib.load_manifest('joint_animation_tutorial')
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control')

    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()


    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = "atlas::pelvis"

    jt.joint_names.append("atlas::back_bkz" )
    jt.joint_names.append("atlas::back_bky" )
    jt.joint_names.append("atlas::back_bkx" )
    jt.joint_names.append("atlas::neck_ry"  )
    jt.joint_names.append("atlas::l_leg_hpz")
    jt.joint_names.append("atlas::l_leg_hpx")
    jt.joint_names.append("atlas::l_leg_hpy")
    jt.joint_names.append("atlas::l_leg_kny")
    jt.joint_names.append("atlas::l_leg_aky")
    jt.joint_names.append("atlas::l_leg_akx")
    jt.joint_names.append("atlas::r_leg_akx")
    jt.joint_names.append("atlas::r_leg_aky")
    jt.joint_names.append("atlas::r_leg_kny")
    jt.joint_names.append("atlas::r_leg_hpy")
    jt.joint_names.append("atlas::r_leg_hpx")
    jt.joint_names.append("atlas::r_leg_hpz")
    jt.joint_names.append("atlas::l_arm_elx")
    jt.joint_names.append("atlas::l_arm_ely")
    jt.joint_names.append("atlas::l_arm_shz")
    jt.joint_names.append("atlas::l_arm_shx")
    jt.joint_names.append("atlas::l_arm_wry")
    jt.joint_names.append("atlas::l_arm_wrx")
    jt.joint_names.append("atlas::l_arm_wry2")
    jt.joint_names.append("atlas::r_arm_elx")
    jt.joint_names.append("atlas::r_arm_ely")
    jt.joint_names.append("atlas::r_arm_shz")
    jt.joint_names.append("atlas::r_arm_shx")
    jt.joint_names.append("atlas::r_arm_wry")
    jt.joint_names.append("atlas::r_arm_wrx")
    jt.joint_names.append("atlas::r_arm_wry2")

    # turn off pid control so it does not interfere with trajectory control
    rospy.loginfo("Turning off PID control")
    for i in jt.joint_names:
        rospy.set_param('/atlas_controller/gains/' + i[7:] + '/p', 0)
        rospy.set_param('/atlas_controller/gains/' + i[7:] + '/i', 0)
        rospy.set_param('/atlas_controller/gains/' + i[7:] + '/d', 0)

    rospy.sleep(1.0)
    # set Atlas to User mode
    mode_pub = rospy.Publisher('/atlas/control_mode', String, queue_size=1)
    mode_pub.publish(String("ragdoll"))
    rospy.sleep(1.0)
    mode_pub.publish(String("User"))
    rospy.sleep(1.0)

    n = 1500
    dt = 0.01
    rps = 0.05
    for i in range (n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 = -0.5*math.sin(2*theta)
        x2 =  0.5*math.sin(1*theta)

        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
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
