#!/usr/bin/env python
import roslib; roslib.load_manifest('atlas_teleop')
import rospy, sys, yaml
from osrf_msgs.msg import JointCommands
from sensor_msgs.msg import Joy
g_jc_pub = None
g_jc_lh_pub = None
g_jc_rh_pub = None
g_jc = JointCommands()
g_jc_rh = JointCommands()
g_jc_lh = JointCommands()
g_vec = [ ]
g_knobs = "unknown"
g_jc_rh = JointCommands()
g_vec_lh = [ ]
g_vec_rh = [ ]
# need to figure out a reasonable way to blend grasps composed of different
# origins... for now, it will ignore the 'origin' data and just use zero...
g_grasps = { 'cyl': { 'origin':[0] * 12,
                      'grasp': [    0, 1.5, 1.7,   0,  1.5,   1.7,    0,  1.5,   1.7, -0.2, 0.8, 1.7]},
             'sph': { 'origin':[  0.0,   0,   0, 0.1,    0,     0,  0.0,    0,     0,    0,   0,   0],
                      'grasp': [ -1.0, 1.4, 1.4, 0.0,  1.4,   1.4,  1.0,  1.4,   1.4,    0, 0.7, 0.7]},
             'par': { 'origin':[  0.0, 0.0, 0.0,   0, -1.4,  -1.4,  0.0, -1.4,  -1.4,  0.5, 0.0, 0.0],
                      'grasp': [  0.0, 1.4, 1.2,   0, -1.4,  -1.4,  0.0, -1.4,  -1.4,  0.5, 0.7, 0.7]} }

def joy_cb(msg):
  global g_jc_pub, g_jc
  g_jc.position = [0] * 28
  g_jc_lh.position = [0] * 12
  g_jc_rh.position = [0] * 12
  for x in xrange(0, 28): # start at origin
    g_jc.position[x] = g_vec[0][x]
  for slider in xrange(1, 9): # process sliders
    #print msg.axes[slider-1]
    #print g_vec[slider]
    slider_pos = msg.axes[slider-1]
    for axis in xrange(0, 28): # do the whole robot other than hands...
      g_jc.position[axis] += slider_pos * g_vec[slider][axis]
    for axis in xrange(0, 12): # do the hands... this is so bad.
      #print g_grasps[g_vec_lh[slider][0]]['grasp'][axis]
      lh_dir = g_grasps[g_vec_lh[slider][0]]['grasp'][axis] 
      rh_dir = g_grasps[g_vec_rh[slider][0]]['grasp'][axis] 
      #print "rh: axis %d dir %f mix %f target %d" % (axis, rh_dir, slider_pos, g_vec_rh[slider][1])
      g_jc_lh.position[axis] += slider_pos * lh_dir * g_vec_lh[slider][1]
      g_jc_rh.position[axis] += slider_pos * rh_dir * g_vec_rh[slider][1]
  for x in xrange(0, 6): # process manual-override 
    if msg.buttons[x * 2] == 1:
      if g_knobs == "right_arm":
        g_jc.position[x + 22] = (msg.axes[x + 9] - 0.5) * 3.14 * 2
      elif g_knobs == "right_leg":
        g_jc.position[x + 10] = (msg.axes[x + 9] - 0.5) * 3.14 * 2
      elif g_knobs == 'back':
        g_jc.position[x] = (msg.axes[x+9] - 0.5) * 3.14
  g_jc_lh_pub.publish(g_jc_lh)
  g_jc_rh_pub.publish(g_jc_rh)
  g_jc_pub.publish(g_jc)
  s = ""
  for x in xrange(0, 28):
    s += "%.3f " % g_jc.position[x]
  print s

if __name__ == '__main__':
  if len(rospy.myargv()) != 2:
    print "usage: atlas_teleop.py FILENAME.yaml"
    sys.exit(1)
  vec = yaml.load(file(rospy.myargv()[1], 'r'))
  print "loaded %s" % rospy.myargv()[1]
  #print len(vec)
  print vec
  g_vec = [[0] * 28] * 10 # controller has 9 sliders, plus origin
  g_vec_lh = [[ 'cyl', 0 ]] * 10
  g_vec_rh = [[ 'cyl', 0 ]] * 10
  for key, value in vec.iteritems():
    if key == "knobs":
      g_knobs = value
      continue
    idx = int(key)
    if idx < 0 or idx > 9:
      print "bogus vector index: %d" % idx
      continue
    print "loaded %d:  %s" % (idx, value)
    g_vec[idx] = [ float(x) for x in value.split()[0:28] ]
    g_vec_lh[idx] = [ value.split()[28], float(value.split()[29]) ]
    g_vec_rh[idx] = [ value.split()[30], float(value.split()[31]) ]
  print g_vec
  print g_vec_lh
  print g_vec_rh
  rospy.init_node('atlas_teleop')
  g_jc.name = ['atlas::back_lbz', 'atlas::back_mby',    # 0
               'atlas::back_ubx', 'atlas::neck_ay',
               'atlas::l_leg_uhz', 'atlas::l_leg_mhx',  # 4
               'atlas::l_leg_lhy', 'atlas::l_leg_kny',
               'atlas::l_leg_uay', 'atlas::l_leg_lax',
               'atlas::r_leg_uhz', 'atlas::r_leg_mhx',  # 10
               'atlas::r_leg_lhy', 'atlas::r_leg_kny',
               'atlas::r_leg_uay', 'atlas::r_leg_lax',
               'atlas::l_arm_usy', 'atlas::l_arm_shx',  # 16
               'atlas::l_arm_ely', 'atlas::l_arm_elx',
               'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
               'atlas::r_arm_usy', 'atlas::r_arm_shx',  # 22
               'atlas::r_arm_ely', 'atlas::r_arm_elx',
               'atlas::r_arm_uwy', 'atlas::r_arm_mwx']
  ''' current settings from launch file:
  g_jc.kp_position = [20, 4000, 2000, 20,  # back
                      5, 100, 2000, 1000, 900, 300, # left leg
                      5, 100, 2000, 1000, 900, 300, # right leg
                      2000, 1000, 200, 200, 50, 100, # left arm
                      2000, 1000, 200, 200, 50, 100] # right arm
  g_jc.kd_position = [0.1, 2.0, 1.0, 1.0, # back
                      0.01, 1.0, 10.0, 10.0, 2.0, 1.0, # left leg
                      0.01, 1.0, 10.0, 10.0, 2.0, 1.0, # right leg
                      3.0, 20.0, 3.0, 3.0, 0.1, 0.2, # left arm
                      3.0, 20.0, 3.0, 3.0, 0.1, 0.2] # right arm
  '''
  g_jc.kp_position = [4000, 4000, 14000, 20,  # back
                      2000, 2000, 2000, 1000, 900, 300, # left leg
                      2000, 2000, 2000, 1000, 900, 300, # right leg
                      2000, 1000, 200, 200, 50, 100, # left arm
                      2000, 1000, 200, 200, 50, 100] # right arm
  g_jc.kd_position = [100.0, 100.0, 100.0, 1.0, # back
                      10.0, 10.0, 10.0, 10.0, 2.0, 1.0, # left leg
                      10.0, 10.0, 10.0, 10.0, 2.0, 1.0, # right leg
                      20.0, 20.0, 3.0, 3.0, 0.1, 0.2, # left arm
                      20.0, 20.0, 3.0, 3.0, 0.1, 0.2] # right arm
  g_jc.position = [0] * 28
  g_jc_pub = rospy.Publisher('atlas/joint_commands', JointCommands)

  g_jc_rh.name = ["f0_j0", "f0_j1", "f0_j2",
                  "f1_j0", "f1_j1", "f1_j2",
                  "f2_j0", "f2_j1", "f2_j2",
                  "f3_j0", "f3_j1", "f3_j2"]
  g_jc_lh.name = ["f0_j0", "f0_j1", "f0_j2",
                  "f1_j0", "f1_j1", "f1_j2",
                  "f2_j0", "f2_j1", "f2_j2",
                  "f3_j0", "f3_j1", "f3_j2"]
  g_jc_lh.position = [0] * 12
  g_jc_lh_pub = rospy.Publisher('sandia_hands/l_hand/joint_commands', 
                                JointCommands) # ohh i hate lines >80 chars...
  g_jc_rh_pub = rospy.Publisher('sandia_hands/r_hand/joint_commands',
                                JointCommands)

  joy_sub = rospy.Subscriber('joy', Joy, joy_cb)
  rospy.spin()
