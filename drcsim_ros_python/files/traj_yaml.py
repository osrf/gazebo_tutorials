#!/usr/bin/env python
import roslib; roslib.load_manifest('tutorial_atlas_control')
import rospy, yaml, sys
from osrf_msgs.msg import JointCommands
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil

atlasJointNames = [
  'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
  'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
  'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
  'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
  'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']

currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

if __name__ == '__main__':
  # first make sure the input arguments are correct
  if len(sys.argv) != 3:
    print "usage: traj_yaml.py YAML_FILE TRAJECTORY_NAME"
    print "  where TRAJECTORY is a dictionary defined in YAML_FILE"
    sys.exit(1)
  traj_yaml = yaml.load(file(sys.argv[1], 'r'))
  traj_name = sys.argv[2]
  if not traj_name in traj_yaml:
    print "unable to find trajectory %s in %s" % (traj_name, sys.argv[1])
    sys.exit(1)
  traj_len = len(traj_yaml[traj_name])

  # Setup subscriber to atlas states
  rospy.Subscriber("/atlas/joint_states", JointState, jointStatesCallback)

  # initialize JointCommands message
  command = JointCommands()
  command.name = list(atlasJointNames)
  n = len(command.name)
  command.position     = zeros(n)
  command.velocity     = zeros(n)
  command.effort       = zeros(n)
  command.kp_position  = zeros(n)
  command.ki_position  = zeros(n)
  command.kd_position  = zeros(n)
  command.kp_velocity  = zeros(n)
  command.i_effort_min = zeros(n)
  command.i_effort_max = zeros(n)

  # now get gains from parameter server
  rospy.init_node('tutorial_atlas_control')
  for i in xrange(len(command.name)):
    name = command.name[i]
    command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/p')
    command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i')
    command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/d')
    command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
    command.i_effort_min[i] = -command.i_effort_max[i]

  # set up the publisher
  pub = rospy.Publisher('/atlas/joint_commands', JointCommands)

  # for each trajectory
  for i in xrange(0, traj_len):
    # get initial joint positions
    initialPosition = array(currentJointState.position)
    # get joint commands from yaml
    y = traj_yaml[traj_name][i]
    # first value is time duration
    dt = float(y[0])
    # subsequent values are desired joint positions
    commandPosition = array([ float(x) for x in y[1].split() ])
    # desired publish interval
    dtPublish = 0.02
    n = ceil(dt / dtPublish)
    for ratio in linspace(0, 1, n):
      interpCommand = (1-ratio)*initialPosition + ratio * commandPosition
      command.position = [ float(x) for x in interpCommand ]
      pub.publish(command)
      rospy.sleep(dt / float(n))
