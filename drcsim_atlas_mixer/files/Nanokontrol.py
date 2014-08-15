#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix
#
# slightly hacked up by Morgan Quigley, November 2012 
#
# MQ todo: look into somehow querying the nanokontrol on program start, to
# find out where the sliders/knobs are, and thus avoid having a jump when
# they are first moved. possibly useful sources of information:
#  http://www.fumph.com/ensoniq_mr/mrsysex.txt
#  http://madamebutterface.com/assets/documents/MIDI%201.0%20Detailed%20Specification.pdf
#  http://indra.com/~cliffcan/01midi.htm
# http://www.korg.com/uploads/Support/nanoKONTROL_MIDIChart_EJ2_634396687471680000.pdf
# need to read more about pygame.midi to learn how to craft messages and do
# midi "transactions" or at least emulate them poorly on startup.
# 


import roslib; roslib.load_manifest('atlas_teleop')
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *

control_axes = [{
  # mode 1, sliders
   2:  0,  3:  1,  4:  2,  5:  3,  6:  4,  8:  5,  9:  6, 12:  7, 13:  8,
  # mode 1, knobs
  14:  9, 15: 10, 16: 11, 17: 12, 18: 13, 19: 14, 20: 15, 21: 16, 22: 17,
  },{
  # mode 2, sliders
  42:  0, 43:  1, 50:  2, 51:  3, 52:  4, 53:  5, 54:  6, 55:  7, 56:  8,
  # mode 2, knobs
  57:  9, 58: 10, 59: 11, 60: 12, 61: 13, 62: 14, 63: 15, 65: 16, 66: 17,
  },{
  # mode 3, sliders
  85:  0, 86:  1, 87:  2, 88:  3, 89:  4, 90:  5, 91:  6, 92:  7, 93:  8,
  # mode 3, knobs
  94:  9, 95: 10, 96: 11, 97: 12, 102: 13, 103: 14, 104: 15, 105: 16, 106: 17,
  },{
  # mode 4, sliders
  7: 0, 263: 1, 519: 2, 775: 3, 1031: 4, 1287: 5, 1543: 6, 1799: 7, 2055: 8,
  # mode 4, knobs
  10: 9, 266: 10, 522: 11, 778: 12, 1034: 13, 1290: 14, 1546: 15, 1802: 16,
  2058: 17,
  }]

control_buttons = [[
  # mode 1
  # up, down
  23, 33, 24, 34, 25, 35, 26, 36, 27, 37, 28, 38, 29, 39, 30, 40, 31, 41,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
],[
  # mode 2
  # up, down
  67, 76, 68, 77, 69, 78, 70, 79, 71, 80, 72, 81, 73, 82, 74, 83, 75, 84,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
],[
  # mode 3
  # up, down
  107, 116, 108, 117, 109, 118, 110, 119, 111, 120, 112, 121, 113, 122, 114, 123, 115, 124,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
],[
  # mode 4
  # up, down
  16, 17, 272, 273, 528, 529, 784, 785, 1040, 1041, 1296, 1297, 1552, 1553, 1808, 1809, 2064, 2065,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
]]

def main():
   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      print "No MIDI devices detected"
      exit(-1)
   print "Found %d MIDI devices" % devices

   if len(sys.argv) > 1:
      input_dev = int(sys.argv[1])
   else:
      print "no input device supplied. will try to use default device."
      input_dev = pygame.midi.get_default_input_id()
      if input_dev == -1:
         print "No default MIDI input device"
         exit(-1)
   print "Using input device %d" % input_dev

   controller = pygame.midi.Input(input_dev)
   print "Opened it"

   rospy.init_node('kontrol')
   pub = rospy.Publisher('joy', Joy, latch=True)

   m = Joy()
   m.axes = [ 0 ] * 18
   m.buttons = [ 0 ] * 25
   mode = None

   p = False

   while not rospy.is_shutdown():
      m.header.stamp = rospy.Time.now()
      # count the number of events that are coalesced together
      c = 0
      while controller.poll():
         c += 1
         data = controller.read(1)
         #print data
         # loop through events received
         for event in data:
            control = event[0]
            timestamp = event[1]

            # look for continuous controller commands
            if (control[0] & 0xF0) == 176:
               control_id = control[1] | ((control[0] & 0x0F) << 8)

               # guess initial mode based on command
               if mode is None:
                  candidate = None
                  for index, control_axis in enumerate(control_axes):
                     if control_id in control_axis:
                        if candidate is not None:
                           candidate = None
                           break
                        candidate = index
                  for index, control_button in enumerate(control_buttons):
                     if control_id in control_button:
                        if candidate is not None:
                           candidate = None
                           break
                        candidate = index
                  mode = candidate
                  if mode is None:
                     print 'skipped because mode is yet unknown'
                     continue

               if control_id in control_axes[mode]:
                  control_val = float(control[2]) / 127.0 # mq haxx  float(control[2] - 63) / 63.0
                  if control_val < 0.0:
                     control_val = 0.0
                  if control_val > 1.0:
                     control_val = 1.0

                  axis = control_axes[mode][control_id]
                  m.axes[axis] = control_val
                  p = True

               if control_id in control_buttons[mode]:
                  button = control_buttons[mode].index(control_id)
                  if control[2] != 0:
                     m.buttons[button] = 1
                  else:
                     m.buttons[button] = 0
                  p = True
            # look for mode commands
            elif control[0] == 79:
               mode = control[1]
               m.buttons[24] = mode
               p = True

      if p:
         pub.publish(m)
         p = False

      rospy.sleep(0.01) # 100hz max
                  


if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
