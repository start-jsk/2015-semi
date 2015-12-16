#! /usr/bin/env python

import time

robot.checkEncoders()
robot.goInitial()

robot.startImpedance('rarm')
time.sleep(1)
print "start teaching"
pos_list = []
rpy_list = []
for i in range(0,50):
  pos = robot.getCurrentPosition('RARM_JOINT5')  
  rpy = robot.getReferenceRPY('RARM_JOINT5')
  pos_list.append(pos)
  rpy_list.append(rpy)
  time.sleep(0.1)
print "end teaching"
robot.stopImpedance('rarm')

time.sleep(1)
print "start playback"
tm =0.1 
for i in range(0,50):
    robot.setTargetPose('rarm', pos_list[i], rpy_list[i], tm)
    time.sleep(0.1)
