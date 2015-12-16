#! /usr/bin/env python

import time

robot.checkEncoders()
robot.goInitial()

robot.startImpedance('rarm')
time.sleep(1)
print "start teaching"
whole_angle_list = []
for i in range(0,50):
    angle = robot.getActualState().angle   
    whole_angle_list.append(angle)
    time.sleep(0.1)
print "end teaching"
robot.stopImpedance('rarm')

time.sleep(1)
robot.goInitial()
target_angle_rarm_list = []
tm = []
for i in range(0,50):
    target_angle_rarm_list.append(whole_angle_list[i][3:9])
    tm.append(0.1)
print "start playback"
robot.playPatternOfGroup('rarm',target_angle_rarm_list,tm)
