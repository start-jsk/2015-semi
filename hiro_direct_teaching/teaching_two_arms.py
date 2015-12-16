#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import time
import threading

class GetKeyInput(threading.Thread):
    def __init__(self):
        super(GetKeyInput, self).__init__()
        self.key = '0'
        rospy.Subscriber("input_key", String, self.sub_input_key)

    def run(self):
        rospy.spin()
        self.exit()

    def sub_input_key(self, input_key):
        self.key = input_key.data
        
    def get_key(self):
        return self.key
        
def main():
    #rospy.init_node('hiro_teaching_two_arms', anonymous=True)
    r = rospy.Rate(10)
    
    key_input_th = GetKeyInput()
    key_input_th.start()

    robot.checkEncoders()
    robot.goInitial()

    robot.startImpedance('rarm')
    robot.startImpedance('larm')
    time.sleep(1)
    print "Enter s to start teaching"
    while key_input_th.get_key() != 's' and not rospy.is_shutdown():
        r.sleep()
    print "start teaching"
    print "Enter e to end teaching"
    whole_angle_list = []
    sample_num = 0
    while key_input_th.get_key() != 'e' and not rospy.is_shutdown():
        angle = robot.getActualState().angle   
        whole_angle_list.append(angle)
        sample_num += 1
        r.sleep()
    print "end teaching"
    robot.stopImpedance('rarm')
    robot.stopImpedance('larm')

    time.sleep(1)
    robot.goInitial()
    target_angle_rarm_list = []
    target_angle_larm_list = []
    tm = []
    for i in range(0,sample_num):
        target_angle_rarm_list.append(whole_angle_list[i][3:9])
        target_angle_larm_list.append(whole_angle_list[i][9:15])
        tm.append(0.1)
    print "start playback"
    robot.playPatternOfGroup('rarm',target_angle_rarm_list,tm)
    robot.playPatternOfGroup('larm',target_angle_larm_list,tm)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
